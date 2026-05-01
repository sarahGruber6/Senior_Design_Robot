// full_integration_v1.ino — ESP32-S3 dual-core
// Core0=MQTT/WiFi  Core1=LiDAR+motors
// queues: scanQueue C1->C0 (overwrite), encoderQueue C1->C0, cmdQueue C0->C1

#include "YDLidarG2.h"
#include <WiFi.h>
#include "esp_wifi.h"
#include <ArduinoMqttClient.h>
#include "config.h"
#ifdef __has_include
  #if __has_include("config_local.h")
    #include "config_local.h"
  #endif
#endif

enum Mode { SENSOR_ONLY = 0, FULL_INTEGRATION = 1 };
static volatile Mode currentMode = FULL_INTEGRATION;

static const uint16_t LIDAR_MIN_MM      = 150;
static const uint16_t LIDAR_MAX_MM      = 10000;
static const uint16_t MIN_VALID_BINS    = 200;
static const uint32_t ENCODER_PERIOD_MS = 50;
static const uint32_t STATUS_PERIOD_MS  = 200;
static const uint32_t CORR_PERIOD_MS    = 50;
static const uint32_t DEFAULT_MOTOR_TIMEOUT_MS = 500;
static const size_t   PAYLOAD_SIZE      = 4 + 4 + 4 + 2 + 360 * 2;  // SCN3

struct ScanMsg {
    uint16_t bins[360];
    uint16_t validBins;
    uint32_t tsMs;
    int32_t leftTicks;
    int32_t rightTicks;
};

struct EncoderMsg {
    int32_t  leftTicks;
    int32_t  rightTicks;
    uint32_t tsMs;
};

struct MotorCmd {
    float v;
    float w;
    uint32_t ttlMs;
    int   seq;
    char  cmdId[64];
};

static QueueHandle_t scanQueue;     // capacity 1 (overwrite), Core1->Core0
static QueueHandle_t encoderQueue;  // capacity 2, Core1->Core0
static QueueHandle_t cmdQueue;      // capacity 4, Core0->Core1

static const int8_t QUAD_TABLE[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static volatile int32_t leftTicks  = 0;
static volatile int32_t rightTicks = 0;
static volatile uint8_t leftPrev   = 0;
static volatile uint8_t rightPrev  = 0;
static portMUX_TYPE     encoderMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR leftISR() {
    uint8_t s = ((digitalRead(LEFT_A) ? 1 : 0) << 1) | (digitalRead(LEFT_B) ? 1 : 0);
    leftTicks += QUAD_TABLE[(leftPrev << 2) | s];
    leftPrev = s;
}

void IRAM_ATTR rightISR() {
    uint8_t s = ((digitalRead(RIGHT_A) ? 1 : 0) << 1) | (digitalRead(RIGHT_B) ? 1 : 0);
    rightTicks += QUAD_TABLE[(rightPrev << 2) | s];
    rightPrev = s;
}

static portMUX_TYPE     motorMux  = portMUX_INITIALIZER_UNLOCKED;
static volatile int16_t sharedLPWM = 0;
static volatile int16_t sharedRPWM = 0;

static volatile bool obstacleBlocked = false;

static int16_t  cmdLPWM        = 0;
static int16_t  cmdRPWM        = 0;
static float    activeCmdV     = 0.0f;
static bool     straightMode   = false;
static int32_t  leftTicksRef   = 0;
static int32_t  rightTicksRef  = 0;
static uint32_t lastMotorCmdMs = 0;
static uint32_t motorTimeoutMs = DEFAULT_MOTOR_TIMEOUT_MS;
static uint32_t lastCorrMs     = 0;
static uint32_t lastEncoderMs  = 0;
static volatile uint32_t scansRejected = 0;  // written Core1, read Core0 (volatile ok)

static HardwareSerial lidarSerial(1);
static YDLidarG2      lidar(lidarSerial, LIDAR_MOTOR_PIN);
static bool           scanning = false;

static const byte MAC[] = {0x10, 0x51, 0xDB, 0x37, 0x4F, 0x14};
static WiFiClient net;
static MqttClient mqtt(net);
static uint32_t   publishSeq    = 0;
static uint32_t   scansOk       = 0;
static uint32_t   motorCmdsRx   = 0;
static const uint8_t RECENT_COMMAND_ID_COUNT = 8;
static String     recentCmdIds[RECENT_COMMAND_ID_COUNT];
static uint8_t    recentCmdIdSlot = 0;

static bool extractJsonStringField(const char* payload, const char* key, char* out, size_t outSize) {
    char quotedKey[32];
    snprintf(quotedKey, sizeof(quotedKey), "\"%s\"", key);

    const char* p = strstr(payload, quotedKey);
    if(!p) return false;

    p = strchr(p + strlen(quotedKey), ':');
    if(!p) return false;
    p++;
    while(*p == ' ' || *p == '\t') p++;
    if(*p != '"') return false;
    p++;

    const char* e = strchr(p, '"');
    if(!e) return false;

    size_t n = min((size_t)(e - p), outSize - 1);
    memcpy(out, p, n);
    out[n] = '\0';
    return true;
}

static bool hasSeenCommandId(const char* cmdId) {
    if(strlen(cmdId) == 0) return false;
    for(uint8_t i = 0; i < RECENT_COMMAND_ID_COUNT; i++) {
        if(recentCmdIds[i] == String(cmdId)) return true;
    }
    return false;
}

static void rememberCommandId(const char* cmdId) {
    if(strlen(cmdId) == 0) return;
    recentCmdIds[recentCmdIdSlot] = String(cmdId);
    recentCmdIdSlot = (recentCmdIdSlot + 1) % RECENT_COMMAND_ID_COUNT;
}

// ========== Motor helpers (Core 1) ==========

static void setupMotors() {
    ledcAttach(LEFT_PWM_PIN,  1000, 8);
    ledcAttach(RIGHT_PWM_PIN, 1000, 8);
    pinMode(LEFT_DIR_PIN,  OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    ledcWrite(LEFT_PWM_PIN,  0);
    ledcWrite(RIGHT_PWM_PIN, 0);
    digitalWrite(LEFT_DIR_PIN,  LOW);
    digitalWrite(RIGHT_DIR_PIN, LOW);
}

static void setMotor(int16_t lPWM, int16_t rPWM) {
    lPWM = constrain(lPWM, -255, 255);
    rPWM = constrain(rPWM, -255, 255);
    digitalWrite(LEFT_DIR_PIN,  lPWM < 0 ? HIGH : LOW);
    ledcWrite(LEFT_PWM_PIN,     (uint8_t)abs(lPWM));
    digitalWrite(RIGHT_DIR_PIN, rPWM < 0 ? LOW : HIGH);
    ledcWrite(RIGHT_PWM_PIN,    (uint8_t)abs(rPWM));
    portENTER_CRITICAL(&motorMux);
    sharedLPWM = lPWM;
    sharedRPWM = rPWM;
    portEXIT_CRITICAL(&motorMux);
}

static void applyMotorCmd(const MotorCmd& cmd) {
    int16_t base = (int16_t)(K_V * cmd.v);
    int16_t turn = (int16_t)(K_W * cmd.w);
    int16_t lPWM = (int16_t)(base * LEFT_TRIM) - turn;
    int16_t rPWM = base + turn;
    setMotor(lPWM, rPWM);
    cmdLPWM = lPWM;
    cmdRPWM = rPWM;
    activeCmdV = cmd.v;
    portENTER_CRITICAL(&encoderMux);
    leftTicksRef  = leftTicks;
    rightTicksRef = rightTicks;
    portEXIT_CRITICAL(&encoderMux);
    straightMode   = (fabsf(cmd.w) < 0.05f) && (fabsf(cmd.v) > 0.01f);
    lastCorrMs     = millis();
    lastMotorCmdMs = millis();
    motorTimeoutMs = cmd.ttlMs;
}

static void checkMotorTimeout() {
    if(currentMode == SENSOR_ONLY) return;
    if(millis() - lastMotorCmdMs > motorTimeoutMs) {
        setMotor(0, 0);
        cmdLPWM = cmdRPWM = 0;
        activeCmdV = 0.0f;
        straightMode = false;
    }
}

static bool isForwardMotion(float v) {
    return OBSTACLE_FORWARD_V_SIGN >= 0 ? v > 0.01f : v < -0.01f;
}

static void updateDrift() {
    if(!straightMode || (cmdLPWM == 0 && cmdRPWM == 0)) return;
    if(millis() - lastCorrMs < CORR_PERIOD_MS) return;
    lastCorrMs = millis();

    int32_t L, R;
    portENTER_CRITICAL(&encoderMux);
    L = leftTicks;
    R = rightTicks;
    portEXIT_CRITICAL(&encoderMux);

    int32_t dL  = L - leftTicksRef;
    int32_t dR  = R - rightTicksRef;
    long    avg = (abs(dL) + abs(dR)) / 2;
    if(avg < 5) return;

    float rel    = (float)(dL - dR) / (float)avg;
    float factor = constrain(1.0f - K_CORRECT * rel, 0.5f, 2.0f);
    setMotor((int16_t)(cmdLPWM * factor), cmdRPWM);
}

// ========== Encoder setup (Core 1) ==========

static void setupEncoders() {
    pinMode(LEFT_A,  INPUT_PULLUP);
    pinMode(LEFT_B,  INPUT_PULLUP);
    pinMode(RIGHT_A, INPUT_PULLUP);
    pinMode(RIGHT_B, INPUT_PULLUP);
    leftPrev  = ((digitalRead(LEFT_A)  ? 1 : 0) << 1) | (digitalRead(LEFT_B)  ? 1 : 0);
    rightPrev = ((digitalRead(RIGHT_A) ? 1 : 0) << 1) | (digitalRead(RIGHT_B) ? 1 : 0);
    attachInterrupt(digitalPinToInterrupt(LEFT_A),  leftISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_B),  leftISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_B), rightISR, CHANGE);
    Serial.println("[ENC] interrupts attached");
}

// ========== LiDAR helpers (Core 1) ==========

struct ExclusionZone { uint16_t center; uint8_t halfWidth; };
static const ExclusionZone EXCLUSION_ZONES[] = POST_EXCLUSION_ZONES;

static void applyExclusion(uint16_t bins[360]) {
    for(uint8_t z = 0; z < POST_EXCLUSION_COUNT; z++) {
        int c  = EXCLUSION_ZONES[z].center;
        int hw = EXCLUSION_ZONES[z].halfWidth;
        for(int o = -hw; o <= hw; o++)
            bins[((c + o) % 360 + 360) % 360] = 0;
    }
}

// rawBins has no LIDAR_MIN_MM floor so close objects still trigger stop.
// called after applyExclusion so mounting posts never fire.
static bool isObstacleAhead(const uint16_t rawBins[360]) {
    if(currentMode == SENSOR_ONLY) return false;
    for(int offset = -OBSTACLE_CONE_DEG; offset <= OBSTACLE_CONE_DEG; offset++) {
        int      idx = ((OBSTACLE_FORWARD_BIN + offset) % 360 + 360) % 360;
        uint16_t d   = rawBins[idx];
        if(d > 0 && d < OBSTACLE_STOP_MM) return true;
    }
    return false;
}

static void startLidar() {
    lidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    if(!lidar.begin())     { Serial.println("ERROR: lidar.begin() failed");     while(1) {} }
    if(!lidar.startScan()) { Serial.println("ERROR: lidar.startScan() failed"); while(1) {} }
    scanning = true;
    Serial.println("[LIDAR] scanning");
}

// ========== SCN3 framing helpers (Core 0) ==========

static inline void u16le(uint8_t* buf, size_t* p, uint16_t v) {
    buf[(*p)++] = v & 0xFF;
    buf[(*p)++] = v >> 8;
}

static inline void u32le(uint8_t* buf, size_t* p, uint32_t v) {
    buf[(*p)++] =  v        & 0xFF;
    buf[(*p)++] = (v >>  8) & 0xFF;
    buf[(*p)++] = (v >> 16) & 0xFF;
    buf[(*p)++] = (v >> 24) & 0xFF;
}

// ========== WiFi / MQTT connect (Core 0 task) ==========

static void doConnectWiFi() {
    if(WiFi.status() == WL_CONNECTED) return;
    Serial.print("[WIFI] connecting");
    WiFi.disconnect(true);
    vTaskDelay(pdMS_TO_TICKS(100));
    WiFi.mode(WIFI_STA);
    esp_wifi_set_mac(WIFI_IF_STA, &MAC[0]);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for(int i = 0; WiFi.status() != WL_CONNECTED && i < 40; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print('.');
    }
    if(WiFi.status() == WL_CONNECTED) {
        WiFi.setSleep(false);  // eliminates up to 100ms RX latency
        Serial.printf("\n[WIFI] %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WIFI] failed — will retry");
    }
}

static void doConnectMQTT() {
    mqtt.stop();
    mqtt.setId("ESP32S3_r1_v1");
    mqtt.setKeepAliveInterval(30000);
    mqtt.setTxPayloadSize(PAYLOAD_SIZE + 32);
    for(int i = 0; !mqtt.connect(MQTT_HOST, MQTT_PORT) && i < 10; i++) {
        Serial.print("[MQTT] retry...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if(!mqtt.connected()) { Serial.println("[MQTT] failed"); return; }
    net.setNoDelay(true);  // disable Nagle — immediate ACKs
    mqtt.subscribe("robot/r1/cmd/twist", 1);
    mqtt.subscribe("robot/r1/cmd/mode",  1);
    Serial.println("[MQTT] connected + subscribed");
}

// ========== MQTT command handlers (Core 0) ==========

static void handleTwist(const char* payload) {
    if(currentMode == SENSOR_ONLY) return;

    float v = 0, w = 0;
    uint32_t ttlMs = DEFAULT_MOTOR_TIMEOUT_MS;
    char  cmdId[64] = {0};
    int   seq = -1;
    const char* p;

    if((p = strstr(payload, "\"v\":")))      v     = atof(p + 4);
    if((p = strstr(payload, "\"w\":")))      w     = atof(p + 4);
    if((p = strstr(payload, "\"seq\":")))    seq   = atoi(p + 6);
    if((p = strstr(payload, "\"ttl_ms\":"))) ttlMs = (uint32_t)max(0, atoi(p + 9));
    extractJsonStringField(payload, "command_id", cmdId, sizeof(cmdId));

    bool isDuplicate = hasSeenCommandId(cmdId);

    if(strlen(cmdId) > 0) {
        mqtt.beginMessage("robot/r1/evt/ack");
        if(!isDuplicate) {
            mqtt.printf("{\"command_id\":\"%s\",\"seq\":%d,\"status\":\"received\"}", cmdId, seq);
        } else {
            mqtt.printf("{\"command_id\":\"%s\",\"seq\":%d,\"status\":\"duplicate\"}", cmdId, seq);
        }
        mqtt.endMessage();
    }

    if(isDuplicate) return;
    rememberCommandId(cmdId);

    MotorCmd cmd;
    cmd.v     = v;
    cmd.w     = w;
    cmd.ttlMs = ttlMs;
    cmd.seq   = seq;
    strncpy(cmd.cmdId, cmdId, sizeof(cmd.cmdId) - 1);
    cmd.cmdId[sizeof(cmd.cmdId) - 1] = '\0';

    xQueueSend(cmdQueue, &cmd, 0);
    motorCmdsRx++;

    Serial.printf("[CMD] v=%.2f w=%.2f ttl=%lums seq=%d\n", v, w, ttlMs, seq);
}

static void handleMode(const char* payload) {
    if      (strstr(payload, "SENSOR_ONLY"))     currentMode = SENSOR_ONLY;
    else if(strstr(payload, "FULL_INTEGRATION")) currentMode = FULL_INTEGRATION;
    Serial.printf("[MODE] %s\n", currentMode == SENSOR_ONLY ? "SENSOR_ONLY" : "FULL_INTEGRATION");
}

// ========== MQTT publishers (Core 0) ==========

static void publishScan(const ScanMsg& scan) {
    if(!mqtt.connected()) return;
    uint8_t payload[PAYLOAD_SIZE];
    size_t  pos = 0;
    payload[pos++] = 'S'; payload[pos++] = 'C';
    payload[pos++] = 'N'; payload[pos++] = '3';
    u32le(payload, &pos, publishSeq++);
    u32le(payload, &pos, scan.tsMs);
    u16le(payload, &pos, scan.validBins);
    for(int i = 0; i < 360; i++) u16le(payload, &pos, scan.bins[i]);
    mqtt.beginMessage("robot/r1/lidar", (unsigned long)pos, false, 0);
    mqtt.write(payload, pos);
    if(mqtt.endMessage()) scansOk++;
}

static void publishEncoder(const EncoderMsg& enc) {
    if(!mqtt.connected()) return;
    mqtt.beginMessage("robot/r1/encoder");
    mqtt.printf("{\"ts_ms\":%lu,\"left_ticks\":%ld,\"right_ticks\":%ld}",
                enc.tsMs, enc.leftTicks, enc.rightTicks);
    mqtt.endMessage();
}

static void publishStatus() {
    if(!mqtt.connected()) return;
    int16_t lPWM, rPWM;
    portENTER_CRITICAL(&motorMux);
    lPWM = sharedLPWM;
    rPWM = sharedRPWM;
    portEXIT_CRITICAL(&motorMux);
    mqtt.beginMessage("robot/r1/status");
    mqtt.printf("{\"mode\":%d,\"scans_ok\":%lu,\"scans_bad\":%lu,\"mot_l\":%d,\"mot_r\":%d,\"obstacle\":%d}",
                (int)currentMode, scansOk, scansRejected, lPWM, rPWM, obstacleBlocked ? 1 : 0);
    mqtt.endMessage();
}

// ========== mqttTask — pinned to Core 0 ==========

void mqttTask(void* pv) {
    doConnectWiFi();
    doConnectMQTT();

    uint32_t lastStatusMs    = 0;
    uint32_t lastReconnectMs = 0;

    for(;;) {
        uint32_t now = millis();

        if(!mqtt.connected()) {
            if(now - lastReconnectMs >= 5000) {
                lastReconnectMs = now;
                if(WiFi.status() != WL_CONNECTED) doConnectWiFi();
                doConnectMQTT();
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // don't call mqtt.poll() — parseMessage() calls it internally; explicit poll() discards waiting payload
        int msgSize = mqtt.parseMessage();
        while(msgSize > 0) {
            String topic = mqtt.messageTopic();
            char   payload[512];
            int    i = 0;
            while(mqtt.available() && i < 511)
                payload[i++] = (char)mqtt.read();
            while(mqtt.available()) mqtt.read();  // drain any bytes past 511
            payload[i] = '\0';

            if      (topic == "robot/r1/cmd/twist") handleTwist(payload);
            else if(topic == "robot/r1/cmd/mode")  handleMode(payload);

            msgSize = mqtt.parseMessage();
        }

        ScanMsg scan;
        if(xQueueReceive(scanQueue, &scan, 0) == pdTRUE) {
            publishScan(scan);
        }

        EncoderMsg enc;
        if(xQueueReceive(encoderQueue, &enc, 0) == pdTRUE) {
            publishEncoder(enc);
        }

        if(now - lastStatusMs >= STATUS_PERIOD_MS) {
            lastStatusMs = now;
            publishStatus();
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ========== setup / loop (Core 1) ==========

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\nROBOT full_integration_v1 — ESP32-S3 dual-core\n");

    scanQueue    = xQueueCreate(1, sizeof(ScanMsg));
    encoderQueue = xQueueCreate(2, sizeof(EncoderMsg));
    cmdQueue     = xQueueCreate(4, sizeof(MotorCmd));

    setupMotors();
    setupEncoders();
    startLidar();

    // Launch MQTT task on Core 0 at higher priority than loop
    xTaskCreatePinnedToCore(mqttTask, "mqtt", 8192, NULL, 2, NULL, 0);

    lastMotorCmdMs = millis();
    Serial.printf("[BOOT] mode=FULL_INTEGRATION  LiDAR+motor=Core1  MQTT=Core0\n");
    Serial.printf("[BOOT] OBSTACLE_FORWARD_BIN=%d  OBSTACLE_CONE_DEG=%d  OBSTACLE_STOP_MM=%d\n",
                  OBSTACLE_FORWARD_BIN, OBSTACLE_CONE_DEG, OBSTACLE_STOP_MM);
}

void loop() {
    uint32_t now = millis();

    if(scanning && lidar.update()) {
        uint16_t valid = lidar.getValidBinCount();
        if(valid >= MIN_VALID_BINS) {
            ScanMsg scan;
            scan.validBins = valid;
            scan.tsMs      = now;

            uint16_t rawBins[360];
            for(int i = 0; i < 360; i++) {
                uint16_t d   = lidar.getDistance(i);
                rawBins[i]   = (d > 0 && d <= LIDAR_MAX_MM) ? d : 0;
                scan.bins[i] = (d >= LIDAR_MIN_MM && d <= LIDAR_MAX_MM) ? d : 0;
            }
            applyExclusion(rawBins);   // zero posts in raw too so they don't trigger stops
            applyExclusion(scan.bins);

            bool wasBlocked = obstacleBlocked;
            bool newBlocked = isObstacleAhead(rawBins);

            // stay blocked if cone goes all-zero: robot is in lidar dead zone (<~100mm), wall still there
            if(wasBlocked && !newBlocked) {
                bool coneHasData = false;
                for(int off = -OBSTACLE_CONE_DEG; off <= OBSTACLE_CONE_DEG && !coneHasData; off++) {
                    int idx = ((OBSTACLE_FORWARD_BIN + off) % 360 + 360) % 360;
                    if(rawBins[idx] > 0) coneHasData = true;
                }
                if(!coneHasData) newBlocked = true;
            }
            obstacleBlocked = newBlocked;

            if(obstacleBlocked && !wasBlocked) {
                bool stoppedForObstacle = false;
                if(isForwardMotion(activeCmdV)) {
                    setMotor(0, 0);
                    cmdLPWM = cmdRPWM = 0;
                    activeCmdV = 0.0f;
                    straightMode = false;
                    stoppedForObstacle = true;
                }
                uint16_t minD = 0xFFFF; int minI = -1;
                for(int off = -OBSTACLE_CONE_DEG; off <= OBSTACLE_CONE_DEG; off++) {
                    int idx = ((OBSTACLE_FORWARD_BIN + off) % 360 + 360) % 360;
                    if(rawBins[idx] > 0 && rawBins[idx] < minD) { minD = rawBins[idx]; minI = idx; }
                }
                Serial.printf("[OBS] %s  bin=%d  d=%umm\n", stoppedForObstacle ? "STOP" : "blocked", minI, minD);
            } else if(!obstacleBlocked && wasBlocked) {
                Serial.println("[OBS] clear");
            }

            // 1Hz: verify OBSTACLE_FORWARD_BIN is pointing forward
            static uint32_t lastFwdDiagMs = 0;
            if(now - lastFwdDiagMs >= 1000) {
                lastFwdDiagMs = now;
                uint16_t minD = 0xFFFF; int minBin = -1;
                for(int off = -OBSTACLE_CONE_DEG; off <= OBSTACLE_CONE_DEG; off++) {
                    int idx = ((OBSTACLE_FORWARD_BIN + off) % 360 + 360) % 360;
                    if(rawBins[idx] > 0 && rawBins[idx] < minD) { minD = rawBins[idx]; minBin = idx; }
                }
                if(minBin >= 0)
                    Serial.printf("[FWD] cone nearest: bin=%d  d=%umm  blocked=%d\n", minBin, minD, (int)obstacleBlocked);
                else
                    Serial.printf("[FWD] cone all-zero (dead zone or open)  blocked=%d\n", (int)obstacleBlocked);
            }

            portENTER_CRITICAL(&encoderMux);
            scan.leftTicks  = leftTicks;
            scan.rightTicks = rightTicks;
            portEXIT_CRITICAL(&encoderMux);

            xQueueOverwrite(scanQueue, &scan);
        } else {
            scansRejected++;
        }
    }

    if(now - lastEncoderMs >= ENCODER_PERIOD_MS) {
        lastEncoderMs = now;
        EncoderMsg enc;
        portENTER_CRITICAL(&encoderMux);
        enc.leftTicks  = leftTicks;
        enc.rightTicks = rightTicks;
        portEXIT_CRITICAL(&encoderMux);
        enc.tsMs = now;
        xQueueSend(encoderQueue, &enc, 0);
    }

    MotorCmd cmd;
    while(xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE) {
        if(currentMode == FULL_INTEGRATION) {
            lastMotorCmdMs = millis();  // refresh timeout even for blocked commands

            motorTimeoutMs = cmd.ttlMs;

            if(obstacleBlocked && isForwardMotion(cmd.v)) {
                setMotor(0, 0);
                cmdLPWM = cmdRPWM = 0;
                activeCmdV = 0.0f;
                straightMode = false;
            } else {
                applyMotorCmd(cmd);
            }
        }
    }

    checkMotorTimeout();
    updateDrift();
}
