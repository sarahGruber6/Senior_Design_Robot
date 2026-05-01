// full_integration.ino — ESP32-S3
// Drop-in replacement for Uno_R4/tests/full_integration_test.
//
// What changed from the UNO R4 version:
//   - WiFiS3.h  → WiFi.h
//   - FspTimer  → attachInterrupt (IRAM_ATTR, all 4 encoder pins)
//   - analogWrite → ledcAttach / ledcWrite (ESP32 Arduino 3.x PWM API)
//   - HardwareSerial lidarSerial(1) with custom RX/TX pins
//   - noInterrupts/interrupts → portENTER/EXIT_CRITICAL_ISR
//   - free_heap via ESP.getFreeHeap()
//
// Everything else — MQTT topics, SCN3 frame format, motor gains, drift correction,
// exclusion zones, reconnect logic — is identical.

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

// -------------------- Mode --------------------

enum TestMode {
    SENSOR_ONLY,
    MOTOR_MANUAL,
    MOTOR_CLOSED_LOOP,
    FULL_INTEGRATION,
    DIAGNOSTIC
};

TestMode currentMode = FULL_INTEGRATION;

// -------------------- Constants --------------------

static const uint16_t LIDAR_MIN_MM  = 150;
static const uint16_t LIDAR_MAX_MM  = 10000;
static const uint16_t MIN_VALID_BINS = 200;

static const uint32_t ENCODER_PUBLISH_PERIOD_MS = 50;
static const uint32_t STATUS_PUBLISH_PERIOD_MS  = 200;
static const uint32_t DIAG_REPORT_PERIOD_MS     = 5000;
static const uint32_t RECONNECT_CHECK_PERIOD_MS = 5000;
static const uint32_t CORRECTION_PERIOD_MS      = 50;
static const uint16_t MOTOR_TIMEOUT_MS          = 500;

// SCN3: "SCN3"(4) + seq(4) + ts_ms(4) + valid(2) + 360*dist(720) = 734 bytes
static const size_t PAYLOAD_SIZE = 4 + 4 + 4 + 2 + 360 * 2;

// -------------------- Quadrature table --------------------

static const int8_t QUAD_TABLE[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

// -------------------- Encoder state --------------------

volatile int32_t leftTicks      = 0;
volatile int32_t rightTicks     = 0;
volatile uint8_t leftPrevState  = 0;
volatile uint8_t rightPrevState = 0;

void IRAM_ATTR leftEncoderISR() {
    uint8_t s = ((digitalRead(LEFT_A) ? 1 : 0) << 1) | (digitalRead(LEFT_B) ? 1 : 0);
    leftTicks += QUAD_TABLE[(leftPrevState << 2) | s];
    leftPrevState = s;
}

void IRAM_ATTR rightEncoderISR() {
    uint8_t s = ((digitalRead(RIGHT_A) ? 1 : 0) << 1) | (digitalRead(RIGHT_B) ? 1 : 0);
    rightTicks += QUAD_TABLE[(rightPrevState << 2) | s];
    rightPrevState = s;
}

// -------------------- Globals --------------------

HardwareSerial lidarSerial(1);
YDLidarG2   lidar(lidarSerial, LIDAR_MOTOR_PIN);
const byte mac[] = {0x10, 0x51, 0xDB, 0x37, 0x4F, 0x14};
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

WiFiClient  net;
MqttClient  mqtt(net);

bool scanning = false;

int16_t  leftMotorPWM    = 0;
int16_t  rightMotorPWM   = 0;
uint32_t lastMotorCommand = 0;
static const uint8_t RECENT_COMMAND_ID_COUNT = 8;
String   recentCommandIds[RECENT_COMMAND_ID_COUNT];
uint8_t  recentCommandIdSlot = 0;

int16_t  cmdLeftPWM       = 0;
int16_t  cmdRightPWM      = 0;
int32_t  leftTicksRef     = 0;
int32_t  rightTicksRef    = 0;
bool     straightMode     = false;
uint32_t lastCorrectionMs = 0;

// LiDAR double-buffer — serviceLidar never blocks on MQTT
uint16_t distanceBins[360];
uint16_t publishBins[360];
uint16_t pendingValidBins = 0;
uint32_t pendingTs        = 0;
bool     scanPending      = false;
uint32_t publishSeq       = 0;

uint32_t lastEncoderPublish = 0;
uint32_t lastStatusReport   = 0;
uint32_t lastDiagReport     = 0;
uint32_t lastReconnectCheck = 0;

uint32_t scansCompleted        = 0;
uint32_t scansPublished        = 0;
uint32_t scansRejected         = 0;
uint32_t motorCommandsReceived = 0;

bool extractJsonStringField(const char* payload, const char* key, char* out, size_t outSize) {
    char quotedKey[32];
    snprintf(quotedKey, sizeof(quotedKey), "\"%s\"", key);

    const char* p = strstr(payload, quotedKey);
    if (!p) return false;

    p = strchr(p + strlen(quotedKey), ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '"') return false;
    p++;

    const char* end = strchr(p, '"');
    if (!end) return false;

    size_t len = min((size_t)(end - p), outSize - 1);
    strncpy(out, p, len);
    out[len] = '\0';
    return true;
}

bool hasSeenCommandId(const char* commandId) {
    if (strlen(commandId) == 0) return false;
    for (uint8_t i = 0; i < RECENT_COMMAND_ID_COUNT; i++) {
        if (recentCommandIds[i] == String(commandId)) return true;
    }
    return false;
}

void rememberCommandId(const char* commandId) {
    if (strlen(commandId) == 0) return;
    recentCommandIds[recentCommandIdSlot] = String(commandId);
    recentCommandIdSlot = (recentCommandIdSlot + 1) % RECENT_COMMAND_ID_COUNT;
}

// -------------------- Motor --------------------

void setupMotors() {
    ledcAttach(LEFT_PWM_PIN,  1000, 8);
    ledcAttach(RIGHT_PWM_PIN, 1000, 8);
    pinMode(LEFT_DIR_PIN,  OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    ledcWrite(LEFT_PWM_PIN,  0);
    ledcWrite(RIGHT_PWM_PIN, 0);
    digitalWrite(LEFT_DIR_PIN,  LOW);
    digitalWrite(RIGHT_DIR_PIN, LOW);
}

void setMotorSpeed(int16_t leftPWM, int16_t rightPWM) {
    leftPWM  = constrain(leftPWM,  -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);
    digitalWrite(LEFT_DIR_PIN,  leftPWM  < 0 ? HIGH : LOW);
    ledcWrite(LEFT_PWM_PIN,     (uint8_t)abs(leftPWM));
    digitalWrite(RIGHT_DIR_PIN, rightPWM < 0 ? LOW : HIGH);
    ledcWrite(RIGHT_PWM_PIN,    (uint8_t)abs(rightPWM));
    leftMotorPWM  = leftPWM;
    rightMotorPWM = rightPWM;
}

void handleMotorTimeout() {
    if (currentMode == SENSOR_ONLY || currentMode == DIAGNOSTIC) return;
    if (millis() - lastMotorCommand > MOTOR_TIMEOUT_MS) {
        setMotorSpeed(0, 0);
        straightMode = false;
        cmdLeftPWM = cmdRightPWM = 0;
    }
}

// -------------------- Encoders --------------------

void setupEncoders() {
    pinMode(LEFT_A,  INPUT_PULLUP);
    pinMode(LEFT_B,  INPUT_PULLUP);
    pinMode(RIGHT_A, INPUT_PULLUP);
    pinMode(RIGHT_B, INPUT_PULLUP);

    leftPrevState  = ((digitalRead(LEFT_A)  ? 1 : 0) << 1) | (digitalRead(LEFT_B)  ? 1 : 0);
    rightPrevState = ((digitalRead(RIGHT_A) ? 1 : 0) << 1) | (digitalRead(RIGHT_B) ? 1 : 0);

    attachInterrupt(digitalPinToInterrupt(LEFT_A),  leftEncoderISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_B),  leftEncoderISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_B), rightEncoderISR, CHANGE);

    Serial.println("[ENC] interrupts attached");
}

// -------------------- Drift correction --------------------

void updateDriftCorrection() {
    if (!straightMode || (cmdLeftPWM == 0 && cmdRightPWM == 0)) return;
    if (millis() - lastCorrectionMs < CORRECTION_PERIOD_MS) return;
    lastCorrectionMs = millis();

    int32_t L, R;
    portENTER_CRITICAL(&encoderMux); L = leftTicks; R = rightTicks; portEXIT_CRITICAL(&encoderMux);

    int32_t dL  = L - leftTicksRef;
    int32_t dR  = R - rightTicksRef;
    long    avg = (abs(dL) + abs(dR)) / 2;
    if (avg < 5) return;

    float relDrift = (float)(dL - dR) / (float)avg;
    float factor   = constrain(1.0f - K_CORRECT * relDrift, 0.5f, 2.0f);
    setMotorSpeed((int16_t)(cmdLeftPWM * factor), cmdRightPWM);
}

// -------------------- LiDAR --------------------

struct ExclusionZone { uint16_t center; uint8_t halfWidth; };
static const ExclusionZone EXCLUSION_ZONES[] = POST_EXCLUSION_ZONES;

static void applyExclusionZones(uint16_t bins[360]) {
    for (uint8_t z = 0; z < POST_EXCLUSION_COUNT; z++) {
        int center = EXCLUSION_ZONES[z].center;
        int hw     = EXCLUSION_ZONES[z].halfWidth;
        for (int offset = -hw; offset <= hw; offset++) {
            bins[((center + offset) % 360 + 360) % 360] = 0;
        }
    }
}

static inline bool isValidRangeMm(uint16_t d) {
    return d >= LIDAR_MIN_MM && d <= LIDAR_MAX_MM;
}

static inline void copyScanBinsFromLidar(uint16_t bins[360]) {
    for (int i = 0; i < 360; ++i) {
        uint16_t d = lidar.getDistance(i);
        bins[i] = isValidRangeMm(d) ? d : 0;
    }
}

static inline void writeU16LE(uint8_t* buf, size_t* pos, uint16_t v) {
    buf[(*pos)++] = (uint8_t)(v & 0xFF);
    buf[(*pos)++] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void writeU32LE(uint8_t* buf, size_t* pos, uint32_t v) {
    buf[(*pos)++] = (uint8_t)(v        & 0xFF);
    buf[(*pos)++] = (uint8_t)((v >> 8)  & 0xFF);
    buf[(*pos)++] = (uint8_t)((v >> 16) & 0xFF);
    buf[(*pos)++] = (uint8_t)((v >> 24) & 0xFF);
}

void startLidar() {
    lidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

    if (!lidar.begin()) {
        Serial.println("ERROR: lidar.begin() failed");
        while (1) {}
    }
    if (!lidar.startScan()) {
        Serial.println("ERROR: lidar.startScan() failed");
        while (1) {}
    }
    scanning = true;
    Serial.println("[LIDAR] Scanning started");
}

void publishCurrentScan(const uint16_t bins[360], uint32_t tsMs, uint16_t validBins) {
    if (!mqtt.connected() || currentMode == DIAGNOSTIC) return;

    uint8_t payload[PAYLOAD_SIZE];
    size_t  pos = 0;

    payload[pos++] = 'S'; payload[pos++] = 'C'; payload[pos++] = 'N'; payload[pos++] = '3';
    writeU32LE(payload, &pos, publishSeq++);
    writeU32LE(payload, &pos, tsMs);
    writeU16LE(payload, &pos, validBins);
    for (int i = 0; i < 360; ++i) writeU16LE(payload, &pos, bins[i]);

    mqtt.beginMessage("robot/r1/lidar", (unsigned long)pos, false, 0);
    mqtt.write(payload, pos);
    lidar.update(); // drain serial before blocking TCP send
    if (mqtt.endMessage()) scansPublished++;
    lidar.update(); // catch bytes that arrived during endMessage
}

void serviceLidar() {
    if (!scanning || currentMode == DIAGNOSTIC) return;
    if (!lidar.update()) return;

    scansCompleted++;

    uint16_t validBins = lidar.getValidBinCount();
    if (validBins < MIN_VALID_BINS) { scansRejected++; return; }

    copyScanBinsFromLidar(distanceBins);
    applyExclusionZones(distanceBins);

    memcpy(publishBins, distanceBins, sizeof(publishBins));
    pendingValidBins = validBins;
    pendingTs        = millis();
    scanPending      = true;
}

// -------------------- Encoder publish --------------------

void serviceEncoderData() {
    uint32_t now = millis();
    if (now - lastEncoderPublish < ENCODER_PUBLISH_PERIOD_MS) return;
    lastEncoderPublish = now;

    int32_t L, R;
    portENTER_CRITICAL(&encoderMux); L = leftTicks; R = rightTicks; portEXIT_CRITICAL(&encoderMux);

    Serial.print("[ENC] L="); Serial.print(L); Serial.print(" R="); Serial.println(R);

    if (!mqtt.connected() || currentMode == DIAGNOSTIC) return;

    mqtt.beginMessage("robot/r1/encoder");
    mqtt.print("{\"ts_ms\":"); mqtt.print(now);
    mqtt.print(",\"left_ticks\":"); mqtt.print(L);
    mqtt.print(",\"right_ticks\":"); mqtt.print(R);
    mqtt.print("}");
    if (!mqtt.endMessage()) return;
}

// -------------------- Status / Diagnostics --------------------

void publishStatus() {
    if (!mqtt.connected()) return;
    uint32_t now = millis();
    if (now - lastStatusReport < STATUS_PUBLISH_PERIOD_MS) return;
    lastStatusReport = now;

    mqtt.beginMessage("robot/r1/status");
    mqtt.print("{\"mode\":"); mqtt.print((int)currentMode);
    mqtt.print(",\"scans_ok\":"); mqtt.print(scansPublished);
    mqtt.print(",\"scans_bad\":"); mqtt.print(scansRejected);
    mqtt.print(",\"mot_l\":"); mqtt.print(leftMotorPWM);
    mqtt.print(",\"mot_r\":"); mqtt.print(rightMotorPWM);
    mqtt.print("}");
    if (!mqtt.endMessage()) return;
}

void printDiagnostics() {
    uint32_t now = millis();
    if (now - lastDiagReport < DIAG_REPORT_PERIOD_MS) return;
    lastDiagReport = now;

    int32_t L, R;
    portENTER_CRITICAL(&encoderMux); L = leftTicks; R = rightTicks; portEXIT_CRITICAL(&encoderMux);

    Serial.println("\n=== DIAGNOSTICS ===");
    Serial.print("Mode: "); Serial.println((int)currentMode);
    Serial.print("LiDAR: "); Serial.print(scansCompleted); Serial.print(" / ");
    Serial.print(scansPublished); Serial.print(" / "); Serial.println(scansRejected);
    Serial.print("Motor: L="); Serial.print(leftMotorPWM); Serial.print(" R="); Serial.println(rightMotorPWM);
    Serial.print("Encoders: L="); Serial.print(L); Serial.print(" R="); Serial.println(R);
    Serial.print("Commands: "); Serial.println(motorCommandsReceived);
    Serial.print("Free heap: "); Serial.println(ESP.getFreeHeap());
    Serial.println("==================\n");
}

// -------------------- MQTT handlers --------------------

void handleTwistCommand(const char* payload) {
    if (currentMode < MOTOR_MANUAL) return;

    float v = 0, w = 0;
    char  command_id[64] = {0};
    int   seq = -1;

    const char* p;
    if ((p = strstr(payload, "\"v\":")))   v   = atof(p + 4);
    if ((p = strstr(payload, "\"w\":")))   w   = atof(p + 4);
    if ((p = strstr(payload, "\"seq\":"))) seq = atoi(p + 6);

    extractJsonStringField(payload, "command_id", command_id, sizeof(command_id));

    bool duplicateCommand = hasSeenCommandId(command_id);
    if (duplicateCommand) {
        mqtt.beginMessage("robot/r1/evt/ack");
        mqtt.print("{\"command_id\":\""); mqtt.print(command_id);
        mqtt.print("\",\"seq\":"); mqtt.print(seq);
        mqtt.print(",\"status\":\"duplicate\"}");
        mqtt.endMessage();
        return;
    }

    if (strlen(command_id) > 0) {
        rememberCommandId(command_id);
        mqtt.beginMessage("robot/r1/evt/ack");
        mqtt.print("{\"command_id\":\""); mqtt.print(command_id);
        mqtt.print("\",\"seq\":"); mqtt.print(seq);
        mqtt.print(",\"status\":\"received\"}");
        mqtt.endMessage();
    }

    int16_t base = (int16_t)(K_V * v);
    int16_t turn = (int16_t)(K_W * w);
    int16_t lPWM = (int16_t)(base * LEFT_TRIM) - turn;
    int16_t rPWM = base + turn;

    setMotorSpeed(lPWM, rPWM);

    cmdLeftPWM  = lPWM;
    cmdRightPWM = rPWM;
    portENTER_CRITICAL(&encoderMux);
    leftTicksRef = leftTicks; rightTicksRef = rightTicks;
    portEXIT_CRITICAL(&encoderMux);
    straightMode     = (fabsf(w) < 0.05f) && (fabsf(v) > 0.01f);
    lastCorrectionMs = millis();
    lastMotorCommand = millis();
    motorCommandsReceived++;

    Serial.print("[MOTOR] v="); Serial.print(v, 2);
    Serial.print(" w="); Serial.print(w, 2);
    Serial.print(" seq="); Serial.println(seq);
}

void handleModeCommand(const char* payload) {
    if      (strstr(payload, "SENSOR_ONLY"))      { currentMode = SENSOR_ONLY;      setMotorSpeed(0,0); }
    else if (strstr(payload, "MOTOR_MANUAL"))      currentMode = MOTOR_MANUAL;
    else if (strstr(payload, "MOTOR_CLOSED_LOOP")) currentMode = MOTOR_CLOSED_LOOP;
    else if (strstr(payload, "FULL_INTEGRATION"))  currentMode = FULL_INTEGRATION;
    else if (strstr(payload, "DIAGNOSTIC"))        currentMode = DIAGNOSTIC;
    Serial.print("[MODE] "); Serial.println((int)currentMode);
}

void handleTestAction(const char* payload) {
    Serial.print("[TEST] "); Serial.println(payload);
}

// -------------------- Connection --------------------

void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.print("[WIFI] Connecting...");
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_mac(WIFI_IF_STA, &mac[0]);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.print("\n[WIFI] IP="); Serial.println(WiFi.localIP());
}

void connectMQTT() {
    mqtt.stop();
    mqtt.setId("ESP32S3_Robot1");
    mqtt.setKeepAliveInterval(30 * 1000);
    mqtt.setTxPayloadSize(PAYLOAD_SIZE + 32);

    Serial.print("[MQTT] Connecting...");
    while (!mqtt.connect(MQTT_HOST, MQTT_PORT)) { Serial.print("."); delay(500); }
    Serial.println("\n[MQTT] Connected");

    mqtt.subscribe("robot/r1/cmd/twist",       1);
    mqtt.subscribe("robot/r1/cmd/mode",        1);
    mqtt.subscribe("robot/r1/cmd/test_action", 1);
}

// -------------------- Setup / Loop --------------------

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\nROBOT FULL INTEGRATION — ESP32-S3\n");

    connectWiFi();
    connectMQTT();
    setupMotors();
    setupEncoders();
    startLidar();

    lastMotorCommand = millis();
    Serial.print("[BOOT] Mode: "); Serial.println((int)currentMode);
}

void loop() {
    uint32_t now = millis();

    // ===== PRIORITY 1: MQTT polling =====
    if (!mqtt.connected()) {
        if (now - lastReconnectCheck >= RECONNECT_CHECK_PERIOD_MS) {
            lastReconnectCheck = now;
            if (WiFi.status() != WL_CONNECTED) connectWiFi();
            connectMQTT();
        }
    } else {
        mqtt.poll();

        int msgSize = mqtt.parseMessage();
        while (msgSize > 0) {
            String topic = mqtt.messageTopic();
            char   payload[512];
            int    i = 0;
            uint32_t t = millis();
            while (i < msgSize && millis() - t < 200) {
                if (mqtt.available()) {
                    if (i < (int)sizeof(payload) - 1) payload[i++] = (char)mqtt.read();
                    else mqtt.read();
                }
            }
            payload[i] = '\0';

            Serial.print("[MQTT] rx topic="); Serial.print(topic);
            Serial.print(" payload="); Serial.println(payload);

            if      (topic == "robot/r1/cmd/twist")       handleTwistCommand(payload);
            else if (topic == "robot/r1/cmd/mode")        handleModeCommand(payload);
            else if (topic == "robot/r1/cmd/test_action") handleTestAction(payload);

            mqtt.poll();
            msgSize = mqtt.parseMessage();
        }

        if (now - lastReconnectCheck >= RECONNECT_CHECK_PERIOD_MS) {
            lastReconnectCheck = now;
            if (WiFi.status() != WL_CONNECTED) connectWiFi();
        }
    }

    // ===== PRIORITY 2: Motor safety & drift correction =====
    handleMotorTimeout();
    updateDriftCorrection();

    // ===== PRIORITY 3: Sensors (non-blocking) =====
    serviceLidar();

    if (scanPending) {
        publishCurrentScan(publishBins, pendingTs, pendingValidBins);
        scanPending = false;
        mqtt.poll();
    }

    serviceEncoderData();
    publishStatus();

    if (currentMode == DIAGNOSTIC) printDiagnostics();
}
