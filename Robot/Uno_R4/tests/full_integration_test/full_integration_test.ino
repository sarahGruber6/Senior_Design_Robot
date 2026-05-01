#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <FspTimer.h>
#include "YDLidarG2.h"
#include "config.h"
#ifdef __has_include
  #if __has_include("config_local.h")
    #include "config_local.h"
  #endif
#endif

// -------------------- Constants --------------------

enum TestMode {
    SENSOR_ONLY,
    MOTOR_MANUAL,
    MOTOR_CLOSED_LOOP,
    FULL_INTEGRATION,
    DIAGNOSTIC
};

TestMode defaultMode = FULL_INTEGRATION;
TestMode currentMode = defaultMode;

static const uint16_t LIDAR_MIN_MM = 150;
static const uint16_t LIDAR_MAX_MM = 10000;
static const uint16_t MIN_VALID_BINS = 200;

// Non-blocking timing intervals
static const uint32_t ENCODER_PUBLISH_PERIOD_MS = 50;    // Publish ~20 Hz
static const uint32_t STATUS_PUBLISH_PERIOD_MS = 200;    // Status ~5 Hz
static const uint32_t DIAG_REPORT_PERIOD_MS = 5000;      // Diagnostics every 5s
static const uint32_t RECONNECT_CHECK_PERIOD_MS = 5000;  // Check connection ~0.2 Hz

// Motor constants — K_V and K_W are defined in config.h for easy tuning
static const uint16_t MOTOR_TIMEOUT_MS = 500;

// Encoder Pins
#define RIGHT_A  5
#define RIGHT_B  6
#define LEFT_A   7
#define LEFT_B   4  

// ==================== GLOBALS ====================

YDLidarG2 lidar(Serial1, 3);
WiFiClient net;
MqttClient mqtt(net);

bool scanning = false;

// Encoder state (1 kHz FspTimer)
volatile int32_t leftTicks      = 0;
volatile int32_t rightTicks     = 0;
volatile uint8_t leftPrevState  = 0;
volatile uint8_t rightPrevState = 0;
FspTimer encoderTimer;

// Motor state
int16_t leftMotorPWM  = 0;
int16_t rightMotorPWM = 0;
uint32_t lastMotorCommand = 0;
String lastCommandId = "";

// Closed-loop drift correction
int16_t  cmdLeftPWM       = 0;   // feed-forward from last twist command
int16_t  cmdRightPWM      = 0;
int32_t  leftTicksRef     = 0;   // encoder snapshot at command time
int32_t  rightTicksRef    = 0;
bool     straightMode     = false;
uint32_t lastCorrectionMs = 0;
static const uint32_t CORRECTION_PERIOD_MS = 50;

// LiDAR scan storage — double-buffered so serviceLidar() never blocks on MQTT
uint16_t distanceBins[360];   // filled by serviceLidar each scan
uint16_t publishBins[360];    // snapshot handed to the publish step
uint16_t pendingValidBins = 0;
uint32_t pendingTs = 0;
bool scanPending = false;
uint32_t publishSeq = 0;

// Timing for non-blocking processing
uint32_t lastEncoderPublish = 0;
uint32_t lastStatusReport = 0;
uint32_t lastDiagReport = 0;
uint32_t lastReconnectCheck = 0;

// Statistics
uint32_t scansCompleted = 0;
uint32_t scansPublished = 0;
uint32_t scansRejected = 0;
uint32_t motorCommandsReceived = 0;

// -------------------- Quad Table --------------------

static const int8_t QUAD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

// -------------------- Encoder Timer ISR (1 kHz) --------------------

// Polled at 1 kHz via FspTimer — more reliable than attachInterrupt on UNO R4 WiFi.
// Both wheels use += so ticks increase when driving forward, matching slam_service odometry.
static void encoderISR(timer_callback_args_t* /*arg*/) {
    uint8_t lNew = ((digitalRead(LEFT_A)  ? 1 : 0) << 1) | (digitalRead(LEFT_B)  ? 1 : 0);
    uint8_t rNew = ((digitalRead(RIGHT_A) ? 1 : 0) << 1) | (digitalRead(RIGHT_B) ? 1 : 0);
    leftTicks  += QUAD_TABLE[(leftPrevState  << 2) | lNew];
    rightTicks += QUAD_TABLE[(rightPrevState << 2) | rNew];
    leftPrevState  = lNew;
    rightPrevState = rNew;
}

// -------------------- Motor Control --------------------

void setupMotors() {
    pinMode(9, OUTPUT);   // Left PWM
    pinMode(8, OUTPUT);   // Left DIR
    pinMode(11, OUTPUT);  // Right PWM
    pinMode(13, OUTPUT);  // Right DIR

    analogWrite(9, 0);
    analogWrite(11, 0);
    digitalWrite(8, LOW);
    digitalWrite(13, LOW);
}

void setMotorSpeed(int16_t leftPWM, int16_t rightPWM) {
    leftPWM = constrain(leftPWM, -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);

    digitalWrite(8, leftPWM < 0 ? HIGH : LOW);
    analogWrite(9, abs(leftPWM));

    digitalWrite(13, rightPWM < 0 ? HIGH : LOW);
    analogWrite(11, abs(rightPWM));

    leftMotorPWM = leftPWM;
    rightMotorPWM = rightPWM;
}

void handleMotorTimeout() {
    if (currentMode == MOTOR_MANUAL || currentMode == MOTOR_CLOSED_LOOP || currentMode == FULL_INTEGRATION) {
        uint32_t now = millis();
        if (now - lastMotorCommand > MOTOR_TIMEOUT_MS) {
            setMotorSpeed(0, 0);
            straightMode = false;
            cmdLeftPWM = cmdRightPWM = 0;
        }
    }
}

// -------------------- MQTT Handlers --------------------

void handleTwistCommand(const char *payload) {
    if (currentMode < MOTOR_MANUAL) {
        return;
    }

    float v = 0, w = 0;
    char command_id[64] = {0};
    int seq = -1;

    const char *v_pos = strstr(payload, "\"v\":");
    if (v_pos != NULL) {
        v = atof(v_pos + 4);
    }

    const char *w_pos = strstr(payload, "\"w\":");
    if (w_pos != NULL) {
        w = atof(w_pos + 4);
    }

    const char *seq_pos = strstr(payload, "\"seq\":");
    if (seq_pos != NULL) {
        seq = atoi(seq_pos + 6);
    }

    const char *cmd_pos = strstr(payload, "\"command_id\":\"");
    if (cmd_pos != NULL) {
        cmd_pos += 14;
        const char *cmd_end = strchr(cmd_pos, '"');
        if (cmd_end != NULL) {
            size_t len = cmd_end - cmd_pos;
            if (len >= sizeof(command_id)) len = sizeof(command_id) - 1;
            strncpy(command_id, cmd_pos, len);
            command_id[len] = '\0';
        }
    }

    // Duplicate detection
    if (strlen(command_id) > 0 && lastCommandId == String(command_id)) {
        mqtt.beginMessage("robot/r1/evt/ack");
        mqtt.print("{\"command_id\":\"");
        mqtt.print(command_id);
        mqtt.print("\",\"status\":\"received\"}");
        mqtt.endMessage();
        return;
    }

    // Send ACK
    if (strlen(command_id) > 0) {
        lastCommandId = String(command_id);
        mqtt.beginMessage("robot/r1/evt/ack");
        mqtt.print("{\"command_id\":\"");
        mqtt.print(command_id);
        mqtt.print("\",\"seq\":");
        mqtt.print(seq);
        mqtt.print(",\"status\":\"received\"}");
        mqtt.endMessage();
    }

    int16_t base = (int16_t)(K_V * v);
    int16_t turn = (int16_t)(K_W * w);
    int16_t leftPWM  = (int16_t)(base * LEFT_TRIM) - turn;
    int16_t rightPWM = base + turn;

    setMotorSpeed(leftPWM, rightPWM);

    // Snapshot for drift correction
    cmdLeftPWM  = leftPWM;
    cmdRightPWM = rightPWM;
    noInterrupts(); leftTicksRef = leftTicks; rightTicksRef = rightTicks; interrupts();
    straightMode     = (fabsf(w) < 0.05f) && (fabsf(v) > 0.01f);
    lastCorrectionMs = millis();

    lastMotorCommand = millis();
    motorCommandsReceived++;

    Serial.print("[MOTOR] v=");
    Serial.print(v, 2);
    Serial.print(" w=");
    Serial.print(w, 2);
    Serial.print(" seq=");
    Serial.println(seq);
}

void handleModeCommand(const char *payload) {
    if (strstr(payload, "SENSOR_ONLY") != NULL) {
        currentMode = SENSOR_ONLY;
        setMotorSpeed(0, 0);
    } else if (strstr(payload, "MOTOR_MANUAL") != NULL) {
        currentMode = MOTOR_MANUAL;
    } else if (strstr(payload, "MOTOR_CLOSED_LOOP") != NULL) {
        currentMode = MOTOR_CLOSED_LOOP;
    } else if (strstr(payload, "FULL_INTEGRATION") != NULL) {
        currentMode = FULL_INTEGRATION;
    } else if (strstr(payload, "DIAGNOSTIC") != NULL) {
        currentMode = DIAGNOSTIC;
    }

    Serial.print("[MODE] ");
    Serial.println((int)currentMode);
}

void handleTestAction(const char *payload) {
    Serial.print("[TEST] ");
    Serial.println(payload);
}

// -------------------- LIDAR FUNCTIONS (Improved from lidar_to_mqtt) --------------------

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

// Binary encoding helpers for clean SCN3 frame building
static inline void writeU16LE(uint8_t* buf, size_t* pos, uint16_t v) {
    buf[(*pos)++] = (uint8_t)(v & 0xFF);
    buf[(*pos)++] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void writeU32LE(uint8_t* buf, size_t* pos, uint32_t v) {
    buf[(*pos)++] = (uint8_t)(v & 0xFF);
    buf[(*pos)++] = (uint8_t)((v >> 8) & 0xFF);
    buf[(*pos)++] = (uint8_t)((v >> 16) & 0xFF);
    buf[(*pos)++] = (uint8_t)((v >> 24) & 0xFF);
}

void startLidar() {
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

    uint8_t payload[4 + 4 + 4 + 2 + 360 * 2];
    size_t pos = 0;

    payload[pos++] = 'S';
    payload[pos++] = 'C';
    payload[pos++] = 'N';
    payload[pos++] = '3';

    writeU32LE(payload, &pos, publishSeq++);
    writeU32LE(payload, &pos, tsMs);
    writeU16LE(payload, &pos, validBins);

    for (int i = 0; i < 360; ++i) {
        writeU16LE(payload, &pos, bins[i]);
    }

    mqtt.beginMessage("robot/r1/lidar", pos, false, 0);
    mqtt.write(payload, pos);
    lidar.update();  // drain Serial1 before the blocking TCP send
    if (mqtt.endMessage()) {
        scansPublished++;
    }
    lidar.update();  // catch bytes that arrived during endMessage
}

void serviceLidar() {
    if (!scanning || currentMode == DIAGNOSTIC) return;
    if (!lidar.update()) return;

    scansCompleted++;

    uint16_t validBins = lidar.getValidBinCount();
    if (validBins < MIN_VALID_BINS) {
        scansRejected++;
        return;
    }

    copyScanBinsFromLidar(distanceBins);
    applyExclusionZones(distanceBins);

    // Snapshot into publish buffer; overwrite any undelivered scan (always use latest)
    memcpy(publishBins, distanceBins, sizeof(publishBins));
    pendingValidBins = validBins;
    pendingTs = millis();
    scanPending = true;
}

// -------------------- ENCODER FUNCTIONS --------------------

void setupEncoders() {
    pinMode(LEFT_A,  INPUT_PULLUP);
    pinMode(LEFT_B,  INPUT_PULLUP);
    pinMode(RIGHT_A, INPUT_PULLUP);
    pinMode(RIGHT_B, INPUT_PULLUP);

    leftPrevState  = ((digitalRead(LEFT_A)  ? 1 : 0) << 1) | (digitalRead(LEFT_B)  ? 1 : 0);
    rightPrevState = ((digitalRead(RIGHT_A) ? 1 : 0) << 1) | (digitalRead(RIGHT_B) ? 1 : 0);

    uint8_t tt = AGT_TIMER;
    int8_t  ch = FspTimer::get_available_timer(tt);
    if (ch < 0) { tt = GPT_TIMER; ch = FspTimer::get_available_timer(tt); }
    if (ch < 0) { Serial.println("ERROR: no timer for encoder"); while (1) {} }

    encoderTimer.begin(TIMER_MODE_PERIODIC, tt, ch, 1000.0f, 0.0f, encoderISR);
    encoderTimer.setup_overflow_irq();
    encoderTimer.open();
    encoderTimer.start();
    Serial.println("[ENC] 1 kHz timer started");
}

// -------------------- DRIFT CORRECTION --------------------

// Proportional closed-loop correction: equalises left/right tick rates during straight driving.
// Only active when |w| < threshold. Runs every CORRECTION_PERIOD_MS.
void updateDriftCorrection() {
    if (!straightMode || (cmdLeftPWM == 0 && cmdRightPWM == 0)) return;
    if (millis() - lastCorrectionMs < CORRECTION_PERIOD_MS) return;
    lastCorrectionMs = millis();

    int32_t L, R;
    noInterrupts(); L = leftTicks; R = rightTicks; interrupts();

    // Both wheels use += so positive delta = forward on both sides
    int32_t dL  = L - leftTicksRef;
    int32_t dR  = R - rightTicksRef;
    long    avg = (abs(dL) + abs(dR)) / 2;
    if (avg < 5) return;   // not enough travel yet to compute a stable ratio

    // Normalised drift: fraction of average travel, so correction scales with speed
    float relDrift = (float)(dL - dR) / (float)avg;   // + → left faster
    float factor   = constrain(1.0f - K_CORRECT * relDrift, 0.5f, 2.0f);
    setMotorSpeed((int16_t)(cmdLeftPWM * factor), cmdRightPWM);
}

void serviceEncoderData() {
    uint32_t now = millis();
    if (now - lastEncoderPublish < ENCODER_PUBLISH_PERIOD_MS) return;
    lastEncoderPublish = now;

    long left, right;
    noInterrupts();
    left  = leftTicks;
    right = rightTicks;
    interrupts();

    Serial.print("[ENC] L=");
    Serial.print(left);
    Serial.print(" R=");
    Serial.println(right);

    if (!mqtt.connected() || currentMode == DIAGNOSTIC) return;

    mqtt.beginMessage("robot/r1/encoder");
    mqtt.print("{\"ts_ms\":");
    mqtt.print(now);
    mqtt.print(",\"left_ticks\":");
    mqtt.print(left);
    mqtt.print(",\"right_ticks\":");
    mqtt.print(right);
    mqtt.print("}");
    mqtt.endMessage();
}

// -------------------- STATUS TELEMETRY --------------------

void publishStatus() {
    if (!mqtt.connected()) return;

    uint32_t now = millis();
    if (now - lastStatusReport < STATUS_PUBLISH_PERIOD_MS) return;

    lastStatusReport = now;

    mqtt.beginMessage("robot/r1/status");
    mqtt.print("{\"mode\":");         mqtt.print((int)currentMode);
    mqtt.print(",\"scans_ok\":");     mqtt.print(scansPublished);
    mqtt.print(",\"scans_bad\":");    mqtt.print(scansRejected);
    mqtt.print(",\"mot_l\":");        mqtt.print(leftMotorPWM);
    mqtt.print(",\"mot_r\":");        mqtt.print(rightMotorPWM);
    mqtt.print("}");
    mqtt.endMessage();
}

void printDiagnostics() {
    uint32_t now = millis();
    if (now - lastDiagReport < DIAG_REPORT_PERIOD_MS) return;

    lastDiagReport = now;

    Serial.println("\n=== DIAGNOSTICS ===");
    Serial.print("Mode: ");
    Serial.println((int)currentMode);
    Serial.print("LiDAR: ");
    Serial.print(scansCompleted);
    Serial.print(" / ");
    Serial.print(scansPublished);
    Serial.print(" / ");
    Serial.println(scansRejected);
    Serial.print("Motor: L=");
    Serial.print(leftMotorPWM);
    Serial.print(" R=");
    Serial.println(rightMotorPWM);
    Serial.print("Commands: ");
    Serial.println(motorCommandsReceived);
    Serial.println("==================\n");
}

// -------------------- SETUP & LOOP --------------------

void setup() {
    Serial.begin(921600);
    delay(1000);

    Serial.println("\n\nROBOT FULL INTEGRATION TEST\n");

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("[WiFi] Connected");

    mqtt.setId("Arduino_R1");
    mqtt.setKeepAliveInterval(30 * 1000);
    mqtt.setTxPayloadSize(734 + 32);

    while (!mqtt.connect(MQTT_HOST, MQTT_PORT)) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("[MQTT] Connected");

    mqtt.subscribe("robot/r1/cmd/twist", 1);
    mqtt.subscribe("robot/r1/cmd/mode", 1);
    mqtt.subscribe("robot/r1/cmd/test_action", 1);

    setupMotors();
    setupEncoders();
    startLidar();

    lastMotorCommand = millis();  // prevents false timeout on first loop

    currentMode = defaultMode;
    Serial.print("[BOOT] Mode: ");
    Serial.println((int)currentMode);
}

void loop() {
    uint32_t now = millis();

    // ===== PRIORITY 1: MQTT POLLING (MAX FREQUENCY - NO TIME GATING) =====
    // Poll MQTT to drain message buffer and send ACKs quickly
    if (!mqtt.connected()) {
        if (now - lastReconnectCheck >= RECONNECT_CHECK_PERIOD_MS) {
            lastReconnectCheck = now;
            if (mqtt.connect(MQTT_HOST, MQTT_PORT)) {
                mqtt.subscribe("robot/r1/cmd/twist", 1);
                mqtt.subscribe("robot/r1/cmd/mode", 1);
                mqtt.subscribe("robot/r1/cmd/test_action", 1);
            }
        }
    } else {
        mqtt.poll();

        // Drain ALL pending incoming messages every loop for low-latency ACKs
        int messageSize = mqtt.parseMessage();
        while (messageSize > 0) {
            String topic = mqtt.messageTopic();

            const int MAX_PAYLOAD = 512;
            int payloadSize = constrain(messageSize, 0, MAX_PAYLOAD - 1);
            char payload[MAX_PAYLOAD];
            int i = 0;

            while (mqtt.available() && i < payloadSize) {
                payload[i++] = (char)mqtt.read();
            }
            while (mqtt.available()) mqtt.read();
            payload[i] = '\0';

            if (topic == "robot/r1/cmd/twist") {
                handleTwistCommand(payload);
            } else if (topic == "robot/r1/cmd/mode") {
                handleModeCommand(payload);
            } else if (topic == "robot/r1/cmd/test_action") {
                handleTestAction(payload);
            }

            messageSize = mqtt.parseMessage();
        }

        // Reconnect check on interval
        if (now - lastReconnectCheck >= RECONNECT_CHECK_PERIOD_MS) {
            lastReconnectCheck = now;
            if (WiFi.status() != WL_CONNECTED) {
                WiFi.begin(WIFI_SSID, WIFI_PASS);
            }
        }
    }

    // ===== PRIORITY 2: MOTOR SAFETY & DRIFT CORRECTION =====
    handleMotorTimeout();
    updateDriftCorrection();

    // ===== PRIORITY 3: SENSORS (Non-blocking intervals) =====

    // LiDAR must be polled every loop to drain Serial1 — scan rate (~7 Hz) is natural throttle
    serviceLidar();

    // Publish is separate from collection so mqtt.endMessage() never blocks lidar.update()
    if (scanPending) {
        publishCurrentScan(publishBins, pendingTs, pendingValidBins);
        scanPending = false;
    }

    // Publish encoder data on interval
    serviceEncoderData();

    // Publish status on interval
    publishStatus();

    // Diagnostics on interval
    if (currentMode == DIAGNOSTIC) {
        printDiagnostics();
    }

    // No blocking delay - loop runs as fast as possible to catch MQTT messages
}
