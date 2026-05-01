#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <FspTimer.h>
#include "config.h"
#ifdef __has_include
  #if __has_include("config_local.h")
    #include "config_local.h"
  #endif
#endif

// -------------------- Constants --------------------

enum TestMode {
    MOTOR_MANUAL,
    MOTOR_CLOSED_LOOP,
    DIAGNOSTIC
};

TestMode defaultMode = MOTOR_CLOSED_LOOP;
TestMode currentMode = defaultMode;

// Non-blocking timing intervals
static const uint32_t ENCODER_PUBLISH_PERIOD_MS = 50;    // Publish ~20 Hz
static const uint32_t STATUS_PUBLISH_PERIOD_MS = 200;    // Status ~5 Hz
static const uint32_t DIAG_REPORT_PERIOD_MS = 5000;      // Diagnostics every 5s
static const uint32_t RECONNECT_CHECK_PERIOD_MS = 5000;  // Check connection ~0.2 Hz

// Motor constants — K_V and K_W are defined in config.h for easy tuning
static uint16_t MOTOR_TIMEOUT_MS = 500;

// Encoder Pins
#define RIGHT_A  5
#define RIGHT_B  6
#define LEFT_A   7
#define LEFT_B   4

// ==================== GLOBALS ====================

WiFiClient net;
MqttClient mqtt(net);

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

// Timing for non-blocking processing
uint32_t lastEncoderPublish = 0;
uint32_t lastStatusReport = 0;
uint32_t lastDiagReport = 0;
uint32_t lastReconnectCheck = 0;

// Statistics
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
    if (currentMode == MOTOR_MANUAL || currentMode == MOTOR_CLOSED_LOOP) {
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
    if (currentMode == DIAGNOSTIC) {
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

    const char *timeout_pos = strstr(payload, "\"ttl_ms\":");
    if (timeout_pos != NULL) {
        MOTOR_TIMEOUT_MS = atof(timeout_pos + 9);
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
    if (strstr(payload, "MOTOR_MANUAL") != NULL) {
        currentMode = MOTOR_MANUAL;
    } else if (strstr(payload, "MOTOR_CLOSED_LOOP") != NULL) {
        currentMode = MOTOR_CLOSED_LOOP;
    } else if (strstr(payload, "DIAGNOSTIC") != NULL) {
        currentMode = DIAGNOSTIC;
        setMotorSpeed(0, 0);
    }

    Serial.print("[MODE] ");
    Serial.println((int)currentMode);
}

void handleTestAction(const char *payload) {
    Serial.print("[TEST] ");
    Serial.println(payload);
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
    if (currentMode != MOTOR_CLOSED_LOOP) return;
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

    mqtt.beginMessage("robot/r1/telemetry");
    mqtt.print("{\"ts_ms\":");
    mqtt.print(now);
    mqtt.print(",\"mode\":");
    mqtt.print((int)currentMode);
    mqtt.print(",\"motor_commands\":");
    mqtt.print(motorCommandsReceived);
    mqtt.print(",\"left_pwm\":");
    mqtt.print(leftMotorPWM);
    mqtt.print(",\"right_pwm\":");
    mqtt.print(rightMotorPWM);
    mqtt.print(",\"straight_mode\":");
    mqtt.print(straightMode ? "true" : "false");
    mqtt.print("}");
    mqtt.endMessage();
}

// -------------------- DIAGNOSTICS --------------------

void publishDiagnostics() {
    if (!mqtt.connected()) return;

    uint32_t now = millis();
    if (now - lastDiagReport < DIAG_REPORT_PERIOD_MS) return;
    lastDiagReport = now;

    long left, right;
    noInterrupts(); left = leftTicks; right = rightTicks; interrupts();

    mqtt.beginMessage("robot/r1/diagnostics");
    mqtt.print("{\"ts_ms\":");
    mqtt.print(now);
    mqtt.print(",\"uptime_ms\":");
    mqtt.print(now);
    mqtt.print(",\"left_ticks\":");
    mqtt.print(left);
    mqtt.print(",\"right_ticks\":");
    mqtt.print(right);
    mqtt.print(",\"motor_commands\":");
    mqtt.print(motorCommandsReceived);
    mqtt.print(",\"wifi_rssi\":");
    mqtt.print(WiFi.RSSI());
    mqtt.print(",\"free_heap\":");
    mqtt.print(0);  // Arduino doesn't have heap info like ESP32
    mqtt.print("}");
    mqtt.endMessage();
}

// -------------------- MQTT SETUP --------------------

void setupMQTT() {
    mqtt.setId("mqtt_motor_encoder_test");
    mqtt.onMessage(onMqttMessage);

    // Subscribe to command topics
    mqtt.subscribe("robot/r1/cmd/twist", 1);
    mqtt.subscribe("robot/r1/cmd/mode", 1);
    mqtt.subscribe("robot/r1/cmd/test", 1);
}

void onMqttMessage(int messageSize) {
    String topic = mqtt.messageTopic();
    String payload = "";

    while (mqtt.available()) {
        payload += (char)mqtt.read();
    }

    if (topic == "robot/r1/cmd/twist") {
        handleTwistCommand(payload.c_str());
    } else if (topic == "robot/r1/cmd/mode") {
        handleModeCommand(payload.c_str());
    } else if (topic == "robot/r1/cmd/test") {
        handleTestAction(payload.c_str());
    }
}

// -------------------- WIFI/MQTT CONNECTION --------------------

void connectWiFi() {
    Serial.print("[WIFI] Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("[WIFI] Connected");
    Serial.print("[WIFI] IP: ");
    Serial.println(WiFi.localIP());
}

void connectMQTT() {
    Serial.print("[MQTT] Connecting to ");
    Serial.print(MQTT_HOST);
    Serial.print(":");
    Serial.println(MQTT_PORT);

    if (!mqtt.connect(MQTT_HOST, MQTT_PORT)) {
        Serial.print("[MQTT] Connection failed: ");
        Serial.println(mqtt.connectError());
        return;
    }

    Serial.println("[MQTT] Connected");
    setupMQTT();
}

void checkConnections() {
    uint32_t now = millis();
    if (now - lastReconnectCheck < RECONNECT_CHECK_PERIOD_MS) return;
    lastReconnectCheck = now;

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] Reconnecting...");
        connectWiFi();
    }

    if (!mqtt.connected()) {
        Serial.println("[MQTT] Reconnecting...");
        connectMQTT();
    }
}

// -------------------- SETUP & LOOP --------------------

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== MQTT MOTOR ENCODER TEST ===");
    Serial.print("Mode: ");
    Serial.println((int)currentMode);

    setupMotors();
    setupEncoders();

    connectWiFi();
    connectMQTT();
}

void loop() {
    checkConnections();
    mqtt.poll();

    // Service encoder data and publish
    serviceEncoderData();

    // Update drift correction (closed-loop mode only)
    updateDriftCorrection();

    // Handle motor timeout
    handleMotorTimeout();

    // Publish status and diagnostics
    publishStatus();
    publishDiagnostics();
}