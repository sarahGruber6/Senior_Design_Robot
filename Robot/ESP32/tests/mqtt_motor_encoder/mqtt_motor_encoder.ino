// mqtt_motor_encoder.ino — ESP32-S3
// Motor control via MQTT twist commands with closed-loop drift correction.
// Encoder quadrature via GPIO interrupts (attachInterrupt on all 4 pins).
// Motor PWM via ESP32 Arduino 3.x ledcAttach/ledcWrite API (8-bit, 1 kHz).
//
// MQTT topics (same as UNO R4 version):
//   subscribe: robot/r1/cmd/twist  {"v": m/s, "w": rad/s, "command_id": "...", "seq": n, "ttl_ms": n}
//   subscribe: robot/r1/cmd/mode   "MOTOR_MANUAL" | "MOTOR_CLOSED_LOOP" | "DIAGNOSTIC"
//   subscribe: robot/r1/cmd/test   any string
//   publish:   robot/r1/encoder    {"ts_ms":n,"left_ticks":n,"right_ticks":n}
//   publish:   robot/r1/evt/ack    {"command_id":"...","seq":n,"status":"received"}
//   publish:   robot/r1/telemetry  {"ts_ms":n,"mode":n,...}
//   publish:   robot/r1/diagnostics {"ts_ms":n,...}

#include <WiFi.h>
#include "esp_wifi.h"
#include <ArduinoMqttClient.h>
#include "config.h"
#ifdef __has_include
  #if __has_include("config_local.h")
    #include "config_local.h"
  #endif
#endif

// -------------------- Constants --------------------

enum TestMode { MOTOR_MANUAL, MOTOR_CLOSED_LOOP, DIAGNOSTIC };

TestMode currentMode = MOTOR_CLOSED_LOOP;

static const uint32_t ENCODER_PUBLISH_PERIOD_MS  = 50;
static const uint32_t STATUS_PUBLISH_PERIOD_MS   = 200;
static const uint32_t DIAG_REPORT_PERIOD_MS      = 5000;
static const uint32_t RECONNECT_CHECK_PERIOD_MS  = 5000;
static const uint32_t CORRECTION_PERIOD_MS       = 50;

static uint16_t MOTOR_TIMEOUT_MS = 500;

// -------------------- Encoder state --------------------

static const int8_t QUAD_TABLE[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

volatile int32_t leftTicks      = 0;
volatile int32_t rightTicks     = 0;
volatile uint8_t leftPrevState  = 0;
volatile uint8_t rightPrevState = 0;
static portMUX_TYPE encoderMux  = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR leftEncoderISR() {
    portENTER_CRITICAL_ISR(&encoderMux);
    uint8_t s = ((digitalRead(LEFT_A) ? 1 : 0) << 1) | (digitalRead(LEFT_B) ? 1 : 0);
    leftTicks += QUAD_TABLE[(leftPrevState << 2) | s];
    leftPrevState = s;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR rightEncoderISR() {
    portENTER_CRITICAL_ISR(&encoderMux);
    uint8_t s = ((digitalRead(RIGHT_A) ? 1 : 0) << 1) | (digitalRead(RIGHT_B) ? 1 : 0);
    rightTicks += QUAD_TABLE[(rightPrevState << 2) | s];
    rightPrevState = s;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

// -------------------- Motor state --------------------

int16_t  leftMotorPWM       = 0;
int16_t  rightMotorPWM      = 0;
uint32_t lastMotorCommand   = 0;
static const uint8_t RECENT_COMMAND_ID_COUNT = 8;
String   recentCommandIds[RECENT_COMMAND_ID_COUNT];
uint8_t  recentCommandIdSlot = 0;

int16_t  cmdLeftPWM         = 0;
int16_t  cmdRightPWM        = 0;
int32_t  leftTicksRef       = 0;
int32_t  rightTicksRef      = 0;
bool     straightMode       = false;
uint32_t lastCorrectionMs   = 0;

// -------------------- Timing --------------------

uint32_t lastEncoderPublish  = 0;
uint32_t lastStatusReport    = 0;
uint32_t lastDiagReport      = 0;
uint32_t lastReconnectCheck  = 0;
uint32_t motorCommandsReceived = 0;

// -------------------- WiFi / MQTT --------------------

const byte mac[] = {0x10, 0x51, 0xDB, 0x37, 0x4F, 0x14};

WiFiClient  net;
MqttClient  mqtt(net);

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

// -------------------- Motor setup --------------------

void setupMotors() {
    // ESP32 Arduino 3.x: ledcAttach(pin, freq_hz, resolution_bits)
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

    digitalWrite(RIGHT_DIR_PIN, rightPWM < 0 ? HIGH : LOW);
    ledcWrite(RIGHT_PWM_PIN,    (uint8_t)abs(rightPWM));

    leftMotorPWM  = leftPWM;
    rightMotorPWM = rightPWM;
}

void handleMotorTimeout() {
    if (currentMode == DIAGNOSTIC) return;
    if (millis() - lastMotorCommand > MOTOR_TIMEOUT_MS) {
        setMotorSpeed(0, 0);
        straightMode = false;
        cmdLeftPWM = cmdRightPWM = 0;
    }
}

// -------------------- Encoder setup --------------------

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
    if (currentMode != MOTOR_CLOSED_LOOP) return;
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

// -------------------- MQTT handlers --------------------

void handleTwistCommand(const char* payload) {
    if (currentMode == DIAGNOSTIC) return;

    float v = 0, w = 0;
    char  command_id[64] = {0};
    int   seq = -1;

    const char* p;
    if ((p = strstr(payload, "\"v\":")))          v   = atof(p + 4);
    if ((p = strstr(payload, "\"w\":")))          w   = atof(p + 4);
    if ((p = strstr(payload, "\"seq\":")))        seq = atoi(p + 6);
    if ((p = strstr(payload, "\"ttl_ms\":")))     MOTOR_TIMEOUT_MS = (uint16_t)atoi(p + 9);

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

    int16_t base  = (int16_t)(K_V * v);
    int16_t turn  = (int16_t)(K_W * w);
    int16_t lPWM  = (int16_t)(base * LEFT_TRIM) - turn;
    int16_t rPWM  = base + turn;

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
    Serial.print(" w=");        Serial.print(w, 2);
    Serial.print(" seq=");      Serial.println(seq);
}

void handleModeCommand(const char* payload) {
    if      (strstr(payload, "MOTOR_MANUAL"))      currentMode = MOTOR_MANUAL;
    else if (strstr(payload, "MOTOR_CLOSED_LOOP")) currentMode = MOTOR_CLOSED_LOOP;
    else if (strstr(payload, "DIAGNOSTIC"))        { currentMode = DIAGNOSTIC; setMotorSpeed(0, 0); }
    Serial.print("[MODE] "); Serial.println((int)currentMode);
}

void handleTestAction(const char* payload) {
    Serial.print("[TEST] "); Serial.println(payload);
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
    mqtt.endMessage();
}

// -------------------- Status / Diagnostics --------------------

void publishStatus() {
    if (!mqtt.connected()) return;
    uint32_t now = millis();
    if (now - lastStatusReport < STATUS_PUBLISH_PERIOD_MS) return;
    lastStatusReport = now;

    mqtt.beginMessage("robot/r1/telemetry");
    mqtt.print("{\"ts_ms\":"); mqtt.print(now);
    mqtt.print(",\"mode\":"); mqtt.print((int)currentMode);
    mqtt.print(",\"motor_commands\":"); mqtt.print(motorCommandsReceived);
    mqtt.print(",\"left_pwm\":"); mqtt.print(leftMotorPWM);
    mqtt.print(",\"right_pwm\":"); mqtt.print(rightMotorPWM);
    mqtt.print(",\"straight_mode\":"); mqtt.print(straightMode ? "true" : "false");
    mqtt.print("}");
    mqtt.endMessage();
}

void publishDiagnostics() {
    if (!mqtt.connected()) return;
    uint32_t now = millis();
    if (now - lastDiagReport < DIAG_REPORT_PERIOD_MS) return;
    lastDiagReport = now;

    int32_t L, R;
    portENTER_CRITICAL(&encoderMux); L = leftTicks; R = rightTicks; portEXIT_CRITICAL(&encoderMux);

    mqtt.beginMessage("robot/r1/diagnostics");
    mqtt.print("{\"ts_ms\":"); mqtt.print(now);
    mqtt.print(",\"uptime_ms\":"); mqtt.print(now);
    mqtt.print(",\"left_ticks\":"); mqtt.print(L);
    mqtt.print(",\"right_ticks\":"); mqtt.print(R);
    mqtt.print(",\"motor_commands\":"); mqtt.print(motorCommandsReceived);
    mqtt.print(",\"wifi_rssi\":"); mqtt.print(WiFi.RSSI());
    mqtt.print(",\"free_heap\":"); mqtt.print(ESP.getFreeHeap());
    mqtt.print("}");
    mqtt.endMessage();
}

// -------------------- Connection --------------------

void connectWiFi() {
    Serial.print("[WIFI] Connecting to "); Serial.println(WIFI_SSID);
    esp_wifi_set_mac(WIFI_IF_STA, &mac[0]);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.print("\n[WIFI] IP="); Serial.println(WiFi.localIP());
}

void connectMQTT() {
    Serial.print("[MQTT] Connecting to "); Serial.print(MQTT_HOST); Serial.print(":"); Serial.println(MQTT_PORT);
    if (!mqtt.connect(MQTT_HOST, MQTT_PORT)) {
        Serial.print("[MQTT] Failed: "); Serial.println(mqtt.connectError());
        return;
    }
    Serial.println("[MQTT] Connected");
    mqtt.subscribe("robot/r1/cmd/twist", 1);
    mqtt.subscribe("robot/r1/cmd/mode",  1);
    mqtt.subscribe("robot/r1/cmd/test",  1);
}

void checkConnections() {
    uint32_t now = millis();
    if (now - lastReconnectCheck < RECONNECT_CHECK_PERIOD_MS) return;
    lastReconnectCheck = now;
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (!mqtt.connected())             connectMQTT();
}

// -------------------- Setup / Loop --------------------

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== MQTT MOTOR ENCODER TEST (ESP32-S3) ===");

    setupMotors();
    setupEncoders();
    connectWiFi();
    connectMQTT();

    lastMotorCommand = millis();
    Serial.print("[BOOT] Mode: "); Serial.println((int)currentMode);
}

void loop() {
    checkConnections();

    if (mqtt.connected()) {
        // parseMessage() services the MQTT client. Calling mqtt.poll() here can
        // advance ArduinoMqttClient past a waiting publish payload.
        int msgSize = mqtt.parseMessage();
        while (msgSize > 0) {
            String topic = mqtt.messageTopic();
            char   payload[512];
            int    i = 0;
            while (mqtt.available() && i < (int)sizeof(payload) - 1) payload[i++] = (char)mqtt.read();
            while (mqtt.available()) mqtt.read();
            payload[i] = '\0';

            if      (topic == "robot/r1/cmd/twist") handleTwistCommand(payload);
            else if (topic == "robot/r1/cmd/mode")  handleModeCommand(payload);
            else if (topic == "robot/r1/cmd/test")  handleTestAction(payload);

            msgSize = mqtt.parseMessage();
        }
    }

    serviceEncoderData();
    updateDriftCorrection();
    handleMotorTimeout();
    publishStatus();
    publishDiagnostics();
}
