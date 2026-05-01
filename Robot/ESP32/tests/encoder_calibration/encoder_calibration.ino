// encoder_calibration.ino — ESP32-S3
//
// Uses attachInterrupt on all four encoder pins for clean quadrature decoding.
// No WiFi or MQTT needed.
//
// HOW TO USE:
//   1. Open Serial Monitor at 115200 baud.
//   2. Type 'r' + Enter to zero both counters.
//   3. Roll ONE wheel exactly ONE full revolution by hand, then stop.
//   4. The tick count shown is your TICKS_PER_REV.
//      Set it in Server/config_defaults.py and full_integration/config.h.
//
// Serial commands:
//   r  — reset both counters
//   l  — reset left counter only
//   R  — reset right counter only

// ---- Pin definitions — change to match your wiring ----
#define LEFT_A   9
#define LEFT_B   10
#define RIGHT_A  11
#define RIGHT_B  12

// ---- Quadrature table ----
static const int8_t QUAD_TABLE[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

// ---- Encoder state ----
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

void setup() {
    Serial.begin(115200);
    delay(500);

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

    Serial.println(F("\n=== ENCODER CALIBRATION (ESP32-S3 interrupt) ==="));
    Serial.println(F("Roll ONE wheel ONE full revolution, then stop."));
    Serial.println(F("The tick count shown is your TICKS_PER_REV."));
    Serial.println(F("Commands: r=reset both  l=reset left  R=reset right\n"));
}

void loop() {
    static uint32_t lastPrint = 0;
    static int32_t  lastL = 0, lastR = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'r') {
            portENTER_CRITICAL(&encoderMux);
            leftTicks = 0; rightTicks = 0;
            portEXIT_CRITICAL(&encoderMux);
            lastL = 0; lastR = 0;
            Serial.println(F("[RESET] Both counters zeroed"));
        } else if (c == 'l') {
            portENTER_CRITICAL(&encoderMux);
            leftTicks = 0;
            portEXIT_CRITICAL(&encoderMux);
            lastL = 0;
            Serial.println(F("[RESET] Left counter zeroed"));
        } else if (c == 'R') {
            portENTER_CRITICAL(&encoderMux);
            rightTicks = 0;
            portEXIT_CRITICAL(&encoderMux);
            lastR = 0;
            Serial.println(F("[RESET] Right counter zeroed"));
        }
    }

    uint32_t now = millis();
    if (now - lastPrint < 300) return;
    lastPrint = now;

    int32_t L, R;
    portENTER_CRITICAL(&encoderMux);
    L = leftTicks; R = rightTicks;
    portEXIT_CRITICAL(&encoderMux);

    Serial.print(F("LEFT="));  Serial.print(L);
    Serial.print(F("  (d="));  Serial.print(L - lastL);
    Serial.print(F(")    RIGHT=")); Serial.print(R);
    Serial.print(F("  (d="));  Serial.print(R - lastR);
    Serial.println(F(")"));

    lastL = L;
    lastR = R;
}
