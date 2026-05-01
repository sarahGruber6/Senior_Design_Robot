// encoder_calibration.ino
// Uses the same 1 kHz FspTimer approach as encoder.ino — no WiFi/MQTT needed.
//
// HOW TO USE:
//   1. Open Serial Monitor at 115200 baud.
//   2. Type 'r' + Enter to zero the counters.
//   3. Roll ONE wheel exactly ONE full revolution by hand, then stop.
//   4. The tick count shown is your TICKS_PER_REV.
//      Set it in Server/config_defaults.py and drive_calibration.ino.
//
// Serial commands:
//   r  — reset both counters
//   l  — reset left counter only
//   R  — reset right counter only

#include <FspTimer.h>

// right = ~760
// left = ~760

// ---- Pin definitions (match full_integration_test) ----
#define LEFT_A   7
#define LEFT_B   4
#define RIGHT_A  5
#define RIGHT_B  6

// ---- Quadrature table ----
static const int8_t QUAD_TABLE[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

// ---- Encoder state (timer ISR-shared) ----
volatile long    leftTicks      = 0;
volatile long    rightTicks     = 0;
volatile uint8_t leftPrevState  = 0;
volatile uint8_t rightPrevState = 0;

FspTimer encoderTimer;

// 1 kHz timer ISR — mirrors encoder.ino exactly (left inverted to match motor orientation)
static void encoderISR(timer_callback_args_t* /*arg*/) {
    uint8_t leftNew  = ((digitalRead(LEFT_A)  ? 1 : 0) << 1) | (digitalRead(LEFT_B)  ? 1 : 0);
    uint8_t rightNew = ((digitalRead(RIGHT_A) ? 1 : 0) << 1) | (digitalRead(RIGHT_B) ? 1 : 0);

    leftTicks  -= QUAD_TABLE[(leftPrevState  << 2) | leftNew];
    rightTicks += QUAD_TABLE[(rightPrevState << 2) | rightNew];

    leftPrevState  = leftNew;
    rightPrevState = rightNew;
}

static bool startEncoderTimer(float hz) {
    uint8_t timerType = AGT_TIMER;
    int8_t  channel   = FspTimer::get_available_timer(timerType);
    if (channel < 0) {
        timerType = GPT_TIMER;
        channel   = FspTimer::get_available_timer(timerType);
    }
    if (channel < 0) return false;

    if (!encoderTimer.begin(TIMER_MODE_PERIODIC, timerType, channel,
                            hz, 0.0f, encoderISR)) return false;
    encoderTimer.setup_overflow_irq();
    encoderTimer.open();
    encoderTimer.start();
    return true;
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

    if (!startEncoderTimer(1000.0f)) {
        Serial.println(F("[ERROR] Could not allocate hardware timer — halting"));
        while (true) {}
    }

    Serial.println(F("\n=== ENCODER CALIBRATION (1 kHz timer) ==="));
    Serial.println(F("Roll ONE wheel ONE full revolution, then stop."));
    Serial.println(F("The tick count shown is your TICKS_PER_REV."));
    Serial.println(F("Commands: r=reset both  l=reset left  R=reset right\n"));
}

void loop() {
    static uint32_t lastPrint = 0;
    static long lastL = 0, lastR = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'r') {
            noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts();
            lastL = 0; lastR = 0;
            Serial.println(F("[RESET] Both counters zeroed"));
        } else if (c == 'l') {
            noInterrupts(); leftTicks = 0; interrupts();
            lastL = 0;
            Serial.println(F("[RESET] Left counter zeroed"));
        } else if (c == 'R') {
            noInterrupts(); rightTicks = 0; interrupts();
            lastR = 0;
            Serial.println(F("[RESET] Right counter zeroed"));
        }
    }

    uint32_t now = millis();
    if (now - lastPrint < 300) return;
    lastPrint = now;

    long L, R;
    noInterrupts(); L = leftTicks; R = rightTicks; interrupts();

    Serial.print(F("LEFT="));  Serial.print(L);
    Serial.print(F("  (d="));  Serial.print(L - lastL);
    Serial.print(F(")    RIGHT=")); Serial.print(R);
    Serial.print(F("  (d="));  Serial.print(R - lastR);
    Serial.println(F(")"));

    lastL = L;
    lastR = R;
}
