// drive_calibration.ino
// Calibrates K_V, K_W, and LEFT_TRIM (toe correction).
//
// Setup: robot on the floor with a tape measure and something to mark heading.
//
// Serial commands (115200 baud, send with newline):
//
//   f  — forward run at CAL_PWM for CAL_TIME_MS
//          → measure distance traveled, enter: d<mm>   e.g. d920
//          → prints K_V, WHEEL_DIAMETER_MM, and LEFT_TRIM
//
//   s  — spin in place (left back, right forward) at CAL_PWM for CAL_TIME_MS
//          → measure angle turned, enter: a<deg>       e.g. a270
//          → prints K_W
//
//   t  — toe verification: drive forward with LEFT_TRIM applied (set below)
//          → watch which way the robot drifts; re-run 'f' and re-enter distance
//          → iterate until robot drives straight
//
//   r  — reset encoder counters

#include <FspTimer.h>
#include "config.h"
#ifdef __has_include
  #if __has_include("config_local.h")
    #include "config_local.h"
  #endif
#endif

// ---- Physical constants — update from encoder_calibration and measurement ----
static const float TICKS_PER_REV  = 760.0f;  // from encoder_calibration sketch
static const float WHEEL_DIA_MM   = 96.0f;   // measured
static const float AXLE_WIDTH_MM  = 480.0f;  // center-to-center wheel contact (mm)

// ---- Toe correction — paste result of 'f' run here, then re-flash and run 't' ----
// 1.0 = no correction. < 1.0 slows left motor, > 1.0 speeds it up.
static float leftTrim = LEFT_TRIM;  // pulls from config.h #define LEFT_TRIM

// ---- Calibration run parameters ----
static const int      CAL_PWM     = 120;    // 0–255
static const uint32_t CAL_TIME_MS = 2000;   // ms per run
static const int      RAMP_STEP   = 8;      // PWM per ramp tick, avoids wheel wind-up slips
static const uint32_t RAMP_MS     = 25;     // ms per ramp tick

// ---- Pin definitions ----
#define LEFT_A        7
#define LEFT_B        4
#define RIGHT_A       5
#define RIGHT_B       6
#define LEFT_PWM_PIN  9
#define LEFT_DIR_PIN  8
#define RIGHT_PWM_PIN 11
#define RIGHT_DIR_PIN 13

// ---- Quadrature ----
static const int8_t QUAD_TABLE[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

volatile long    leftTicks      = 0;
volatile long    rightTicks     = 0;
volatile uint8_t leftPrevState  = 0;
volatile uint8_t rightPrevState = 0;

FspTimer encoderTimer;

static void encoderISR(timer_callback_args_t* /*arg*/) {
    uint8_t lNew = ((digitalRead(LEFT_A)  ? 1 : 0) << 1) | (digitalRead(LEFT_B)  ? 1 : 0);
    uint8_t rNew = ((digitalRead(RIGHT_A) ? 1 : 0) << 1) | (digitalRead(RIGHT_B) ? 1 : 0);
    leftTicks  -= QUAD_TABLE[(leftPrevState  << 2) | lNew];
    rightTicks += QUAD_TABLE[(rightPrevState << 2) | rNew];
    leftPrevState  = lNew;
    rightPrevState = rNew;
}

void setupEncoders() {
    pinMode(LEFT_A,  INPUT_PULLUP); pinMode(LEFT_B,  INPUT_PULLUP);
    pinMode(RIGHT_A, INPUT_PULLUP); pinMode(RIGHT_B, INPUT_PULLUP);
    leftPrevState  = ((digitalRead(LEFT_A)  ? 1 : 0) << 1) | (digitalRead(LEFT_B)  ? 1 : 0);
    rightPrevState = ((digitalRead(RIGHT_A) ? 1 : 0) << 1) | (digitalRead(RIGHT_B) ? 1 : 0);

    uint8_t tt = AGT_TIMER;
    int8_t  ch = FspTimer::get_available_timer(tt);
    if (ch < 0) { tt = GPT_TIMER; ch = FspTimer::get_available_timer(tt); }
    if (ch < 0) { Serial.println(F("[ERROR] No timer")); while (true) {} }
    encoderTimer.begin(TIMER_MODE_PERIODIC, tt, ch, 1000.0f, 0.0f, encoderISR);
    encoderTimer.setup_overflow_irq();
    encoderTimer.open();
    encoderTimer.start();
}

void setupMotors() {
    pinMode(LEFT_PWM_PIN,  OUTPUT); pinMode(LEFT_DIR_PIN,  OUTPUT);
    pinMode(RIGHT_PWM_PIN, OUTPUT); pinMode(RIGHT_DIR_PIN, OUTPUT);
    analogWrite(LEFT_PWM_PIN, 0);   analogWrite(RIGHT_PWM_PIN, 0);
    digitalWrite(LEFT_DIR_PIN, LOW); digitalWrite(RIGHT_DIR_PIN, LOW);
}

void setMotors(int16_t lPWM, int16_t rPWM) {
    lPWM = constrain(lPWM, -255, 255);
    rPWM = constrain(rPWM, -255, 255);
    digitalWrite(LEFT_DIR_PIN,  lPWM < 0 ? HIGH : LOW);
    analogWrite(LEFT_PWM_PIN,   abs(lPWM));
    digitalWrite(RIGHT_DIR_PIN, rPWM < 0 ? HIGH : LOW);
    analogWrite(RIGHT_PWM_PIN,  abs(rPWM));
}

void rampMotors(int16_t fromL, int16_t fromR, int16_t toL, int16_t toR) {
    int16_t l = fromL;
    int16_t r = fromR;
    while (l != toL || r != toR) {
        if (l < toL) l = min(l + RAMP_STEP, (int)toL);
        else if (l > toL) l = max(l - RAMP_STEP, (int)toL);

        if (r < toR) r = min(r + RAMP_STEP, (int)toR);
        else if (r > toR) r = max(r - RAMP_STEP, (int)toR);

        setMotors(l, r);
        delay(RAMP_MS);
    }
}

// Blocking run; returns tick deltas (both as positive values)
void doRun(int16_t lPWM, int16_t rPWM, long &dLeft, long &dRight) {
    noInterrupts(); long l0 = leftTicks; long r0 = rightTicks; interrupts();
    rampMotors(0, 0, lPWM, rPWM);
    delay(CAL_TIME_MS);
    rampMotors(lPWM, rPWM, 0, 0);
    noInterrupts(); dLeft = leftTicks - l0; dRight = rightTicks - r0; interrupts();
    dLeft  = abs(dLeft);
    dRight = abs(dRight);
}

// ---- State machine ----
enum WaitState { IDLE, WAIT_DIST, WAIT_ANGLE };
static WaitState waitState = IDLE;
static long savedL = 0, savedR = 0;

// trimApplied: the LEFT_TRIM multiplier that was in effect during this run.
// Pass 1.0 for 'f' (no trim); pass leftTrim for 't' (trim was active).
// The suggested new LEFT_TRIM is always an absolute value ready to paste in config.h.
void printResults(long dL, long dR, float trimApplied) {
    Serial.print(F("  Left ticks : ")); Serial.println(dL);
    Serial.print(F("  Right ticks: ")); Serial.println(dR);

    if (dL == 0 || dR == 0) { Serial.println(F("  [no movement detected]")); return; }

    // Absolute trim needed = trimApplied * (dR/dL)
    // Because: left traveled (dL) at (CAL_PWM * trimApplied).
    // To make dL == dR we need to multiply left PWM by (dR/dL),
    // so new absolute trim = trimApplied * dR/dL.
    float newTrim = trimApplied * ((float)dR / (float)dL);
    int pctDiff   = (int)(abs(dL - dR) * 100L / max(dL, dR));

    Serial.print(F("  Imbalance  : ")); Serial.print(pctDiff); Serial.println(F("%"));
    if (pctDiff > 2) {
        Serial.print(F("  >> Toe drift. New absolute LEFT_TRIM = ")); Serial.println(newTrim, 4);
        Serial.println(F("     Paste into config.h then re-flash and run 't' to verify."));
    } else {
        Serial.println(F("  >> Balanced (within 2%) — LEFT_TRIM is good."));
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    setupMotors();
    setupEncoders();

    Serial.println(F("\n=== DRIVE CALIBRATION ==="));
    Serial.print(F("CAL_PWM=")); Serial.print(CAL_PWM);
    Serial.print(F("  CAL_TIME=")); Serial.print(CAL_TIME_MS); Serial.println(F("ms"));
    Serial.print(F("LEFT_TRIM=")); Serial.println(leftTrim, 4);
    Serial.println(F("Commands:  f=forward  s=spin  t=toe-verify  v=live-verify  r=reset"));
    Serial.println(F("After run, enter: d<mm>  or  a<deg>\n"));
}

void loop() {
    static char    buf[16];
    static uint8_t bufLen = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            buf[bufLen] = '\0';
            String cmd = String(buf);
            bufLen = 0;
            if (cmd.length() == 0) continue;

            // ---- reset ----
            if (cmd == "r") {
                noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts();
                waitState = IDLE;
                Serial.println(F("[RESET]"));

            // ---- forward run (no trim — gives absolute LEFT_TRIM) ----
            } else if (cmd == "f" && waitState == IDLE) {
                Serial.println(F("\n--- FORWARD RUN (no trim) ---"));
                Serial.println(F("Running..."));
                doRun(CAL_PWM, CAL_PWM, savedL, savedR);
                printResults(savedL, savedR, 1.0f);
                Serial.println(F("Measure distance (mm) and enter: d<mm>  e.g. d920"));
                waitState = WAIT_DIST;

            // ---- toe verification (trim applied — iterates toward correct absolute trim) ----
            } else if (cmd == "t" && waitState == IDLE) {
                Serial.println(F("\n--- TOE VERIFY (trim applied) ---"));
                Serial.print(F("Applying LEFT_TRIM=")); Serial.println(leftTrim, 4);
                Serial.println(F("Running..."));
                long dL, dR;
                int16_t trimmedPWM = (int16_t)(CAL_PWM * leftTrim);
                doRun(trimmedPWM, CAL_PWM, dL, dR);
                // Pass leftTrim so the recommendation is an absolute value, not relative
                printResults(dL, dR, leftTrim);
                Serial.println(F("Update LEFT_TRIM in config.h, re-flash, run 't' again to iterate."));

            // ---- live closed-loop: adjusts leftTrim in real time, reports convergence ----
            } else if (cmd == "v" && waitState == IDLE) {
                Serial.println(F("\n--- LIVE CLOSED-LOOP TOE CALIBRATION ---"));
                Serial.print(F("Starting LEFT_TRIM=")); Serial.println(leftTrim, 4);
                Serial.println(F("Adjusting trim every 500ms until balanced. Send any key to stop.\n"));
                Serial.println(F("  [trim]      dL    dR   ratio  imbal%"));

                noInterrupts(); long lPrev = leftTicks; long rPrev = rightTicks; interrupts();
                setMotors((int16_t)(CAL_PWM * leftTrim), CAL_PWM);

                uint32_t lastTick = millis();
                while (true) {
                    if (Serial.available()) { while (Serial.available()) Serial.read(); break; }
                    if (millis() - lastTick < 500) continue;
                    lastTick = millis();

                    noInterrupts(); long lNow = leftTicks; long rNow = rightTicks; interrupts();
                    long dL = abs(lNow - lPrev);
                    long dR = abs(rNow - rPrev);
                    lPrev = lNow;
                    rPrev = rNow;

                    // Update trim: new trim = current trim * (dR/dL) so left matches right
                    if (dL > 5 && dR > 5) {
                        leftTrim = constrain(leftTrim * ((float)dR / (float)dL), 0.5f, 2.0f);
                        setMotors((int16_t)(CAL_PWM * leftTrim), CAL_PWM);
                    }

                    int pct = (dL + dR > 0) ? (int)(abs(dL - dR) * 100L / max(dL, dR)) : 0;
                    Serial.print(F("  "));
                    Serial.print(leftTrim, 4);
                    Serial.print(F("   "));   Serial.print(dL);
                    Serial.print(F("  "));    Serial.print(dR);
                    Serial.print(F("   "));
                    Serial.print((dR > 0) ? (float)dL / (float)dR : 0.0f, 3);
                    Serial.print(F("   "));   Serial.print(pct); Serial.println(F("%"));
                }
                setMotors(0, 0);
                Serial.println(F("\n--- STOPPED ---"));
                Serial.print(F("Converged LEFT_TRIM = ")); Serial.println(leftTrim, 4);
                Serial.println(F("Paste into config.h and re-flash full_integration_test."));

            // ---- spin run ----
            } else if (cmd == "s" && waitState == IDLE) {
                Serial.println(F("\n--- SPIN RUN ---"));
                Serial.println(F("Mark heading. Running CCW..."));
                doRun(-CAL_PWM, CAL_PWM, savedL, savedR);
                Serial.print(F("  Left ticks : ")); Serial.println(savedL);
                Serial.print(F("  Right ticks: ")); Serial.println(savedR);
                Serial.println(F("Measure angle turned (deg) and enter: a<deg>  e.g. a270"));
                waitState = WAIT_ANGLE;

            // ---- distance entered ----
            } else if (cmd.startsWith("d") && waitState == WAIT_DIST) {
                float measuredMm = cmd.substring(1).toFloat();
                if (measuredMm <= 0.0f) { Serial.println(F("Bad value")); return; }

                float avgTicks  = (savedL + savedR) / 2.0f;
                float mmPerTick = measuredMm / avgTicks;
                float wheelCirc = mmPerTick * TICKS_PER_REV;
                float diamMm    = wheelCirc / PI;
                float speedMs   = (measuredMm / (float)CAL_TIME_MS) * 1000.0f;
                float kv        = (float)CAL_PWM / speedMs;
                float newTrim   = (float)savedR / (float)savedL;

                Serial.println(F("\n--- FORWARD RESULTS ---"));
                Serial.print(F("  Measured distance  : ")); Serial.print(measuredMm, 1); Serial.println(F(" mm"));
                Serial.print(F("  Speed              : ")); Serial.print(speedMs, 3);    Serial.println(F(" m/s"));
                Serial.println(F("  >> Paste into config.h:"));
                Serial.print(F("     #define K_V        ")); Serial.println(kv, 1);
                Serial.println(F("  >> Paste into config_defaults.py + drive_calibration.ino:"));
                Serial.print(F("     WHEEL_DIAMETER_MM = ")); Serial.println(diamMm, 1);
                Serial.println(F("  >> Toe correction (LEFT_TRIM):"));
                Serial.print(F("     Suggested: #define LEFT_TRIM  ")); Serial.println(newTrim, 4);
                Serial.println(F("  Set LEFT_TRIM in config.h, re-flash, run 't' to verify."));
                leftTrim = newTrim;
                waitState = IDLE;

            // ---- angle entered ----
            } else if (cmd.startsWith("a") && waitState == WAIT_ANGLE) {
                float measuredDeg = cmd.substring(1).toFloat();
                if (measuredDeg <= 0.0f) { Serial.println(F("Bad value")); return; }

                float omegaRads = (measuredDeg * (PI / 180.0f)) / ((float)CAL_TIME_MS / 1000.0f);
                float kw        = (float)CAL_PWM / omegaRads;

                Serial.println(F("\n--- SPIN RESULTS ---"));
                Serial.print(F("  Measured angle : ")); Serial.print(measuredDeg, 1); Serial.println(F(" deg"));
                Serial.print(F("  Angular rate   : ")); Serial.print(omegaRads, 3);   Serial.println(F(" rad/s"));
                Serial.println(F("  >> Paste into config.h:"));
                Serial.print(F("     #define K_W        ")); Serial.println(kw, 1);
                waitState = IDLE;

            } else {
                Serial.println(F("Commands: f  s  t  v  r  d<mm>  a<deg>"));
                waitState = IDLE;
            }
        } else {
            if (bufLen < sizeof(buf) - 1) buf[bufLen++] = c;
        }
    }
}
