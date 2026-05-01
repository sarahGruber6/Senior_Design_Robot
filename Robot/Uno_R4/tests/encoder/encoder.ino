#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <FspTimer.h>
#include "config.h"
#ifdef __has_include
  #if __has_include("config_local.h")
    #include "config_local.h"
  #endif
#endif

// -------------------- Pins --------------------

#define LEFT_A   7
#define LEFT_B   4
#define RIGHT_A  5
#define RIGHT_B  6

// -------------------- MQTT --------------------

WiFiClient  net;
MqttClient  mqtt(net);

static const char TOPIC_ENCODER[] = "robot/r1/encoder";
static       char CLIENT_ID[]     = "Arduino_Encoders_R1";

// -------------------- Encoder state (ISR-shared) --------------------

volatile long    leftTicks  = 0;
volatile long    rightTicks = 0;

volatile uint8_t leftPrevState  = 0;
volatile uint8_t rightPrevState = 0;

// -------------------- Quadrature table --------------------

// index = (old_state << 2) | new_state
// state bits: A=MSB, B=LSB  →  00, 01, 10, 11
static const int8_t QUAD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

// -------------------- Timer --------------------

FspTimer encoderTimer;

// Called at SAMPLE_HZ (1 kHz) by the hardware timer ISR
static void encoderISR(timer_callback_args_t* /*arg*/) {
  // Read both encoders
  uint8_t leftNew  = ((digitalRead(LEFT_A)  ? 1 : 0) << 1)
                   |  (digitalRead(LEFT_B)  ? 1 : 0);
  uint8_t rightNew = ((digitalRead(RIGHT_A) ? 1 : 0) << 1)
                   |  (digitalRead(RIGHT_B) ? 1 : 0);

  // Update tick counters via lookup table
  leftTicks  -= QUAD_TABLE[(leftPrevState  << 2) | leftNew];
  rightTicks += QUAD_TABLE[(rightPrevState << 2) | rightNew];

  leftPrevState  = leftNew;
  rightPrevState = rightNew;
}

// Start the AGT/GPT timer at the requested frequency.
// Returns true on success.
static bool startEncoderTimer(float hz) {
  uint8_t timerType = AGT_TIMER;   // use AGT (lower priority, frees GPT for PWM)
  int8_t  channel   = FspTimer::get_available_timer(timerType);

  if (channel < 0) {               // fall back to GPT if no AGT available
    timerType = GPT_TIMER;
    channel   = FspTimer::get_available_timer(timerType);
  }
  if (channel < 0) return false;

  if (!encoderTimer.begin(TIMER_MODE_PERIODIC, timerType, channel,
                          hz, 0.0f, encoderISR)) {
    return false;
  }
  encoderTimer.setup_overflow_irq();   // enable the overflow interrupt
  encoderTimer.open();
  encoderTimer.start();
  return true;
}

// -------------------- WiFi / MQTT --------------------

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("\n[wifi] Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.print(" OK  IP=");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  if (mqtt.connected()) return;

  Serial.print("[mqtt] Connecting");
  while (!mqtt.connect(MQTT_HOST, MQTT_PORT)) {
    Serial.print(" err="); Serial.print(mqtt.connectError());
    delay(500);
  }
  Serial.println(" OK");
}

// -------------------- Publish --------------------

// Snapshot tick counters with interrupts disabled to prevent torn reads,
// then publish as a compact JSON message.
void publishEncoders() {
  if (!mqtt.connected()) return;

  // Atomic snapshot
  noInterrupts();
  long left  = leftTicks;
  long right = rightTicks;
  interrupts();

  mqtt.beginMessage(TOPIC_ENCODER);
  mqtt.print("{\"ts_ms\":");  mqtt.print(millis());
  mqtt.print(",\"left_ticks\":");  mqtt.print(left);
  mqtt.print(",\"right_ticks\":"); mqtt.print(right);
  mqtt.print("}");

  if (!mqtt.endMessage()) {
    Serial.println("[mqtt] publish failed");
  }
}

// -------------------- Setup --------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  // Configure encoder pins
  pinMode(LEFT_A,  INPUT_PULLUP);
  pinMode(LEFT_B,  INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT_PULLUP);

  // Seed previous states before enabling ISR
  leftPrevState  = ((digitalRead(LEFT_A)  ? 1 : 0) << 1) | (digitalRead(LEFT_B)  ? 1 : 0);
  rightPrevState = ((digitalRead(RIGHT_A) ? 1 : 0) << 1) | (digitalRead(RIGHT_B) ? 1 : 0);

  // Start 1 kHz hardware timer
  if (!startEncoderTimer(1000.0f)) {
    Serial.println("[ERROR] Could not allocate hardware timer — halting");
    while (true) {}
  }
  Serial.println("[timer] Encoder ISR running at 1 kHz");

  // Network
  mqtt.setId(CLIENT_ID);
  connectWiFi();
  connectMQTT();

  Serial.println("[setup] Ready");
}

// -------------------- Loop --------------------

// Timing constants (ms)
static const uint32_t PUBLISH_PERIOD_MS   =   50;   // 20 Hz publish rate
static const uint32_t RECONNECT_PERIOD_MS = 5000;
static const uint32_t STATUS_PERIOD_MS    = 1000;

void loop() {
  mqtt.poll();   // keep MQTT session alive, process ACKs

  uint32_t now = millis();

  // ---- Reconnect watchdog ----
  static uint32_t lastReconnect = 0;
  if (now - lastReconnect >= RECONNECT_PERIOD_MS) {
    lastReconnect = now;
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (!mqtt.connected())             connectMQTT();
  }

  // ---- Publish ticks ----
  static uint32_t lastPublish = 0;
  if (now - lastPublish >= PUBLISH_PERIOD_MS) {
    lastPublish = now;
    publishEncoders();
  }

  // ---- Serial status ----
  static uint32_t lastStatus = 0;
  if (now - lastStatus >= STATUS_PERIOD_MS) {
    lastStatus = now;

    noInterrupts();
    long l = leftTicks;
    long r = rightTicks;
    interrupts();

    Serial.print("[status] left="); Serial.print(l);
    Serial.print(" right=");        Serial.print(r);
    Serial.print(" mqtt=");         Serial.println(mqtt.connected() ? "OK" : "DOWN");
  }
}
