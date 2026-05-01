#include "YDLidarG2.h"
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include "config.h"
#ifdef __has_include
  #if __has_include("config_local.h")
    #include "config_local.h"
  #endif
#endif

// -------------------- Constants --------------------

// LiDAR
static const uint16_t LIDAR_MIN_MM        = 150;
static const uint16_t LIDAR_MAX_MM        = 8000;
static const uint16_t MIN_VALID_BINS      = 200;

// MQTT / system
static const uint32_t RECONNECT_PERIOD_MS = 5000;
static const uint32_t STATUS_PERIOD_MS    = 3000;

// Encoder publish rate
static const uint32_t ENCODER_PUBLISH_PERIOD_MS = 50;  // 20 Hz

// SCN3 frame layout (734 bytes total):
// [0..3]   magic  "SCN3"
// [4..7]   seq    uint32 LE
// [8..11]  ts_ms  uint32 LE
// [12..13] valid  uint16 LE
// [14..733] 360 x uint16 LE distances in mm (0 = invalid/no return)
static const size_t LIDAR_PAYLOAD_SIZE = 4 + 4 + 4 + 2 + 360 * 2;

// Topics
static const char TOPIC_LIDAR[]   = "robot/r1/lidar";
static const char TOPIC_ENCODER[] = "robot/r1/encoder";
static char CLIENT_ID[]           = "Arduino_R1_Lidar_Encoder";

// Encoder pins
#define LEFT_A   3
#define LEFT_B   4
#define RIGHT_A  8
#define RIGHT_B  9

// -------------------- Globals --------------------

YDLidarG2 lidar(Serial1, 3);

WiFiClient net;
MqttClient mqtt(net);

// LiDAR state
bool scanning = false;
uint32_t lidarSeq = 0;
uint16_t distanceBins[360];

uint32_t scansCompleted   = 0;
uint32_t scansPublished   = 0;
uint32_t scansRejected    = 0;
uint32_t mqttPublishFails = 0;

// Encoder state
volatile long leftTicks = 0;
volatile long rightTicks = 0;
uint32_t lastEncoderPublishMs = 0;

// Housekeeping
uint32_t lastReconnectMs = 0;
uint32_t lastStatusMs    = 0;

// -------------------- Helpers --------------------

static bool isValidRangeMm(uint16_t d) {
  return d >= LIDAR_MIN_MM && d <= LIDAR_MAX_MM;
}

uint16_t countValidBins(const uint16_t bins[360]) {
  uint16_t n = 0;
  for (int i = 0; i < 360; ++i) {
    if (bins[i] > 0) n++;
  }
  return n;
}

void copyScanBinsFromLidar(uint16_t bins[360]) {
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
  buf[(*pos)++] = (uint8_t)(v & 0xFF);
  buf[(*pos)++] = (uint8_t)((v >> 8) & 0xFF);
  buf[(*pos)++] = (uint8_t)((v >> 16) & 0xFF);
  buf[(*pos)++] = (uint8_t)((v >> 24) & 0xFF);
}

// -------------------- Encoder ISRs --------------------

// Simple quadrature decode: trigger on A edges, read B for direction.
// If sign is backwards for your mounting, just swap ++ and -- in that ISR.
void leftISR() {
  if (digitalRead(LEFT_B)) leftTicks++;
  else                     leftTicks--;
}

void rightISR() {
  if (digitalRead(RIGHT_B)) rightTicks++;
  else                      rightTicks--;
}

// -------------------- WiFi / MQTT --------------------

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("\nConnecting WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  while (WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
    delay(200);
  }

  Serial.print(" OK  IP=");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  if (mqtt.connected()) return;

  Serial.print("Connecting MQTT...");
  while (!mqtt.connect(MQTT_HOST, MQTT_PORT)) {
    Serial.print(" err=");
    Serial.print(mqtt.connectError());
    delay(500);
  }
  Serial.println(" OK");
}

// -------------------- LiDAR --------------------

void startLidar() {
  if (!lidar.begin()) {
    Serial.println("ERROR: lidar.begin() failed");
    while (1) {}
  }

  if (!lidar.startScan()) {
    Serial.println("ERROR: startScan() failed");
    while (1) {}
  }

  scanning = true;
  Serial.println("SCAN_START");
}

void publishCurrentLidarScan(uint32_t tsMs, uint16_t validBins) {
  if (!mqtt.connected()) return;

  uint8_t payload[LIDAR_PAYLOAD_SIZE];
  size_t pos = 0;

  payload[pos++] = 'S';
  payload[pos++] = 'C';
  payload[pos++] = 'N';
  payload[pos++] = '3';

  writeU32LE(payload, &pos, lidarSeq++);
  writeU32LE(payload, &pos, tsMs);
  writeU16LE(payload, &pos, validBins);

  for (int i = 0; i < 360; ++i) {
    writeU16LE(payload, &pos, distanceBins[i]);
  }

  mqtt.beginMessage(TOPIC_LIDAR, (unsigned long)pos, false, 0);
  mqtt.write(payload, pos);

  if (!mqtt.endMessage()) {
    mqttPublishFails++;
    return;
  }

  scansPublished++;
}

void serviceLidar() {
  if (!scanning) return;
  if (!lidar.update()) return;

  scansCompleted++;

  copyScanBinsFromLidar(distanceBins);
  uint16_t validBins = countValidBins(distanceBins);

  if (validBins < MIN_VALID_BINS) {
    scansRejected++;
    return;
  }

  publishCurrentLidarScan(millis(), validBins);
}

// -------------------- Encoders --------------------

void publishEncoders() {
  if (!mqtt.connected()) return;

  long left, right;
  uint32_t tsMs = millis();

  noInterrupts();
  left = leftTicks;
  right = rightTicks;
  interrupts();

  mqtt.beginMessage(TOPIC_ENCODER, 0, false, 0);
  mqtt.print("{\"ts_ms\":");
  mqtt.print(tsMs);
  mqtt.print(",\"left_ticks\":");
  mqtt.print(left);
  mqtt.print(",\"right_ticks\":");
  mqtt.print(right);
  mqtt.print("}");

  if (!mqtt.endMessage()) {
    mqttPublishFails++;
    return;
  }
}

void serviceEncoders() {
  uint32_t now = millis();
  if (now - lastEncoderPublishMs >= ENCODER_PUBLISH_PERIOD_MS) {
    lastEncoderPublishMs = now;
    publishEncoders();
  }
}

// -------------------- Setup / Loop --------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Encoder pins
  pinMode(LEFT_A, INPUT_PULLUP);
  pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightISR, CHANGE);

  mqtt.setId(CLIENT_ID);
  mqtt.setKeepAliveInterval(30 * 1000);
  mqtt.setTxPayloadSize(LIDAR_PAYLOAD_SIZE + 64);

  connectWiFi();
  connectMQTT();

  Serial.println("BOOT");
  startLidar();
  Serial.println("ENCODERS_READY");
}

void loop() {
  mqtt.poll();

  uint32_t now = millis();

  if (now - lastReconnectMs >= RECONNECT_PERIOD_MS) {
    lastReconnectMs = now;

    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (!mqtt.connected()) connectMQTT();
  }

  serviceLidar();
  serviceEncoders();

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;

    long left, right;
    noInterrupts();
    left = leftTicks;
    right = rightTicks;
    interrupts();

    Serial.print("[status] lidar_completed=");
    Serial.print(scansCompleted);
    Serial.print(" lidar_published=");
    Serial.print(scansPublished);
    Serial.print(" lidar_rejected=");
    Serial.print(scansRejected);
    Serial.print(" mqttFail=");
    Serial.print(mqttPublishFails);
    Serial.print(" left_ticks=");
    Serial.print(left);
    Serial.print(" right_ticks=");
    Serial.print(right);
    Serial.print(" mqtt=");
    Serial.println(mqtt.connected() ? "OK" : "DOWN");
  }
}