// first draft of sending lidar points to MQTT and having them converted to a map! might try to integrate
// motor controls as well, but no encoder yet so that will be limited for this version

#include <WiFiS3.h>
#include <math.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>

#include "config.h"
#ifdef __has_include
  #if __has_include("config_local.h")
    #include "config_local.h"
  #endif
#endif

#include <CytronMotorDriver.h>
#include "YDLidarG2_Uno.h"

// ------ Network / MQTT ------

const byte mac[] = {0x10, 0x51, 0xDB, 0x37, 0x4F, 0x14};

const char topic_job[]   = "robot/r1/cmd/job";
const char topic_done[]  = "robot/r1/evt/done";
const char topic_ack[]   = "robot/r1/evt/ack";
const char topic_twist[] = "robot/r1/cmd/twist";
const char topic_lidar[] = "robot/r1/lidar";

WiFiClient net;
MqttClient mqtt(net);

char clientId[] = "Arduino_R1";
bool subscribed = false;
String lastCommandId = "";

// ------ Motors ------

CytronMD motorL(PWM_DIR, 9, 8);
CytronMD motorR(PWM_DIR, 11, 13);

float K_V = 350.0f;   // PWM per m/s (needs tuning)
float K_W = 140.0f;   // PWM per rad/s

volatile uint32_t cmdExpiresAtMs = 0;
volatile int targetL = 0;
volatile int targetR = 0;

// ------ Lidar ------

YDLidarG2_Uno lidar;
YDLidarG2_Uno::DeviceHealth health;
YDLidarG2_Uno::DeviceInfo info;

bool scanning = false;
uint32_t lidarSeq = 0;

// publish settings
static const int LIDAR_MIN_MM = 150;
static const int LIDAR_MAX_MM = 10000;
static const int LIDAR_BIN_COUNT = 360;   

// Heuristic scan assembly tuning
static const uint32_t LIDAR_MIN_SCAN_MS       = 85;   // ~10 Hz revolution ≈ 100 ms
static const uint32_t LIDAR_MAX_SCAN_MS       = 250;  // reset if we smear too long
static const uint32_t LIDAR_MIN_PUBLISH_GAPMS = 80;
static const int LIDAR_MIN_VALID_BINS         = 180;
static const int LIDAR_MIN_TOUCHED_BINS       = 220;
static const int LIDAR_CHANGE_EPS_MM          = 60;   // ignore tiny jitter

uint16_t shadow_mm[LIDAR_BIN_COUNT];
bool touched[LIDAR_BIN_COUNT];

uint32_t scanWindowStartMs = 0;
uint32_t lastLidarPublishMs = 0;
uint32_t lastLidarChangeMs = 0;

int touchedCount = 0;
int validCountNow = 0;

// ------ Helpers ------

static const char* lidarResultName(YDLidarG2_Uno::Result r){
  switch (r){
    case YDLidarG2_Uno::OK: return "OK";
    case YDLidarG2_Uno::TIMEOUT: return "TIMEOUT";
    case YDLidarG2_Uno::BAD_HEADER: return "BAD_HEADER";
    case YDLidarG2_Uno::BAD_RESPONSE: return "BAD_RESPONSE";
    case YDLidarG2_Uno::NOT_OPEN: return "NOT_OPEN";
    default: return "UNKNOWN";
  }
}

static int clampInt(int x, int low, int high){
  if(x < low) return low;
  if(x > high) return high;
  return x;
}

static bool isValidRange(uint16_t d){
  return d >= LIDAR_MIN_MM && d <= LIDAR_MAX_MM;
}

// ------ Connection helpers ------

void connectWiFi(){
  if(WiFi.status() == WL_CONNECTED) return;

  Serial.print("\nConnecting WiFi... ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  while(WiFi.localIP() == IPAddress(0,0,0,0)){
    delay(200);
  }

  Serial.print(" Connected!");
  Serial.print("\nIP: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT(){
  if(mqtt.connected()) return;

  subscribed = false;

  Serial.print("Connecting MQTT... ");
  while(!mqtt.connect(MQTT_HOST, MQTT_PORT)) {
    Serial.print("MQTT connect failed: ");
    Serial.println(mqtt.connectError());
    delay(1000);
  }
  Serial.println("Connected!");
}

void subscribeMQTT(){
  if(!mqtt.connected()) return;
  if(subscribed) return;

  Serial.println("Subscribing to:");
  Serial.println(topic_twist);

  mqtt.subscribe(topic_twist, 1);
  subscribed = true;

  Serial.println("Subscription success!");
}

// ------ Motor helpers ------

void setMotors(int left, int right){
  targetL = clampInt(left, -255, 255);
  targetR = clampInt(right, -255, 255);

  motorL.setSpeed(targetL);
  motorR.setSpeed(targetR);
}

void stopMotors(){
  targetL = 0;
  targetR = 0;
  setMotors(0, 0);
}

void handleTwistMessage(const char* json, size_t n){
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, json, n);
  if(err){
    Serial.print("[twist] JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }

  int seq = doc["seq"] | -1;
  const char* command_id = doc["command_id"] | "";
  float v = doc["v"] | 0.0f;
  float w = doc["w"] | 0.0f;
  int ttl = doc["ttl_ms"] | 0;

  if(strlen(command_id) > 0 && lastCommandId == String(command_id)){
    Serial.println("Duplicate ACK sent");

    mqtt.beginMessage(topic_ack);
    mqtt.print("{\"command_id\":\"");
    mqtt.print(command_id);
    mqtt.print("\",\"status\":\"received\"}");
    mqtt.endMessage();
    return;
  }

  if(strlen(command_id) > 0){
    lastCommandId = String(command_id);
    Serial.println("ACK sent");

    mqtt.beginMessage(topic_ack);
    mqtt.print("{\"command_id\":\"");
    mqtt.print(command_id);
    mqtt.print("\",\"seq\":");
    mqtt.print(seq);
    mqtt.print(",\"status\":\"received\"}");
    mqtt.endMessage();
  }

  int base = (int)roundf(K_V * v);
  int turn = (int)roundf(K_W * w);

  int left = base - turn;
  int right = base + turn;

  setMotors(left, right);

  uint32_t now = millis();
  if(ttl <= 0){
    cmdExpiresAtMs = now;
  } else {
    cmdExpiresAtMs = now + (uint32_t)ttl;
  }

  Serial.print("[twist] seq="); Serial.print(seq);
  Serial.print("; v="); Serial.print(v, 3);
  Serial.print("; w="); Serial.print(w, 3);
  Serial.print("; ttl_ms="); Serial.print(ttl);
  Serial.print("; L="); Serial.print(targetL);
  Serial.print("; R="); Serial.println(targetR);
}

// ------ Lidar helpers ------

void printLidarInfo(){
  Serial.println("=== LIDAR INFO ===");

  auto r = lidar.getHealth(health, 800);
  Serial.print("getHealth -> ");
  Serial.println(lidarResultName(r));
  if(r == YDLidarG2_Uno::OK){
    Serial.print("Health status: ");
    Serial.println(health.status);
    Serial.print("Error code: ");
    Serial.println(health.error_code);
  }

  r = lidar.getDeviceInfo(info, 800);
  Serial.print("getDeviceInfo -> ");
  Serial.println(lidarResultName(r));
  if(r == YDLidarG2_Uno::OK){
    Serial.print("Model: ");
    Serial.println(info.model);

    Serial.print("Firmware: ");
    Serial.print((info.firmware_version >> 8) & 0xFF);
    Serial.print(".");
    Serial.println(info.firmware_version & 0xFF);

    Serial.print("Hardware: ");
    Serial.println(info.hardware_version);

    Serial.print("Serial: ");
    for(int i = 0; i < 16; ++i){
      if(info.serialnum[i] < 16) Serial.print('0');
      Serial.print(info.serialnum[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }

  Serial.println();
}

bool startLidar(){
  lidar.begin(Serial1);
  delay(100);

  printLidarInfo();

  auto r = lidar.startScan(1200);
  Serial.print("startScan -> ");
  Serial.println(lidarResultName(r));

  if(r == YDLidarG2_Uno::OK){
    Serial.println("Scanning started.");
    Serial.println();
    return true;
  }

  Serial.println("Failed to start scanning.");
  return false;
}

void initShadowFromCurrent(){
  for(int i = 0; i < LIDAR_BIN_COUNT; ++i){
    shadow_mm[i] = lidar.distance_mm[i];
    touched[i] = false;
  }
  touchedCount = 0;
  validCountNow = 0;
  scanWindowStartMs = millis();
  lastLidarChangeMs = scanWindowStartMs;
}

void updateScanAccumulator(uint32_t nowMs){
  int validNow = 0;
  bool anyChange = false;

  for(int angle = 0; angle < LIDAR_BIN_COUNT; ++angle){
    uint16_t cur = lidar.distance_mm[angle];

    if(isValidRange(cur)){
      validNow++;
    }

    uint16_t old = shadow_mm[angle];

    bool oldValid = isValidRange(old);
    bool curValid = isValidRange(cur);

    bool changed = false;

    if(oldValid != curValid){
      changed = true;
    } else if(curValid && oldValid){
      int diff = (int)cur - (int)old;
      if(diff < 0) diff = -diff;
      if(diff >= LIDAR_CHANGE_EPS_MM){
        changed = true;
      }
    }

    if(changed){
      shadow_mm[angle] = cur;
      anyChange = true;

      if(!touched[angle]){
        touched[angle] = true;
        touchedCount++;
      }
    }
  }

  validCountNow = validNow;

  if(anyChange){
    lastLidarChangeMs = nowMs;
  }
}

bool shouldPublishScan(uint32_t nowMs){
  if(!mqtt.connected()) return false;

  uint32_t ageMs = nowMs - scanWindowStartMs;
  uint32_t sinceLastPub = nowMs - lastLidarPublishMs;

  if(ageMs < LIDAR_MIN_SCAN_MS) return false;
  if(sinceLastPub < LIDAR_MIN_PUBLISH_GAPMS) return false;
  if(validCountNow < LIDAR_MIN_VALID_BINS) return false;
  if(touchedCount < LIDAR_MIN_TOUCHED_BINS) return false;

  return true;
}

void publishFreshScan(uint32_t nowMs){
  if(!mqtt.connected()) return;

  String payload;
  payload.reserve(2600);

  payload += "L360,seq=";
  payload += String(lidarSeq++);
  payload += ",ts=";
  payload += String(nowMs);
  payload += ",valid=";
  payload += String(validCountNow);
  payload += ",touched=";
  payload += String(touchedCount);
  payload += ";";

  for(int angle = 0; angle < LIDAR_BIN_COUNT; ++angle){
    uint16_t d = lidar.distance_mm[angle];

    if(!isValidRange(d)){
      d = 0;
    }

    payload += String(d);
    if(angle < LIDAR_BIN_COUNT - 1){
      payload += ",";
    }
  }

  mqtt.beginMessage(topic_lidar, (unsigned long)payload.length());
  mqtt.print(payload);
  mqtt.endMessage();

  lastLidarPublishMs = nowMs;

  Serial.print("[lidar] published seq=");
  Serial.print(lidarSeq - 1);
  Serial.print(" valid=");
  Serial.print(validCountNow);
  Serial.print(" touched=");
  Serial.print(touchedCount);
  Serial.print(" age_ms=");
  Serial.print(nowMs - scanWindowStartMs);
  Serial.print(" len=");
  Serial.println(payload.length());

  resetScanAccumulator(nowMs);
}

void serviceLidar(uint32_t nowMs){
  if(!scanning) return;

  auto r = lidar.pollScanPacket(20);

  if(r == YDLidarG2_Uno::OK){
    if(lidar.scanReady()){
      // publish lidar.distance_mm[] here
      publishCompletedScan(nowMs);
      lidar.consumeScan();
    }
  }
  else if(r != YDLidarG2_Uno::TIMEOUT){
    Serial.print("[lidar] poll error: ");
    Serial.println((int)r);
  }
}



// ------ Setup ------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Robot v1.0 boot");

  mqtt.setId(clientId);
  mqtt.setKeepAliveInterval(30 * 1000);

  stopMotors();

  connectWiFi();
  connectMQTT();
  subscribeMQTT();

  scanning = startLidar();

  initShadowFromCurrent();
  resetScanAccumulator(millis());
}

// ------ Loop ------

void loop(){
  static uint32_t lastConnectCheck = 0;
  static uint32_t lastPoll = 0;
  static uint32_t lastTelemetry = 0;

  uint32_t now = millis();

  // reconnect check
  if(now - lastConnectCheck >= 5000){
    connectWiFi();
    connectMQTT();
    subscribeMQTT();
    lastConnectCheck = now;
  }

  // mqtt polling
  now = millis();
  if(now - lastPoll >= 10){
    mqtt.poll();
    lastPoll = now;
  }

  // message drain
  int messageSize;
  while((messageSize = mqtt.parseMessage()) > 0){
    String topic = mqtt.messageTopic();

    const int MAX = 512;
    int n = clampInt(messageSize, 0, MAX - 1);
    char buf[MAX];
    int i = 0;

    while(mqtt.available() && i < n){
      buf[i++] = (char)mqtt.read();
    }
    buf[i] = '\0';

    while(mqtt.available()) mqtt.read();

    if(topic == topic_twist){
      handleTwistMessage(buf, i);
    }
  }

  // stop if command expired
  now = millis();
  if(cmdExpiresAtMs != 0 && (int32_t)(now - cmdExpiresAtMs) >= 0){
    cmdExpiresAtMs = 0;
    stopMotors();
    Serial.println("[twist] TTL expired -> STOP");
  }

  // lidar polling
  serviceLidar(millis());

  now = millis();
  if(now - lastTelemetry >= 5000){
    lastTelemetry = now;
    Serial.print("WiFi=");
    Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "DOWN");
    Serial.print(" MQTT=");
    Serial.print(mqtt.connected() ? "OK" : "DOWN");
    Serial.print(" cmdTTL=");
    Serial.print(cmdExpiresAtMs ? (int32_t)(cmdExpiresAtMs - now) : 0);
    Serial.print(" scanning=");
    Serial.print(scanning ? "YES" : "NO");
    Serial.print(" valid=");
    Serial.print(validCountNow);
    Serial.print(" touched=");
    Serial.println(touchedCount);
  }
}
