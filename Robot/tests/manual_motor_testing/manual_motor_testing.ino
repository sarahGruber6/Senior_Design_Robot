#include <WiFiS3.h>
#include <math.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>

#include <config.h>
#ifdef __has_include
  #if __has_include("config_local.h")
    #include "config_local.h"
  #endif
#endif

#include <CytronMotorDriver.h>

const byte mac[] = {0x10, 0x51, 0xDB, 0x37, 0x4F, 0x14};  // MAC address for utexas iot

// --------- jobs + MQTT ---------

const char topic_job[]  = "robot/r1/cmd/job";
const char topic_done[] = "robot/r1/evt/done";
const char topic_twist[] = "robot/r1/cmd/twist";

WiFiClient net;
MqttClient mqtt(net);

char clientId[] = "Arduino_R1";
bool subscribed = false;

// --------- motors ---------

CytronMD motorL(PWM_DIR, 9, 8); // PWM 2 = Pin 9, DIR 2 = Pin 10.
CytronMD motorR(PWM_DIR, 11, 13);  // PWM 1 = Pin 3, DIR 1 = Pin 4.

// twist commands to motor mapping
float K_V = 350.0f;   // PWM per m/s (needs tuning)
float K_W = 140.0f;   // PWM per rad/s (needs tuning)

// safety and command vars
volatile int lastSeq = -1;
volatile uint32_t cmdExpiresAtMs = 0;
volatile int targetL = 0;
volatile int targetR = 0;

// --------- helper functions ---------

// --- connection helpers ---
void connectWiFi(){
  if(WiFi.status() == WL_CONNECTED) return;

  Serial.print("\nConnecting WiFi... ");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  while (WiFi.localIP() == IPAddress(0,0,0,0)) delay(200);

  Serial.print("Connected!");
  Serial.print("\nIP: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT(){
  if(mqtt.connected()) return;

  subscribed = false;

  Serial.print("Connecting MQTT... ");
  while (!mqtt.connect(broker, port)) {
    Serial.print("MQTT connect failed: ");
    Serial.println(mqtt.connectError());
    delay(1000);
  }
  Serial.print("Connected!");
}

void subscribeMQTT(){
  if(!mqtt.connected()) return;
  if(subscribed) return;

  Serial.println("\nSubscribing to:");
  Serial.println(topic_job);

  mqtt.subscribe(topic_job,1);
  subscribed = true;

  Serial.println("...Success!");
}

// --- motor helpers ---
static int clamp(int x, int low, int high){
  if(x < low) return low;
  if(x > high) return high;
  return x;
}

void setMotors(int left, int right){
  targetL = clamp(left, -255, 255);
  targetR = clamp(right, -255, 255);

  motorL.setSpeed(targetL);
  motorR.setSpeed(targetR);
}

void stopMotors(){
  targetL = 0;
  targetR = 0;
  setMotors(0,0);
}

void handleTwistMessage(const char* json, size_t n){
  // json comes like this: {"mode":"manual","seq":12,"v":0.25,"w":0.0,"ttl_ms":500}
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc,json,n);
  if(err){
    Serial.print("[twist] JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }

  int seq = doc["seq"] | -1;
  float v = doc["v"] | 0.0;
  float w = doc["w"] | 0.0;
  int ttl = doc["ttl_ms"] | 0;

  // ignore repeats and out of order
  if((seq >= 0 && seq == lastSeq) || (seq >= 0 && seq != seq+1)){
    return; 
  }
  lastSeq = seq;

  // convert twist to PWMs
  int base = (int)roundf(K_V * v);
  int turn = (int)roundf(K_W * w);

  int left = base - turn;
  int right = base + turn;

  setMotors(left,right);

  // duration
  uint32_now = millis();
  if(ttl <= 0){
    cmdExpiresAtMs = now;
  }else{
    cmdExpiresAtMs = now + (uint32_t)ttl;
  }

  Serial.print("[twist] seq="); Serial.print(seq);
  Serial.print("; v="); Serial.print(v, 3);
  Serial.print("; w="); Serial.print(w, 3);
  Serial.print("; ttl_ms="); Serial.print(ttl);
  Serial.print("; L="); Serial.print(targetL);
  Serial.print("; R="); Serial.println(targetR);
}
// --------- setup ---------

void setup() {
  Serial.begin(115200);
  delay(1000);

  mqtt.setId(clientId);
  mqtt.setKeepAliveInterval(30*1000); // 30s

  stopMotors();

  connectWiFi();
  connectMQTT();
  subscribeMQTT();
}

// --------- main loop ---------

void loop(){
  // polling times
  static uint32_t lastConnectCheck = 0;
  static uint32_t lastPoll = 0;

  uint32_t now = millis();
  if(now - lastConnectCheck >= 5000){   // check and reconnect 5s
    connectWiFi();
    connectMQTT();
    subscribeMQTT();
    lastConnectCheck = now;
  }
  now = millis();
  if(now - lastPoll >= 10){   // 10ms
    mqtt.poll();
    lastPoll = now;
  }

  // loop gap check (testing)
  /*
  static uint32_t lastLoopMs = 0;
  static uint32_t worstGap = 0;
  static uint32_t lastGapReport = 0;

  now = millis();
  uint32_t gap = (lastLoopMs == 0) ? 0 : (now - lastLoopMs);
  lastLoopMs = now;
  if (gap > worstGap) worstGap = gap;
  */

  // message drain
  static uint32_t rxCount = 0;
  int messageSize;
  while((messageSize = mqtt.parseMessage()) > 0) {
    String topic = mqtt.messageTopic()

    // payload to buffer
    const int MAX = 512;
    int n = clamp(messageSize, 0, MAX - 1);
    char buf[MAX];
    int i = 0;

    while(mqtt.available() && i < n){
      buf[i++] = (char)mqtt.read();
    }
    buf[i] = '\0';

    // discard extra if payload > MAX
    while(mqtt.available()) mqtt.read();

    if(topic == topic_twist){
      handleTwistMessage(buf, i);
    }else{
      // optional debug
      // Serial.print("[mqtt] msg on "); Serial.println(topic);
    }
  }

  // stop if command expired
  now = millis();
  if(cmdExpiresAtMs != 0 && (int32_t)(now - cmdExpiresAtMs) >= 0){  // cmd expired
    cmdExpiresAtMs = 0;
    stopMotors();
    Serial.println("[twist] TTL expired -> STOP");
  }


  // telemetry
  static uint32_t lastReport = 0;
  if(now - lastReport >= 5000) {  // 5s
    lastReport = now;
    Serial.print("WiFi=");
    Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "DOWN");
    Serial.print(" MQTT=");
    Serial.print(mqtt.connected() ? "OK" : "DOWN");
    Serial.print(" cmdTTL=");
    Serial.println(cmdExpiresAtMs ? (int32_t)(cmdExpiresAtMs - now) : 0);
  }
}