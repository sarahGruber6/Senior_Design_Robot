#include <WiFiS3.h>
#include <ArduinoMqttClient.h>

//const char ssid[] = "utexas-iot";
//const char pass[] = "pass";
const char ssid[] = "ItHurtsWhenIP";
const char pass[] = "pass";
const byte mac[] = {0x10, 0x51, 0xDB, 0x37, 0x4F, 0x14};  // MAC address for utexas iot

const char broker[] = "broker";
const int port = port;

const char topic_job[]  = "robot/r1/cmd/job";
const char topic_done[] = "robot/r1/evt/done";

WiFiClient net;
MqttClient mqtt(net);

char clientId[] = "Arduino_R1";
bool subscribed = false;

// --------- helper functions ---------

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

// --------- setup ---------

void setup() {
  Serial.begin(115200);
  delay(1000);

  mqtt.setId(clientId);
  mqtt.setKeepAliveInterval(30*1000); // 30s

  connectWiFi();
  connectMQTT();
  subscribeMQTT();
}

// --------- main loop ---------

int flag = 1;

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

  // loop gap check
  static uint32_t lastLoopMs = 0;
  static uint32_t worstGap = 0;
  static uint32_t lastGapReport = 0;

  now = millis();
  uint32_t gap = (lastLoopMs == 0) ? 0 : (now - lastLoopMs);
  lastLoopMs = now;
  if (gap > worstGap) worstGap = gap;

  // message drain
  static uint32_t rxCount = 0;
  int messageSize;
  while ((messageSize = mqtt.parseMessage()) > 0) {
    while (mqtt.available()){
      mqtt.read(); // discard for now
    } 
    rxCount++;
  }

  // report
  if (now - lastGapReport >= 1000) {
    /* // just testing if done publishing works
    if(flag){
      mqtt.beginMessage(topic_done);
      mqtt.print("{\"job_id\":\"J002\"}");
      mqtt.endMessage();
      flag = 0;
    }
    */

    lastGapReport = now;
    Serial.print("rx/s=");
    Serial.print(rxCount);
    Serial.print("  worstLoopGapMs=");
    Serial.println(worstGap);
    rxCount = 0;
    worstGap = 0;
  }
}