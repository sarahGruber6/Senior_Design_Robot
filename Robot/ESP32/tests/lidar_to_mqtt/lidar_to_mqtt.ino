// lidar_to_mqtt.ino — ESP32-S3
// Streams SCN3 binary scan frames to MQTT, identical format to UNO R4 version.
// Lidar UART uses Serial1 with configurable RX/TX pins (see config.h).

#include "YDLidarG2.h"
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

static const uint16_t LIDAR_MIN_MM  = 150;
static const uint16_t LIDAR_MAX_MM  = 8000;
static const uint16_t MIN_VALID_BINS = 200;

static const uint32_t RECONNECT_PERIOD_MS = 5000;
static const uint32_t STATUS_PERIOD_MS    = 3000;

// SCN3 frame: "SCN3" | seq(u32le) | ts_ms(u32le) | valid(u16le) | 360 x dist(u16le) = 734 bytes
static const size_t PAYLOAD_SIZE = 4 + 4 + 4 + 2 + 360 * 2;

static const char TOPIC_LIDAR[] = "robot/r1/lidar";
static const char CLIENT_ID[]   = "ESP32S3_Robot1_Lidar";

const byte mac[] = {0x10, 0x51, 0xDB, 0x37, 0x4F, 0x14};

// -------------------- Globals --------------------

HardwareSerial lidarSerial(1);
YDLidarG2 lidar(lidarSerial, LIDAR_MOTOR_PIN);

WiFiClient  net;
MqttClient  mqtt(net);

bool     scanning        = false;
uint32_t publishSeq      = 0;
uint16_t distanceBins[360];

uint32_t scansCompleted   = 0;
uint32_t scansPublished   = 0;
uint32_t scansRejected    = 0;
uint32_t mqttPublishFails = 0;

uint32_t lastReconnectMs = 0;
uint32_t lastStatusMs    = 0;

// -------------------- Helpers --------------------

static inline bool isValidRangeMm(uint16_t d) {
    return d >= LIDAR_MIN_MM && d <= LIDAR_MAX_MM;
}

static void copyScanBinsFromLidar(uint16_t bins[360]) {
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
    buf[(*pos)++] = (uint8_t)(v        & 0xFF);
    buf[(*pos)++] = (uint8_t)((v >> 8)  & 0xFF);
    buf[(*pos)++] = (uint8_t)((v >> 16) & 0xFF);
    buf[(*pos)++] = (uint8_t)((v >> 24) & 0xFF);
}

// -------------------- WiFi / MQTT --------------------

void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;

    Serial.print("\nConnecting WiFi...");
  
    WiFi.mode(WIFI_STA);

    esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, &mac[0]);
    if (err == ESP_OK) {
    Serial.println("Success changing MAC Address");
  } else {
    Serial.println("Failed to change MAC Address");
  }

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
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
    lidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

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

void publishCurrentScan(uint32_t tsMs, uint16_t validBins) {
    if (!mqtt.connected()) return;

    uint8_t payload[PAYLOAD_SIZE];
    size_t  pos = 0;

    payload[pos++] = 'S';
    payload[pos++] = 'C';
    payload[pos++] = 'N';
    payload[pos++] = '3';

    writeU32LE(payload, &pos, publishSeq++);
    writeU32LE(payload, &pos, tsMs);
    writeU16LE(payload, &pos, validBins);

    for (int i = 0; i < 360; ++i) writeU16LE(payload, &pos, distanceBins[i]);

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
    uint16_t validBins = lidar.getValidBinCount();

    if (validBins < MIN_VALID_BINS) {
        scansRejected++;
        return;
    }

    publishCurrentScan(millis(), validBins);
}

// -------------------- Setup / Loop --------------------

void setup() {
    Serial.begin(115200);
    delay(1000);

    mqtt.setId(CLIENT_ID);
    mqtt.setKeepAliveInterval(30 * 1000);
    mqtt.setTxPayloadSize(PAYLOAD_SIZE + 32);

    connectWiFi();
    connectMQTT();

    Serial.println("BOOT");
    startLidar();
}

void loop() {
    mqtt.poll();

    uint32_t now = millis();

    if (now - lastReconnectMs >= RECONNECT_PERIOD_MS) {
        lastReconnectMs = now;
        if (WiFi.status() != WL_CONNECTED) connectWiFi();
        if (!mqtt.connected())             connectMQTT();
    }

    serviceLidar();

    if (now - lastStatusMs >= STATUS_PERIOD_MS) {
        lastStatusMs = now;
        Serial.print("[status] completed="); Serial.print(scansCompleted);
        Serial.print(" published=");         Serial.print(scansPublished);
        Serial.print(" rejected=");          Serial.print(scansRejected);
        Serial.print(" mqttFail=");          Serial.print(mqttPublishFails);
        Serial.print(" mqtt=");              Serial.println(mqtt.connected() ? "OK" : "DOWN");
    }
}
