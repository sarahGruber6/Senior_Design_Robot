// mqtt_rx_test.ino — ESP32-S3
//
// Stress-tests MQTT receive reliability while publishing large outgoing payloads.
// Two phases auto-advance on the serial monitor:
//
//   Phase A (first 30s): subscribe only — no outgoing publishes.
//     Baseline: every sent command should print an [RX] line.
//
//   Phase B (after 30s): subscribe + 734-byte scan publish every 200ms.
//     Same size/rate as full_integration_v1.  If [RX] lines drop out here
//     but not in Phase A, the publish timing is interfering with receive.
//
// How to use:
//   1. Flash to ESP32-S3.
//   2. From the server, send commands to robot/r1/cmd/twist at ~5 Hz.
//   3. Watch Serial @ 115200.  Every received message prints immediately as [RX].
//   4. After 30s, Phase B starts — check whether drop rate increases.
//   5. [STATS] lines every 5s summarise total received and worst gap.
//
// This sketch uses the correct ArduinoMqttClient receive pattern (parseMessage()
// only, no explicit poll() calls in the loop).  If this sketch also drops messages,
// the problem is deeper than the poll() call ordering bug.

#include <WiFi.h>
#include "esp_wifi.h"
#include <ArduinoMqttClient.h>
#include "config.h"
#ifdef __has_include
  #if __has_include("config_local.h")
    #include "config_local.h"
  #endif
#endif

static const byte  MAC[]  = {0x10, 0x51, 0xDB, 0x37, 0x4F, 0x14};
static WiFiClient  net;
static MqttClient  mqtt(net);

static uint32_t rxCount    = 0;
static uint32_t lastRxMs   = 0;
static uint32_t maxGapMs   = 0;
static uint32_t lastStatMs = 0;
static uint32_t lastScanMs = 0;
static bool     phaseB     = false;

// Matches full_integration_v1 SCN3 payload exactly (734 bytes).
static const size_t SCAN_BYTES = 4 + 4 + 4 + 2 + 360 * 2;
static uint8_t      scanBuf[SCAN_BYTES];

static void connectWiFi() {
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_mac(WIFI_IF_STA, MAC);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[WIFI] connecting");
    for (int i = 0; WiFi.status() != WL_CONNECTED && i < 40; i++) {
        delay(500);
        Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
        WiFi.setSleep(false);
        Serial.printf("\n[WIFI] %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WIFI] failed");
    }
}

static void connectMQTT() {
    mqtt.stop();
    mqtt.setId("ESP32S3_rx_test");
    mqtt.setKeepAliveInterval(30000);
    mqtt.setTxPayloadSize(SCAN_BYTES + 32);
    for (int i = 0; !mqtt.connect(MQTT_HOST, MQTT_PORT) && i < 5; i++) {
        Serial.println("[MQTT] retry...");
        delay(1000);
    }
    if (!mqtt.connected()) { Serial.println("[MQTT] failed"); return; }
    net.setNoDelay(true);
    mqtt.subscribe("robot/r1/cmd/twist", 1);
    mqtt.subscribe("robot/r1/cmd/mode",  1);
    Serial.println("[MQTT] connected + subscribed (QoS 1)");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== mqtt_rx_test — ESP32-S3 ===");
    scanBuf[0] = 'S'; scanBuf[1] = 'C'; scanBuf[2] = 'N'; scanBuf[3] = '3';
    connectWiFi();
    connectMQTT();
    lastStatMs = lastRxMs = lastScanMs = millis();
    Serial.println("[BOOT] Phase A: pure receive for 30s — send commands now");
}

void loop() {
    if (!mqtt.connected()) {
        delay(1000);
        if (WiFi.status() != WL_CONNECTED) connectWiFi();
        connectMQTT();
        return;
    }

    uint32_t now = millis();

    if (!phaseB && now > 30000) {
        phaseB = true;
        Serial.println("[PHASE] B — 734-byte publish every 200ms added");
    }

    // Simulated scan publish — same size/QoS/topic as full_integration_v1.
    if (phaseB && now - lastScanMs >= 200) {
        lastScanMs = now;
        uint32_t seq = now;
        scanBuf[4]  =  seq        & 0xFF;
        scanBuf[5]  = (seq >>  8) & 0xFF;
        scanBuf[6]  = (seq >> 16) & 0xFF;
        scanBuf[7]  = (seq >> 24) & 0xFF;
        mqtt.beginMessage("robot/r1/lidar", (unsigned long)SCAN_BYTES, false, 0);
        mqtt.write(scanBuf, SCAN_BYTES);
        mqtt.endMessage();
    }

    // Receive loop — correct ArduinoMqttClient pattern.
    // parseMessage() calls poll() internally only when _rxState == READ_TYPE.
    // Never call mqtt.poll() explicitly here — it causes message drops.
    int msgSize = mqtt.parseMessage();
    while (msgSize > 0) {
        uint32_t t = millis();
        String   topic = mqtt.messageTopic();
        char     payload[256];
        int      i = 0;
        while (mqtt.available() && i < 255) payload[i++] = (char)mqtt.read();
        while (mqtt.available()) mqtt.read();
        payload[i] = '\0';

        rxCount++;
        uint32_t gap = (lastRxMs > 0) ? (t - lastRxMs) : 0;
        if (gap > maxGapMs) maxGapMs = gap;
        lastRxMs = t;

        Serial.printf("[RX #%4lu] gap=%4lums  topic=%-25s  %.80s\n",
                      rxCount, gap, topic.c_str(), payload);

        msgSize = mqtt.parseMessage();  // advance — no poll() here
    }

    // Stats every 5s.
    if (now - lastStatMs >= 5000) {
        lastStatMs = now;
        Serial.printf("[STATS] phase=%c  rx=%lu  maxGap=%lums  heap=%lu\n",
                      phaseB ? 'B' : 'A', rxCount, maxGapMs, ESP.getFreeHeap());
        maxGapMs = 0;
    }
}
