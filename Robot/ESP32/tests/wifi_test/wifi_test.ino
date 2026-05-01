// wifi_test.ino — minimal ESP32-S3 WiFi connectivity test
// No dependencies. Fill in your hotspot credentials below.

#include <WiFi.h>

#define SSID "BingusPhone"
#define PASS "LetsLoveLain"

static const char* wifiStatusStr(wl_status_t s) {
    switch (s) {
        case WL_IDLE_STATUS:     return "IDLE";
        case WL_NO_SSID_AVAIL:  return "NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED:  return "SCAN_COMPLETED";
        case WL_CONNECTED:       return "CONNECTED";
        case WL_CONNECT_FAILED:  return "CONNECT_FAILED";
        case WL_CONNECTION_LOST: return "CONNECTION_LOST";
        case WL_DISCONNECTED:    return "DISCONNECTED";
        default:                 return "UNKNOWN";
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== WiFi Test ===");
    Serial.print("Target SSID: "); Serial.println(SSID);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(200);

    Serial.print("Starting WiFi.begin... ");
    WiFi.begin(SSID, PASS);
    Serial.println("done");

    uint32_t start = millis();
    wl_status_t prev = WL_IDLE_STATUS;

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        wl_status_t cur = WiFi.status();
        if (cur != prev) {
            Serial.print("  status -> "); Serial.println(wifiStatusStr(cur));
            prev = cur;
        } else {
            Serial.print(".");
        }
        if (millis() - start > 20000) {
            Serial.println("\nTIMEOUT after 20s");
            Serial.print("Final status: "); Serial.println(wifiStatusStr(cur));
            return;
        }
    }

    Serial.println("\nConnected!");
    Serial.print("  IP:   "); Serial.println(WiFi.localIP());
    Serial.print("  GW:   "); Serial.println(WiFi.gatewayIP());
    Serial.print("  RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    Serial.print("  MAC:  "); Serial.println(WiFi.macAddress());
}

void loop() {}
