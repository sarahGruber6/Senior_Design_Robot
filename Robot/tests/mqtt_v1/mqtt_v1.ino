#include <WiFiS3.h>

const char ssid[] = "name";
const char pass[] = "pass";

// Flask server IP on LAN
const char host[] = "192.168.4.25";   // <-- Flask machine IP
const int port = 5000;

WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print("\nConnecting WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  while(WiFi.localIP() == IPAddress(0,0,0,0)){
    delay(500);
  }
  Serial.println("\nWiFi connected!");
  delay(2000);
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

String httpGet(const char* path) {
  if (!client.connect(host, port)) {
    Serial.println("HTTP connect failed");
    return "";
  }

  client.print(String("GET ") + path + " HTTP/1.1\r\n");
  client.print(String("Host: ") + host + "\r\n");
  client.print("Connection: close\r\n\r\n");

  // Skip headers
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }

  String body = client.readString();
  client.stop();
  return body;
}

void loop() {
  String body = httpGet("/api/robot/active_or_next");
  if (body.length()) {
    Serial.println("Response body:");
    Serial.println(body);
  }

  delay(2000); // poll every 2s for now
}