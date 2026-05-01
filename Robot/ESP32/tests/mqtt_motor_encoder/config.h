#pragma once

#define MQTT_HOST ""
#define MQTT_PORT 1883

#define WIFI_SSID ""
#define WIFI_PASS ""

// Motor control gains — tune with drive_calibration
#define K_V         350.0f  // PWM per m/s
#define K_W         140.0f  // PWM per rad/s
#define LEFT_TRIM     1.0f  // toe correction
#define K_CORRECT    0.15f  // drift correction gain (0 to disable)

// Motor pins — ESP32-S3
#define LEFT_PWM_PIN   5
#define LEFT_DIR_PIN   6
#define RIGHT_PWM_PIN  7
#define RIGHT_DIR_PIN  8

// Encoder pins — ESP32-S3
#define LEFT_A   9
#define LEFT_B   10
#define RIGHT_A  11
#define RIGHT_B  12
