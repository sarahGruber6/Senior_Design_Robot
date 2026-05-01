#pragma once

#define MQTT_HOST ""
#define MQTT_PORT 1883

#define WIFI_SSID ""
#define WIFI_PASS ""

// Motor control gains
#define K_V         350.0f  // PWM per m/s         — tune with drive_calibration 'f' run
#define K_W         140.0f  // PWM per rad/s        — tune with drive_calibration 's' run
#define LEFT_TRIM     1.0f  // toe correction
#define K_CORRECT    0.15f  // drift correction gain (0 to disable)

// Motor pins — ESP32-S3
#define LEFT_PWM_PIN   7
#define LEFT_DIR_PIN   8
#define RIGHT_PWM_PIN  5
#define RIGHT_DIR_PIN  6

// Encoder pins — ESP32-S3
#define LEFT_A   38
#define LEFT_B   39
#define RIGHT_A  40
#define RIGHT_B  41

// Lidar UART pins — ESP32-S3 Serial1
#define LIDAR_RX_PIN    17
#define LIDAR_TX_PIN    18
#define LIDAR_MOTOR_PIN 16

// Post exclusion zones: {center_bin, half_width_bins} — zero these bins before publishing
// Copy values from lidar_calibration 'a' run output.
#define POST_EXCLUSION_ZONES  {{45, 5}, {150, 6}, {238, 5}, {344, 5}}
#define POST_EXCLUSION_COUNT  4
