#pragma once

#define MQTT_HOST ""
#define MQTT_PORT ""

#define WIFI_SSID ""
#define WIFI_PASS ""

#define K_V         350.0f  // PWM per m/s         — tune with drive_calibration 'f' run
#define K_W         140.0f  // PWM per rad/s        — tune with drive_calibration 's' run
#define LEFT_TRIM     1.0f  // toe correction        — copy from drive_calibration/config.h
#define K_CORRECT    0.15f  // drift correction gain — 0 to disable closed-loop
