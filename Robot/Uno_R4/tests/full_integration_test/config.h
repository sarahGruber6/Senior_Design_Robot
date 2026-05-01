#pragma once

#define MQTT_HOST ""
#define MQTT_PORT ""

#define WIFI_SSID ""
#define WIFI_PASS ""

#define K_V         350.0f  // PWM per m/s         — tune with drive_calibration 'f' run
#define K_W         140.0f  // PWM per rad/s        — tune with drive_calibration 's' run
#define LEFT_TRIM     1.0f  // toe correction        — copy from drive_calibration/config.h
#define K_CORRECT    0.15f  // drift correction gain — 0 to disable closed-loop

// Post exclusion zones: {center_bin, half_width_bins} pairs.
// Bins in each range are zeroed before publishing so posts don't pollute the SLAM map.
// Copy values from lidar_calibration 'a' run output.
#define POST_EXCLUSION_ZONES  {{45, 5}, {150, 6}, {238, 5}, {344, 5}}
#define POST_EXCLUSION_COUNT  4