#pragma once

#define MQTT_HOST ""
#define MQTT_PORT 1883

#define WIFI_SSID ""
#define WIFI_PASS ""

// motor gains — tune with drive_calibration sketches
#define K_V         350.0f
#define K_W         140.0f
#define LEFT_TRIM     1.0f
#define K_CORRECT    0.15f

// motor pins
#define LEFT_PWM_PIN   7
#define LEFT_DIR_PIN   8
#define RIGHT_PWM_PIN  5
#define RIGHT_DIR_PIN  6

// encoder pins
#define LEFT_A   38
#define LEFT_B   39
#define RIGHT_A  40
#define RIGHT_B  41

// lidar pins
#define LIDAR_RX_PIN    17
#define LIDAR_TX_PIN    18
#define LIDAR_MOTOR_PIN 16

// exclusion zones from lidar_calibration 'a' run: {center_bin, half_width}
#define POST_EXCLUSION_ZONES  {{45, 5}, {150, 6}, {238, 5}, {344, 5}}
#define POST_EXCLUSION_COUNT  4

// obstacle avoidance: forward bin, cone half-width, stop distance, v sign (+1 or -1)
#define OBSTACLE_FORWARD_BIN  350
#define OBSTACLE_CONE_DEG    20
#define OBSTACLE_STOP_MM    350
#define OBSTACLE_FORWARD_V_SIGN -1
