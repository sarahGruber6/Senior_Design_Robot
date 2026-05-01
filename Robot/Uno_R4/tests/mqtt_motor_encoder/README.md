# MQTT Motor Encoder Test

This Arduino sketch provides MQTT-controlled motor operation with encoder feedback and closed-loop drift correction, without any LiDAR functionality.

## Features

- **MQTT Motor Control**: Receives twist commands via MQTT and controls motors
- **Encoder Feedback**: Reads quadrature encoders at 1kHz and publishes data
- **Closed-Loop Correction**: Automatically corrects for wheel drift during straight driving
- **Motor Timeout**: Safely stops motors if no commands received within timeout period
- **Status Telemetry**: Publishes robot status and diagnostics

## MQTT Topics

### Commands
- `robot/r1/cmd/twist` - Motor velocity commands
- `robot/r1/cmd/mode` - Change operating mode
- `robot/r1/cmd/test` - Test actions

### Data
- `robot/r1/encoder` - Encoder tick counts
- `robot/r1/telemetry` - Robot status
- `robot/r1/diagnostics` - System diagnostics
- `robot/r1/evt/ack` - Command acknowledgments

## Operating Modes

- **MOTOR_MANUAL**: Direct motor control from MQTT commands
- **MOTOR_CLOSED_LOOP**: Motor control with encoder-based drift correction
- **DIAGNOSTIC**: Motors disabled, only telemetry publishing

## Configuration

Edit `config.h` to set:
- WiFi credentials
- MQTT broker details
- Motor calibration constants (K_V, K_W, LEFT_TRIM, K_CORRECT)

## Usage

1. Configure WiFi and MQTT settings in `config.h`
2. Upload sketch to Arduino UNO R4 WiFi
3. Send motor commands via MQTT:

```bash
# Move forward
mosquitto_pub -h broker -t "robot/r1/cmd/twist" -m '{"v": 0.2, "w": 0.0}'

# Turn left
mosquitto_pub -h broker -t "robot/r1/cmd/twist" -m '{"v": 0.1, "w": 0.5}'

# Switch to closed-loop mode
mosquitto_pub -h broker -t "robot/r1/cmd/mode" -m "MOTOR_CLOSED_LOOP"
```

## Hardware Connections

- Left motor: PWM pin 9, DIR pin 8
- Right motor: PWM pin 11, DIR pin 13
- Left encoder: A=7, B=4
- Right encoder: A=5, B=6

## Similar to

This sketch is equivalent to `full_integration_test.ino` but without LiDAR scanning and processing. It's like `encoder_calibration.ino` but with MQTT motor control added.