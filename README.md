# Senior Design Robot

An autonomous differential-drive robot that uses a YDLidar G2 for SLAM-based navigation. The ESP32 handles hardware (LiDAR, motors, encoders) and communicates over MQTT. A Python Flask server on the host machine runs RMHC-SLAM, plans paths, and sends movement commands back to the robot.

## System Overview

```
ESP32 (Robot)                       Server (Host PC)
─────────────────────               ─────────────────────────────────
LiDAR scan (360 bins)  --MQTT->   SlamService  >  RMHC-SLAM map
Encoder ticks          --MQTT->   MotionExecutor > A* path planning
                                   MotorCommander > PD heading control
Twist command (v, w)   <-MQTT--    > publish twist
```

---

## ESP32 Firmware

### Dependencies

The firmware depends on a custom YDLidar G2 Arduino library. Arduino has no standard dependency file, so install it manually:

1. Open Arduino IDE -> **Sketch -> Include Library -> Add .ZIP Library…**
2. Download and install from: [sarahGruber6/YDLidarG2_ESP32](https://github.com/sarahGruber6/YDLidarG2_ESP32)
3. Open `Robot/ESP32/full_integration_v1/full_integration_v1.ino`

Before flashing, create `Robot/ESP32/full_integration_v1/config_local.h` with your WiFi and MQTT credentials (see [Config Variables](#config-variables) below).

---

## Backend Server

### Requirements

- Python 3.14
- The [BreezySLAM](https://github.com/simondlevy/BreezySLAM) C extension (included in this repo under `BreezySLAM/`)

### Setup

```bash
# From the repo root:
cd Server
python -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
cd ..
pip install BreezySLAM/python  # builds the C extension
```

### Run

```bash
# From the repo root, with the venv active:
python -m Server.app
```

The web UI is served at `http://localhost:5000`.

---

## Config Variables

There are two config layers: one on the server (Python), and one on the ESP32 (C++). Both use a `_defaults` file checked into git and a `_local` file you create locally for secrets and machine-specific overrides.

### Server — `Server/config_defaults.py` (and `Server/config_local.py`)

| Variable | Default | When to change |
|---|---|---|
| `MQTT_HOST` | `""` | Set in `config_local.py` to your broker address |
| `MQTT_PORT` | `1883` | Change if your broker uses a non-standard port |
| `ROBOT_ID` | `"r1"` | Change if running multiple robots; all MQTT topics use this prefix |
| `WHEEL_DIAMETER_MM` | `96.0` | Update if you swap drive wheels |
| `AXLE_WIDTH_MM` | `480.0` | Update if the wheelbase changes |
| `TICKS_PER_REV` | `760.0` | Update after encoder calibration if counts per revolution differ |
| `ENCODER_LEFT_SIGN` / `ENCODER_RIGHT_SIGN` | `1` | Flip to `-1` if an encoder counts backwards |
| `ODOM_THETA_SIGN` | `1` | Flip if odometry rotation direction is inverted |
| `ROBOT_RADIUS_MM` | `350` | Used for obstacle avoidance clearance; update to match physical robot size |
| `MAP_SIZE_PIXELS` / `MAP_SIZE_METERS` | `800` / `20` | Increase for larger environments; larger maps use more RAM |
| `sigma_xy_mm` / `sigma_theta_degrees` | `200` / `30` | SLAM position/heading uncertainty. Increase if SLAM drifts; decrease for tighter but less robust matching |

### ESP32 — `Robot/ESP32/full_integration_v1/config.h` (and `config_local.h`)

| Variable | Default | When to change |
|---|---|---|
| `WIFI_SSID` / `WIFI_PASS` | — | Set in `config_local.h` for your network |
| `MQTT_HOST` / `MQTT_PORT` | — | Set in `config_local.h` to match the server |
| `LEFT_PWM`, `RIGHT_PWM` | 7, 5 | Change if motor driver PWM wiring changes |
| `LEFT_DIR`, `RIGHT_DIR` | 8, 6 | Change if motor driver direction wiring changes |
| Left encoder `A` / `B` | 38, 39 | Change if encoder wiring changes |
| Right encoder `A` / `B` | 40, 41 | Change if encoder wiring changes |
| `LIDAR_RX` / `LIDAR_TX` / `LIDAR_MOTOR` | 17, 18, 16 | Change if LiDAR wiring changes |
| `LEFT_TRIM` | `1.0` | Scale factor to compensate if one motor runs faster; tune via `drive_calibration.ino` |
| `POST_EXCLUSION_ZONES` | `{{45,5},{150,6},{238,5},{344,5}}` | LiDAR bins blocked by the robot chassis. Recalibrate with `lidar_calibration.ino` if the LiDAR is remounted |
| `OBSTACLE_STOP_MM` | `350` | Distance at which the robot brakes for an obstacle |
| `OBSTACLE_FORWARD_BIN` | `350` | Which LiDAR bin is "straight ahead" — update if the LiDAR is remounted at a different angle |
| `OBSTACLE_FORWARD_V_SIGN` | `-1` | Flip if forward motion maps to negative velocity |
