import os
from pathlib import Path

# config defaults

MQTT_HOST = ""
MQTT_PORT = ""
ROBOT_ID = "r1"

WIFI_SSID = ""
WIFI_PASS = ""

BASE_DIR = Path(__file__).parent
DB_PATH = BASE_DIR / "jobs_db" / "jobs.db"
ARCHIVE_DIR = BASE_DIR / "jobs_db" / "archive" 

TOPIC_JOB = f"robot/{ROBOT_ID}/cmd/job"
TOPIC_TWIST = f"robot/{ROBOT_ID}/cmd/twist"
TOPIC_GOAL = f"robot/{ROBOT_ID}/cmd/goal"
TOPIC_STOP = f"robot/{ROBOT_ID}/cmd/stop"

TOPIC_DONE = f"robot/{ROBOT_ID}/evt/done"
TOPIC_ACK = f"robot/{ROBOT_ID}/evt/ack"

TOPIC_TELEMETRY = f"robot/{ROBOT_ID}/telemetry"
TOPIC_LIDAR     = f"robot/{ROBOT_ID}/lidar"
TOPIC_ENCODER   = f"robot/{ROBOT_ID}/encoder"
TOPIC_POSE      = f"robot/{ROBOT_ID}/slam/pose"
TOPIC_STATUS    = f"robot/{ROBOT_ID}/slam/status"

# Bins (0–359) for fixed mounting posts to exclude from scans.
# Each entry: (center_bin, half_width_bins). Fill in after a calibration run.
POST_EXCLUSION_ZONES = []  # e.g. [(45, 5), (135, 5), (225, 5), (315, 5)]

# YDLidar G2 rotates clockwise (CW). BreezySLAM expects CCW, so we reverse.
# If the map appears left-right mirrored set False in config_local.py.
REVERSE_SCAN = True

# Physical robot parameters — needed to compute odometry from encoder ticks.
# Measure these and override in config_local.py if the defaults are off.
#   WHEEL_DIAMETER_MM:      outer diameter of the drive wheel in mm
#   WHEELBASE_MM:           distance between left/right wheel contact patches in mm
#   ENCODER_TICKS_PER_REV:  quadrature ticks per wheel revolution (PPR × 4)
WHEEL_DIAMETER_MM      = 96.0
WHEELBASE_MM           = 480.0
ENCODER_TICKS_PER_REV  = 760.0    


