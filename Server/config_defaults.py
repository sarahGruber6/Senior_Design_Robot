import os
from pathlib import Path

MQTT_HOST = ""
MQTT_PORT = ""
ROBOT_ID = "r1"
WIFI_SSID = ""
WIFI_PASS = ""

BASE_DIR = Path(__file__).parent
DB_PATH = BASE_DIR / "jobs_db" / "jobs.db"
ARCHIVE_DIR = BASE_DIR / "jobs_db" / "archive"
STATIC_MAP_PATH = BASE_DIR / "jobs_db" / "static_map.npy"

TOPIC_JOB      = f"robot/{ROBOT_ID}/cmd/job"
TOPIC_TWIST    = f"robot/{ROBOT_ID}/cmd/twist"
TOPIC_GOAL     = f"robot/{ROBOT_ID}/cmd/goal"
TOPIC_STOP     = f"robot/{ROBOT_ID}/cmd/stop"
TOPIC_DONE     = f"robot/{ROBOT_ID}/evt/done"
TOPIC_ACK      = f"robot/{ROBOT_ID}/evt/ack"
TOPIC_TELEMETRY = f"robot/{ROBOT_ID}/telemetry"
TOPIC_LIDAR    = f"robot/{ROBOT_ID}/lidar"
TOPIC_ENCODER  = f"robot/{ROBOT_ID}/encoder"
TOPIC_STATUS   = f"robot/{ROBOT_ID}/status"

POST_EXCLUSION_ZONES = []

ROBOT_RADIUS_MM = 350

TICKS_PER_REV     = 760.0
WHEEL_DIAMETER_MM = 96.0
AXLE_WIDTH_MM     = 480.0

ENCODER_LEFT_SIGN  = 1
ENCODER_RIGHT_SIGN = 1

ODOM_THETA_SIGN = 1

ODOM_MAX_DELTA_TICKS = 5000

NAMED_LOCATIONS: dict = {}
