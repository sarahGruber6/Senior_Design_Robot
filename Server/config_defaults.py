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
TOPIC_LIDAR = f"robot/{ROBOT_ID}/lidar"

