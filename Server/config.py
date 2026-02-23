import os
from pathlib import Path

BROKER_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
BROKER_PORT = int(os.getenv("MQTT_PORT", "1883"))
ROBOT_ID = os.getenv("ROBOT_ID", "r1")

BASE_DIR = Path(__file__).parent
DB_PATH = BASE_DIR / "jobs_db" / "jobs.db"
ARCHIVE_DIR = BASE_DIR / "jobs_db" / "archive" 

TOPIC_JOB = f"robot/{ROBOT_ID}/cmd/job"
TOPIC_DONE = f"robot/{ROBOT_ID}/evt/done"
TOPIC_TELEMETRY = f"robot/{ROBOT_ID}/telemetry"
