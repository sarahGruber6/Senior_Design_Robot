import json
from datetime import datetime, timezone
import paho.mqtt.client as mqtt

from .config import BROKER_HOST, BROKER_PORT, TOPIC_JOB, TOPIC_DONE, TOPIC_TELEMETRY
from .db import mark_done

STATE = {
    "last_published_job": None,
    "last_done": None,
    "last_telemetry": None,
}

def now_iso():
    return datetime.now(timezone.utc).isoformat()

class MqttBus:
    def __init__(self):
        self.client = mqtt.Client(client_id="job_publisher_api")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def start(self):
        self.client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
        self.client.loop_start()

    def publish_job(self, payload: dict):
        msg = json.dumps(payload, separators=(",", ":"))
        self.client.publish(TOPIC_JOB, msg, qos=1, retain=True)
        STATE["last_published_job"] = payload

    def clear_retained_job(self):
        self.client.publish(TOPIC_JOB, payload=b"", qos=1, retain=True)

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe([(TOPIC_DONE, 1), (TOPIC_TELEMETRY, 0)])
        print(f"[MQTT] Connected rc={rc}. Subscribed to {TOPIC_DONE} and {TOPIC_TELEMETRY}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload_raw = msg.payload.decode("utf-8", errors="replace").strip()

        if topic == TOPIC_DONE:
            job_id = None
            try:
                data = json.loads(payload_raw) if payload_raw else {}
                job_id = data.get("job_id")
            except json.JSONDecodeError:
                job_id = payload_raw

            if job_id:
                mark_done(job_id)

            STATE["last_done"] = {"job_id": job_id, "raw": payload_raw, "at": now_iso()}
            print(f"[MQTT] DONE received job_id={job_id}. Clearing retained job.")
            self.clear_retained_job()

        elif topic == TOPIC_TELEMETRY:
            STATE["last_telemetry"] = {"raw": payload_raw, "at": now_iso()}
