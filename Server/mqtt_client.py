import json
from datetime import datetime, timezone
import paho.mqtt.client as mqtt
import os
import socket
import time

from .config import MQTT_HOST, MQTT_PORT, TOPIC_JOB, TOPIC_TWIST, TOPIC_GOAL, TOPIC_STOP, TOPIC_DONE, TOPIC_TELEMETRY, TOPIC_LIDAR
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
        self._seq = 0

        self.client = mqtt.Client(client_id="job_publisher_api")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def start(self):
        self.client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
        self.client.loop_start()

    def publish_job(self, payload: dict):
        print("[MQTT] publish_job called, connected=", self.client.is_connected())
        msg = json.dumps(payload, separators=(",", ":"))
        self.client.publish(TOPIC_JOB, msg, qos=1, retain=True)
        STATE["last_published_job"] = payload

    def publish_twist(self, v: float, w: float, ttl_ms: int, mode: str):
        self._seq += 1
        payload = {"mode": mode, "seq": self._seq, "v": v, "w": w, "ttl_ms": ttl_ms}
        msg = json.dumps(payload, separators=(",",":"))
        self.client.publish(TOPIC_TWIST, msg, qos=1, retain=False)
        return payload

    def clear_retained_job(self):
        self.client.publish(TOPIC_JOB, payload=b"", qos=1, retain=True)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"[MQTT] Connected successfully rc={rc}")
            client.subscribe([(TOPIC_DONE, 1), (TOPIC_TELEMETRY, 0)])
        else:
            print(f"[MQTT] Connection failed rc={rc}")

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
