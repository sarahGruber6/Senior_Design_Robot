import json
from datetime import datetime, timezone
import paho.mqtt.client as mqtt
import os
import socket
import time
import threading
import uuid

from .config import MQTT_HOST, MQTT_PORT, TOPIC_JOB, TOPIC_TWIST, TOPIC_GOAL, TOPIC_STOP, TOPIC_DONE, TOPIC_ACK, TOPIC_TELEMETRY, TOPIC_LIDAR
from .db import mark_done

STATE = {
    "last_published_job": None,
    "last_done": None,
    "last_ack": None,
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
        self._pending_acks = {}
        self._topic_handlers = {}  # {topic: callback(payload, msg.topic)}
        self._seq_lock = threading.Lock()
        self._recent_dedupe = {}

    def start(self):
        self.client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
        self.client.loop_start()

    def publish_job(self, payload: dict):
        print("[MQTT] publish_job called, connected=", self.client.is_connected())
        msg = json.dumps(payload, separators=(",", ":"))
        self.client.publish(TOPIC_JOB, msg, qos=1, retain=True)
        STATE["last_published_job"] = payload

    def publish_twist(self, v: float, w: float, ttl_ms: int, mode: str, command_id: str = None):
        with self._seq_lock:
            self._seq += 1
            seq = self._seq
        if command_id is None:
            command_id = f"{mode}-{seq}-{uuid.uuid4().hex[:6]}"
        payload = {
            "mode": mode,
            "seq": seq,
            "command_id": command_id,
            "v": v,
            "w": w,
            "ttl_ms": ttl_ms,
        }
        msg = json.dumps(payload, separators=(",",":"))
        self.client.publish(TOPIC_TWIST, msg, qos=1, retain=False)
        return payload
    
    def publish_twist_and_wait_ack(
        self,
        v: float,
        w: float,
        ttl_ms: int,
        mode: str,
        ack_timeout_s: float = 1.5,
        retries: int = 2,
        dedupe_key: str = None,
        dedupe_window_s: float = 0.75,
    ):
        now = time.monotonic()
        with self._seq_lock:
            for key, entry in list(self._recent_dedupe.items()):
                if now - entry["at"] > 10.0:
                    self._recent_dedupe.pop(key, None)

            entry = self._recent_dedupe.get(dedupe_key) if dedupe_key else None
            if entry and now - entry["at"] <= dedupe_window_s:
                seq = entry["seq"]
                command_id = entry["command_id"]
                entry["at"] = now
            else:
                self._seq += 1
                seq = self._seq
                command_id = f"twist-{seq}-{uuid.uuid4().hex[:6]}"
                if dedupe_key:
                    self._recent_dedupe[dedupe_key] = {
                        "seq": seq,
                        "command_id": command_id,
                        "at": now,
                    }

        payload = {
            "mode": mode,
            "seq": seq,
            "command_id": command_id,
            "v": v,
            "w": w,
            "ttl_ms": ttl_ms,
        }

        msg = json.dumps(payload, separators=(",",":"))

        for attempt in range(retries + 1):
            self._pending_acks[command_id] = None
            self.client.publish(TOPIC_TWIST, msg, qos=1, retain=False)

            deadline = time.time() + ack_timeout_s
            while time.time() < deadline:
                ack = self._pending_acks.get(command_id)
                if ack is not None:
                    del self._pending_acks[command_id]
                    return {
                        "ok": True,
                        "attempt": attempt + 1,
                        "sent": payload,
                        "ack": ack,
                    }
                time.sleep(0.01)

        self._pending_acks.pop(command_id, None)
        return {
            "ok": False,
            "error": "ack_timeout",
            "sent": payload,
            "retries": retries,
        }
    
    def publish_done(self, job_id: str, clear_job: bool = True):
        job_id = (job_id or "").strip()
        if not job_id:
            raise ValueError("publish_done requires non-empty job_id")

        payload = {"job_id": job_id, "at": now_iso(), "source": "server"}
        msg = json.dumps(payload, separators=(",", ":"))

        info = self.client.publish(TOPIC_DONE, msg, qos=1, retain=False)

        STATE["last_done"] = {"job_id": job_id, "raw": msg, "at": now_iso()}
        if clear_job:
            self.clear_retained_job()

        return{
            "job_id": job_id,
            "published": True,
            "rc": getattr(info, "rc", None),
            "cleared_job": clear_job,
        }

    def clear_retained_job(self):
        self.client.publish(TOPIC_JOB, payload=b"", qos=1, retain=True)

    def register_handler(self, topic: str, callback):
        self._topic_handlers[topic] = callback
        if self.client.is_connected():
            self.client.subscribe(topic, qos=1)
        print(f"[MQTT] registered handler for {topic}")

    def on_connect(self, client, userdata, flags, rc):
        print("[MQTT] subscribing to:")
        print("  DONE:", TOPIC_DONE)
        print("  TELEMETRY:", TOPIC_TELEMETRY)
        print("  ACK:", TOPIC_ACK)

        topics = [(TOPIC_DONE, 1), (TOPIC_ACK, 1), (TOPIC_TELEMETRY, 0)]
        for topic in self._topic_handlers:
            print(f"  {topic} (handler)")
            topics.append((topic, 1))

        if rc == 0:
            print(f"[MQTT] Connected successfully rc={rc}")
            client.subscribe(topics)
        else:
            print(f"[MQTT] Connection failed rc={rc}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload_raw = msg.payload.decode("utf-8", errors="replace").strip()

        if topic in self._topic_handlers:
            try:
                self._topic_handlers[topic](msg.payload, topic)
            except Exception as e:
                print(f"[MQTT] handler error for {topic}: {e}")
            return

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

        elif topic == TOPIC_ACK:
            try:
                data = json.loads(payload_raw) if payload_raw else{}
            except json.JSONDecodeError:
                data = {"raw": payload_raw}

            cmd_id = data.get("command_id")
            if cmd_id and cmd_id in self._pending_acks:
                self._pending_acks[cmd_id] = data

            STATE["last_ack"] = {"data": data, "at": now_iso()}

        elif topic == TOPIC_TELEMETRY:
            STATE["last_telemetry"] = {"raw": payload_raw, "at": now_iso()}
