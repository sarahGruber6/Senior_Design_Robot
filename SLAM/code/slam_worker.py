# Subscribe to MQTT lidar scans, parse them, update BreezySLAM,
# publish pose/status, and periodically save a map image.

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import paho.mqtt.client as mqtt
from PIL import Image

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser

from config import MQTT_HOST, MQTT_PORT, TOPIC_LIDAR, TOPIC_POSE, TOPIC_STATUS
from slam_parser import parse_l360


BROKER_HOST = MQTT_HOST
BROKER_PORT = MQTT_PORT

MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 20

SCAN_SIZE = 360
SCAN_RATE_HZ = 10
MAX_RANGE_MM = 10000

SAVE_MAP_EVERY_N_SCANS = 10
STATUS_EVERY_N_SCANS = 5

OUTPUT_DIR = Path(".")
MAP_PATH = OUTPUT_DIR / "latest_map.png"


laser = Laser(
    SCAN_SIZE,
    SCAN_RATE_HZ,
    360,
    MAX_RANGE_MM
)

slam = RMHC_SLAM(
    laser,
    MAP_SIZE_PIXELS,
    MAP_SIZE_METERS
)

mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

scans_used = 0
last_seq = -1


def save_map_png(path: Path = MAP_PATH):
    slam.getmap(mapbytes)
    arr = np.array(mapbytes, dtype=np.uint8).reshape((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
    img = Image.fromarray(arr, mode="L")
    img.save(path)


def publish_pose(client: mqtt.Client, seq: int, ts_robot_ms: int):
    x_mm, y_mm, theta_deg = slam.getpos()

    payload = {
        "seq": seq,
        "ts_robot_ms": ts_robot_ms,
        "x_mm": x_mm,
        "y_mm": y_mm,
        "theta_deg": theta_deg,
        "ts_server_ms": int(time.time() * 1000),
    }
    client.publish(TOPIC_POSE, json.dumps(payload), qos=0)


def publish_status(client: mqtt.Client):
    payload = {
        "state": "running",
        "scans_used": scans_used,
        "last_seq": last_seq,
        "ts_server_ms": int(time.time() * 1000),
    }
    client.publish(TOPIC_STATUS, json.dumps(payload), qos=0)


def handle_lidar_message(client: mqtt.Client, payload_text: str):
    global scans_used, last_seq

    parsed = parse_l360(payload_text, reverse_scan=True)

    slam.update(parsed.distances_mm)
    scans_used += 1
    last_seq = parsed.seq

    publish_pose(client, parsed.seq, parsed.ts_robot_ms)

    if scans_used % STATUS_EVERY_N_SCANS == 0:
        publish_status(client)

    if scans_used % SAVE_MAP_EVERY_N_SCANS == 0:
        save_map_png()
        print(f"[slam] map saved to {MAP_PATH}")

    x_mm, y_mm, theta_deg = slam.getpos()
    print(
        f"[slam] seq={parsed.seq} "
        f"used={scans_used} "
        f"valid={parsed.valid_bins} "
        f"pose=({x_mm:.1f}, {y_mm:.1f}, {theta_deg:.1f})"
    )


def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"[mqtt] connected, reason_code={reason_code}")
    client.subscribe(TOPIC_LIDAR)
    print(f"[mqtt] subscribed to {TOPIC_LIDAR}")


def on_message(client, userdata, msg):
    try:
        payload_text = msg.payload.decode("utf-8", errors="replace")
        handle_lidar_message(client, payload_text)
    except Exception as e:
        print(f"[slam] message error: {e}")


def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="slam_worker_r1")
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"[mqtt] connecting to {BROKER_HOST}:{BROKER_PORT}")
    client.connect(BROKER_HOST, BROKER_PORT, 60)
    client.loop_forever()


if __name__ == "__main__":
    main()