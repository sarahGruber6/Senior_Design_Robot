#!/usr/bin/env python3
from __future__ import annotations
import json
import math
import struct
import threading
from collections import deque
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import paho.mqtt.client as mqtt
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from config import (
    MQTT_HOST, MQTT_PORT,
    TOPIC_LIDAR, TOPIC_ENCODER,
    REVERSE_SCAN,
    WHEEL_DIAMETER_MM, WHEELBASE_MM, ENCODER_TICKS_PER_REV,
)
from slam_parser import parse_scan

MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 20
SCAN_SIZE       = 360
SCAN_RATE_HZ    = 10

# mm travelled per encoder tick (one side)
_MM_PER_TICK = math.pi * WHEEL_DIAMETER_MM / ENCODER_TICKS_PER_REV

# ---- shared state (MQTT thread → animation thread) ----

_scan_lock = threading.Lock()
_pending_scans: deque[tuple[int, int, list[int]]] = deque(maxlen=20)

_enc_lock   = threading.Lock()
_enc_latest: dict | None = None   # latest {"ts_ms", "left_ticks", "right_ticks"}

_stats = {"received": 0, "bad": 0, "used": 0, "seq_last": -1}
_debug_done = False


def debug_polar_plot(scan: list[int]):
    angles = np.deg2rad(np.arange(360))
    ranges = np.array(scan, dtype=float)
    ranges[ranges == 0] = np.nan
    fig, ax = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(6, 6))
    ax.scatter(angles, ranges, s=1, c="blue")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)   # CW, matching G2 native direction
    ax.set_title("Raw scan — forward should be at top (0°)")
    plt.savefig("scan_debug.png", dpi=150)
    plt.close()
    print("[debug] scan_debug.png written — check that forward is at the top")


def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"[mqtt] connected → {TOPIC_LIDAR}, {TOPIC_ENCODER}")
    client.subscribe(TOPIC_LIDAR,   qos=0)
    client.subscribe(TOPIC_ENCODER, qos=0)


def on_message(client, userdata, msg):
    global _debug_done

    if msg.topic == TOPIC_ENCODER:
        try:
            data = json.loads(msg.payload)
            with _enc_lock:
                global _enc_latest
                _enc_latest = data
        except Exception:
            pass
        return

    # lidar
    try:
        parsed = parse_scan(bytes(msg.payload), reverse_scan=REVERSE_SCAN)
    except ValueError:
        _stats["bad"] += 1
        return
    if parsed is None:
        return

    if not _debug_done:
        debug_polar_plot(parsed.distances_mm)
        _debug_done = True

    _stats["received"] += 1
    _stats["seq_last"] = parsed.seq
    with _scan_lock:
        _pending_scans.append((parsed.seq, parsed.ts_robot_ms, parsed.distances_mm))


def main():
    laser = Laser(
        SCAN_SIZE,
        SCAN_RATE_HZ,
        360,   # detection angle
        0,     # no-detection value (parse_scan interpolates, so no blanks)
        0,
        0.0,
    )
    slam = RMHC_SLAM(
        laser,
        MAP_SIZE_PIXELS,
        MAP_SIZE_METERS,
        random_seed=0xabcd,
        max_search_iter=2000,
        sigma_xy_mm=200,
        sigma_theta_degrees=30,
    )
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="slam_worker_r1")
    client.on_connect = on_connect
    client.on_message = on_message
    print(f"[mqtt] connecting to {MQTT_HOST}:{MQTT_PORT}")
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    client.loop_start()

    fig, ax = plt.subplots(figsize=(7, 7))
    img_handle = ax.imshow(
        np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype=np.uint8),
        cmap="gray", vmin=0, vmax=255,
        origin="lower",   # row 0 = bottom of image = y=0 in BreezySLAM coords
    )
    title = ax.set_title("waiting for scans…")
    plt.tight_layout()

    enc_at_last_slam: dict | None = None

    def update(_frame):
        nonlocal enc_at_last_slam
        scan_item = None
        with _scan_lock:
            while _pending_scans:
                scan_item = _pending_scans.popleft()
        if scan_item is None:
            return (img_handle,)

        seq, ts_ms, scan = scan_item

        with _enc_lock:
            curr_enc = _enc_latest  # snapshot under lock

        velocities = None
        if curr_enc is not None and enc_at_last_slam is not None:
            dL_ticks = curr_enc["left_ticks"]  - enc_at_last_slam["left_ticks"]
            dR_ticks = curr_enc["right_ticks"] - enc_at_last_slam["right_ticks"]
            dt_ms    = curr_enc["ts_ms"]       - enc_at_last_slam["ts_ms"]
            if dt_ms > 0:
                dL_mm      = dL_ticks * _MM_PER_TICK
                dR_mm      = dR_ticks * _MM_PER_TICK
                dxy_mm     = (dL_mm + dR_mm) / 2.0
                dtheta_deg = (dR_mm - dL_mm) / WHEELBASE_MM * (180.0 / math.pi)
                velocities = (dxy_mm, dtheta_deg, dt_ms / 1000.0)

        enc_at_last_slam = curr_enc

        if velocities is not None:
            slam.update(scan, velocities)
        else:
            slam.update(scan)
        _stats["used"] += 1

        x, y, theta = slam.getpos()
        slam.getmap(mapbytes)
        mapimg = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
            (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
        )
        img_handle.set_data(mapimg)
        title.set_text(
            f"used={_stats['used']}  rx={_stats['received']}  bad={_stats['bad']}  "
            f"seq={seq}  x={x:.0f} y={y:.0f} θ={theta:.1f}°"
        )
        return (img_handle,)

    ani = animation.FuncAnimation(
        fig, update, interval=50, blit=False, cache_frame_data=False
    )
    try:
        plt.show()
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
