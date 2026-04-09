#!/usr/bin/env python3
from __future__ import annotations
import struct
import threading
from collections import deque
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import paho.mqtt.client as mqtt
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from config import MQTT_HOST, MQTT_PORT, TOPIC_LIDAR

MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 20
SCAN_SIZE       = 360
SCAN_RATE_HZ    = 10
MAX_RANGE_MM = 8000  

# Tune these for your physical lidar mounting:
ANGLE_OFFSET_DEG = 0     # rotate scan so index-0 = robot forward
FLIP_SCAN        = True  # True if lidar sweeps CCW (YDLidar G2 does)

MAGIC       = b"SCN3"
HEADER_FMT  = "<4sIIH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 14
BINS_FMT    = "<360H"
BINS_SIZE   = struct.calcsize(BINS_FMT)     # 720
FRAME_SIZE  = HEADER_SIZE + BINS_SIZE       # 734

NO_DETECTION_MM = 0  # single source of truth

_lock = threading.Lock()
_pending_scans: deque[tuple[int, int, list[int]]] = deque(maxlen=20)
_stats = {"received": 0, "bad": 0, "used": 0, "seq_last": -1}
_debug_done = False

def decode_frame(payload: bytes) -> tuple[int, int, list[int]] | None:
    if len(payload) != FRAME_SIZE:
        _stats["bad"] += 1
        return None

    magic, seq, ts_ms, valid_bins = struct.unpack_from(HEADER_FMT, payload, 0)
    if magic != MAGIC:
        _stats["bad"] += 1
        return None

    bins = list(struct.unpack_from(BINS_FMT, payload, HEADER_SIZE))

    # Rotate so 0-index aligns with robot forward
    if ANGLE_OFFSET_DEG:
        bins = bins[ANGLE_OFFSET_DEG:] + bins[:ANGLE_OFFSET_DEG]

    # Convert CCW → CW for BreezySLAM
    if FLIP_SCAN:
        bins = bins[::-1]

    # Use 0 for no-return — NOT max range (0 = "no data" in BreezySLAM)
    scan = [mm if 0 < mm <= MAX_RANGE_MM else NO_DETECTION_MM for mm in bins]

    _stats["received"] += 1
    _stats["seq_last"] = seq
    return seq, ts_ms, scan

def debug_polar_plot(scan: list[int]):
    """Call this once to verify scan orientation before running SLAM."""

    angles = np.deg2rad(np.arange(360))
    ranges = np.array(scan, dtype=float)
    # zero out no-data for cleaner plot
    ranges[ranges == 0] = np.nan

    fig, ax = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(6, 6))
    ax.scatter(angles, ranges, s=1, c="blue")
    ax.set_theta_zero_location("N")   # 0° = top = robot forward
    ax.set_theta_direction(-1)         # clockwise
    ax.set_title("Raw scan — forward should be at top (0°)")
    plt.savefig("scan_debug.png", dpi=150)
    plt.close()
    print("[debug] scan_debug.png written")


def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"[mqtt] connected → subscribing to {TOPIC_LIDAR}")
    client.subscribe(TOPIC_LIDAR, qos=0)


def on_message(client, userdata, msg):
    global _debug_done
    decoded = decode_frame(bytes(msg.payload))
    if decoded is None:
        return
    if not _debug_done:
        debug_polar_plot(decoded[2])
        _debug_done = True
    with _lock:
        _pending_scans.append(decoded)


def main():
    laser = Laser(
        SCAN_SIZE,        # 360
        SCAN_RATE_HZ,     # 10
        360,              # detection angle
        NO_DETECTION_MM,  # must match fill value below
        0,                # detection margin
        0.0,              # offset mm
    )
    slam = RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS)
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
        cmap="gray", vmin=0, vmax=255, origin="upper",
    )
    title = ax.set_title("waiting for scans…")
    plt.tight_layout()

    def update(_frame):
        # Drain queue, keep only the newest scan
        scan_item = None
        with _lock:
            while _pending_scans:
                scan_item = _pending_scans.popleft()
        if scan_item is None:
            return (img_handle,)

        seq, ts_ms, scan = scan_item
        slam.update(scan)
        _stats["used"] += 1

        x, y, theta = slam.getpos()
        slam.getmap(mapbytes)
        mapimg = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
            (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
        )
        img_handle.set_data(mapimg)
        title.set_text(
            f"used={_stats['used']}  rx={_stats['received']}  "
            f"bad={_stats['bad']}  seq={seq}  "
            f"x={x:.1f} y={y:.1f} θ={theta:.1f}°"
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