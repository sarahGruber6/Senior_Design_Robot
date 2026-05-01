import math
import threading
import json
import struct
import time
from collections import deque
from typing import Optional
import numpy as np
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser

import sys
from pathlib import Path

# Add SLAM code to path for slam_parser import
sys_path_add = Path(__file__).parent.parent / "SLAM" / "code"
if str(sys_path_add) not in sys.path:
    sys.path.insert(0, str(sys_path_add))

try:
    from slam_parser import parse_scan
except ImportError:
    # Fallback if slam_parser not available
    parse_scan = None

from .config import (
    TOPIC_LIDAR,
    TOPIC_ENCODER,
    TOPIC_STATUS,
    POST_EXCLUSION_ZONES,
    TICKS_PER_REV,
    WHEEL_DIAMETER_MM,
    AXLE_WIDTH_MM,
    STATIC_MAP_PATH,
    ENCODER_LEFT_SIGN,
    ENCODER_RIGHT_SIGN,
    ODOM_THETA_SIGN,
    ODOM_MAX_DELTA_TICKS,
)


class SlamService:
    def __init__(self, mqtt_bus):
        self.mqtt_bus = mqtt_bus
        self._running = False
        self._thread = None
        self._lock = threading.Lock()

        # SLAM parameters — stored so reset() can recreate an identical instance
        MAP_SIZE_PIXELS = 800
        MAP_SIZE_METERS = 20
        SCAN_SIZE = 360
        SCAN_RATE_HZ = 10
        self._laser_args = (SCAN_SIZE, SCAN_RATE_HZ, 360, 0, 0, 0.0)
        self._slam_args  = (MAP_SIZE_PIXELS, MAP_SIZE_METERS)
        # Wider search window helps RMHC recover from small encoder errors during turns.
        self._rmhc_kwargs = dict(
            random_seed=0xAB30,
            sigma_xy_mm=200,
            sigma_theta_degrees=30,
            max_search_iter=2000,
        )

        # Create laser and SLAM
        laser = Laser(*self._laser_args)
        self.slam = RMHC_SLAM(laser, *self._slam_args, **self._rmhc_kwargs)

        self.TICKS_PER_REV = TICKS_PER_REV
        self.WHEEL_DIAMETER_MM = WHEEL_DIAMETER_MM
        self.AXLE_WIDTH_MM = AXLE_WIDTH_MM
        self.ENCODER_LEFT_SIGN = ENCODER_LEFT_SIGN
        self.ENCODER_RIGHT_SIGN = ENCODER_RIGHT_SIGN
        self.ODOM_THETA_SIGN = ODOM_THETA_SIGN
        self.ODOM_MAX_DELTA_TICKS = ODOM_MAX_DELTA_TICKS
        self.last_encoder = None

        # Accumulated odometry between lidar updates
        self._pending_dxy_mm = 0.0
        self._pending_dtheta_deg = 0.0
        self._last_lidar_ts = 0.0

        # State
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.map_pixels = MAP_SIZE_PIXELS
        self.map_size_m = MAP_SIZE_METERS
        self.current_pose = {"x_mm": 0, "y_mm": 0, "theta_deg": 0}

        # Static map — saved snapshot used for planning and composite display
        self._static_map: Optional[bytearray] = None
        self._static_map_path = STATIC_MAP_PATH
        self._try_load_static_map()  # restore from disk if available

        # Latest robot status message (from robot/r1/status topic)
        self._robot_status = {"obstacle": 0, "mode": 0, "mot_l": 0, "mot_r": 0, "scans_ok": 0}

        # Statistics
        self.stats = {
            "lidar_received": 0,
            "lidar_skipped": 0,
            "encoder_received": 0,
            "encoder_baselined": False,
            "encoder_dropped": 0,
            "last_odom": {"dxy_mm": 0.0, "dtheta_deg": 0.0, "dt_ms": 0},
            "slam_updates": 0,
            "last_update_ms": 0,
        }

    def start(self):
        if self._running:
            return

        self._running = True
        self.mqtt_bus.register_handler(TOPIC_LIDAR, self._on_lidar_message)
        self.mqtt_bus.register_handler(TOPIC_ENCODER, self._on_encoder_message)
        self.mqtt_bus.register_handler(TOPIC_STATUS, self._on_status_message)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print("[SLAM] SlamService started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)
        print("[SLAM] SlamService stopped")

    def _on_lidar_message(self, payload_bytes, topic):
        try:
            if parse_scan is None:
                return

            parsed_scan = parse_scan(payload_bytes, reverse_scan=True, exclusion_zones=POST_EXCLUSION_ZONES or None)
            if parsed_scan is None:
                self.stats["lidar_skipped"] += 1
                return

            with self._lock:
                self.stats["lidar_received"] += 1
                now = time.time()
                dt = now - self._last_lidar_ts if self._last_lidar_ts else 0.0
                self._last_lidar_ts = now
                pose_change = (self._pending_dxy_mm, self._pending_dtheta_deg, dt)
                self.slam.update(parsed_scan.distances_mm, pose_change)
                self._pending_dxy_mm = 0.0
                self._pending_dtheta_deg = 0.0
                self._update_pose()
                self.stats["slam_updates"] += 1
                self.stats["last_update_ms"] = int(time.time() * 1000)

        except Exception as e:
            print(f"[SLAM] LiDAR parse error: {e}")
            self.stats["lidar_skipped"] += 1

    def _on_status_message(self, payload_bytes, topic):
        try:
            data = json.loads(payload_bytes.decode("utf-8"))
            with self._lock:
                self._robot_status = {
                    "obstacle": int(data.get("obstacle", 0)),
                    "mode":     int(data.get("mode", 0)),
                    "mot_l":    int(data.get("mot_l", 0)),
                    "mot_r":    int(data.get("mot_r", 0)),
                    "scans_ok": int(data.get("scans_ok", 0)),
                }
        except Exception:
            pass

    def _on_encoder_message(self, payload_bytes, topic):
        try:
            data = json.loads(payload_bytes.decode("utf-8"))
            left_ticks = data.get("left_ticks", 0)
            right_ticks = data.get("right_ticks", 0)
            ts_ms = data.get("ts_ms", 0)

            with self._lock:
                self.stats["encoder_received"] += 1

                if self.last_encoder is None:
                    self.last_encoder = {"left_ticks": left_ticks, "right_ticks": right_ticks, "ts_ms": ts_ms}
                    self.stats["encoder_baselined"] = True
                    return

                raw_delta_left = left_ticks - self.last_encoder["left_ticks"]
                raw_delta_right = right_ticks - self.last_encoder["right_ticks"]
                dt_ms = ts_ms - self.last_encoder["ts_ms"]
                self.last_encoder = {"left_ticks": left_ticks, "right_ticks": right_ticks, "ts_ms": ts_ms}

                if (
                    abs(raw_delta_left) > self.ODOM_MAX_DELTA_TICKS
                    or abs(raw_delta_right) > self.ODOM_MAX_DELTA_TICKS
                ):
                    self.stats["encoder_dropped"] += 1
                    return

                delta_left = raw_delta_left * self.ENCODER_LEFT_SIGN
                delta_right = raw_delta_right * self.ENCODER_RIGHT_SIGN
                mm_per_tick = (math.pi * self.WHEEL_DIAMETER_MM) / self.TICKS_PER_REV
                d_left = delta_left * mm_per_tick
                d_right = delta_right * mm_per_tick
                dxy_mm = (d_left + d_right) / 2.0
                dtheta_deg = (
                    self.ODOM_THETA_SIGN
                    * ((d_right - d_left) / self.AXLE_WIDTH_MM)
                    * (180.0 / math.pi)
                )
                self._pending_dxy_mm += dxy_mm
                self._pending_dtheta_deg += dtheta_deg
                self.stats["last_odom"] = {
                    "dxy_mm": round(dxy_mm, 2),
                    "dtheta_deg": round(dtheta_deg, 2),
                    "dt_ms": dt_ms,
                    "raw_delta_left": raw_delta_left,
                    "raw_delta_right": raw_delta_right,
                    "signed_delta_left": delta_left,
                    "signed_delta_right": delta_right,
                }

        except Exception as e:
            print(f"[SLAM] Encoder parse error: {e}")

    def _update_pose(self):
        try:
            x, y, theta_deg = self.slam.getpos()
            self.current_pose = {
                "x_mm": x,
                "y_mm": y,
                "theta_deg": theta_deg,
            }
        except Exception as e:
            print(f"[SLAM] getpos error: {e}")

    def _run(self):
        while self._running:
            time.sleep(0.1)

    def get_pose(self):
        with self._lock:
            return dict(self.current_pose)

    def save_static_map(self) -> dict:
        with self._lock:
            snapshot = bytearray(self.map_pixels * self.map_pixels)
            self.slam.getmap(snapshot)
            self._static_map = snapshot

        try:
            self._static_map_path.parent.mkdir(parents=True, exist_ok=True)
            arr = np.frombuffer(self._static_map, dtype=np.uint8).copy()
            np.save(str(self._static_map_path), arr)
            print(f"[SLAM] Static map saved → {self._static_map_path}")
            return {"ok": True}
        except Exception as e:
            print(f"[SLAM] Map save error: {e}")
            return {"ok": False, "error": str(e)}

    def clear_static_map(self) -> dict:
        with self._lock:
            self._static_map = None
        try:
            if self._static_map_path.exists():
                self._static_map_path.unlink()
            print("[SLAM] Static map cleared")
            return {"ok": True}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def _try_load_static_map(self):
        if not self._static_map_path.exists():
            return
        try:
            arr = np.load(str(self._static_map_path))
            loaded = bytearray(arr.astype(np.uint8).tobytes())
            expected = self.map_pixels * self.map_pixels
            if len(loaded) != expected:
                print(f"[SLAM] Static map size mismatch ({len(loaded)} vs {expected}) — ignoring")
                return
            with self._lock:
                self._static_map = loaded
                self.slam.setmap(self._static_map)
            print(f"[SLAM] Static map loaded from {self._static_map_path}")
        except Exception as e:
            print(f"[SLAM] Could not load static map: {e}")

    def reset(self) -> dict:
        with self._lock:
            laser = Laser(*self._laser_args)
            self.slam = RMHC_SLAM(laser, *self._slam_args, **self._rmhc_kwargs)
            if self._static_map is not None:
                self.slam.setmap(self._static_map)
            self.current_pose = {"x_mm": 0, "y_mm": 0, "theta_deg": 0}
            self._pending_dxy_mm = 0.0
            self._pending_dtheta_deg = 0.0
            self._last_lidar_ts = 0.0
            self.last_encoder = None
            self.stats["slam_updates"] = 0
            self.stats["lidar_received"] = 0
            self.stats["lidar_skipped"] = 0
            self.stats["encoder_received"] = 0
            self.stats["encoder_baselined"] = False
            self.stats["encoder_dropped"] = 0
            self.stats["last_odom"] = {"dxy_mm": 0.0, "dtheta_deg": 0.0, "dt_ms": 0}
        print("[SLAM] Map reset — awaiting first scan")
        return {"ok": True, "seeded_from_static": self._static_map is not None}

    def has_static_map(self) -> bool:
        with self._lock:
            return self._static_map is not None

    def get_planning_map(self) -> bytearray:
        with self._lock:
            if self._static_map is not None:
                return bytearray(self._static_map)
            live = bytearray(self.map_pixels * self.map_pixels)
            self.slam.getmap(live)
            return live

    def get_map(self):
        with self._lock:
            live = bytearray(self.map_pixels * self.map_pixels)
            self.slam.getmap(live)

            if self._static_map is None:
                return live

            # Vectorised composite — runs in <1 ms on 800×800
            s = np.frombuffer(self._static_map, dtype=np.uint8)
            c = np.frombuffer(live, dtype=np.uint8)

            result = c.copy()
            static_wall = s < 50
            dynamic_hit = (~static_wall) & (c < 50)

            result[static_wall] = s[static_wall]   # keep static walls
            result[dynamic_hit] = 175               # mark transient obstacles

            return bytearray(result.tobytes())

    def get_stats(self):
        with self._lock:
            return dict(self.stats)

    def is_ready(self):
        return self.stats["slam_updates"] > 0

    def get_robot_status(self) -> dict:
        with self._lock:
            return dict(self._robot_status)
