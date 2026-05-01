import math
import time
from typing import Tuple, Dict


class MotorCommander:
    def __init__(self, max_v: float = 0.3, max_w: float = 0.5):
        self.max_v = max_v
        self.max_w = max_w
        self.K_p = 0.5
        self.K_d = 0.12
        self.heading_deadband_rad = math.radians(4)
        self.rotate_in_place_rad = math.radians(65)
        self.forward_turn_limit = 0.45
        self.waypoint_tolerance_mm = 250
        self._last_heading_error_rad = None
        self._last_control_time = None
        self._last_target_waypoint = None

    def reset(self):
        self._last_heading_error_rad = None
        self._last_control_time = None
        self._last_target_waypoint = None

    def compute_twist(
        self,
        current_pose: Dict,
        target_waypoint: Tuple[float, float],
    ) -> Dict:
        x_mm = current_pose["x_mm"]
        y_mm = current_pose["y_mm"]
        theta_deg = current_pose["theta_deg"]
        theta_rad = math.radians(theta_deg)
        dx_mm = target_waypoint[0] - x_mm
        dy_mm = target_waypoint[1] - y_mm
        distance_mm = math.sqrt(dx_mm**2 + dy_mm**2)
        desired_theta_rad = math.atan2(dy_mm, dx_mm)
        angle_error_rad = desired_theta_rad - theta_rad
        angle_error_rad = math.atan2(math.sin(angle_error_rad), math.cos(angle_error_rad))

        now = time.monotonic()
        if self._last_target_waypoint != target_waypoint:
            self._last_heading_error_rad = None
            self._last_control_time = None
            self._last_target_waypoint = target_waypoint

        if self._last_heading_error_rad is None or self._last_control_time is None:
            d_error = 0.0
        else:
            dt = max(0.02, min(0.5, now - self._last_control_time))
            error_delta = angle_error_rad - self._last_heading_error_rad
            error_delta = math.atan2(math.sin(error_delta), math.cos(error_delta))
            d_error = max(-3.0, min(3.0, error_delta / dt))

        self._last_heading_error_rad = angle_error_rad
        self._last_control_time = now

        w = (self.K_p * angle_error_rad) + (self.K_d * d_error)
        w = max(-self.max_w, min(self.max_w, w))
        if abs(angle_error_rad) < self.heading_deadband_rad:
            w = 0.0

        if distance_mm < self.waypoint_tolerance_mm:
            v = 0.0
            w = 0.0
        elif abs(angle_error_rad) > self.rotate_in_place_rad:
            v = 0.0
        else:
            alignment = max(0.35, math.cos(angle_error_rad))
            distance_scale = max(0.35, min(1.0, distance_mm / 900.0))
            v = self.max_v * alignment * distance_scale
            w = max(-self.forward_turn_limit, min(self.forward_turn_limit, w))

        return {
            "v": v,
            "w": w,
            "angle_error_deg": math.degrees(angle_error_rad),
            "distance_mm": distance_mm,
        }

    def is_at_waypoint(
        self,
        current_pose: Dict,
        waypoint: Tuple[float, float],
        tolerance_mm: float = None,
    ) -> bool:
        if tolerance_mm is None:
            tolerance_mm = self.waypoint_tolerance_mm
        dx = waypoint[0] - current_pose["x_mm"]
        dy = waypoint[1] - current_pose["y_mm"]
        distance = math.sqrt(dx**2 + dy**2)
        return distance < tolerance_mm
