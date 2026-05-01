import math
import threading
import time
from typing import List, Tuple, Optional, Dict

from .pathfinder import OccupancyGrid, AStarPathfinder
from .motor_commander import MotorCommander
from . import config


class MotionExecutor:
    def __init__(self, mqtt_bus, slam_service, db_module):
        self.mqtt_bus = mqtt_bus
        self.slam_service = slam_service
        self.db = db_module

        self._running = False
        self._thread = None
        self._lock = threading.Lock()

        # Navigation state
        self.current_job_id = None
        self.current_goal_mm: Optional[Tuple[float, float]] = None
        self.planned_path: Optional[List[Tuple[float, float]]] = None
        self.path_index = 0
        self._executing = False   # True only after go() is called
        self._blocked_by_obstacle = False
        self._planning_mode = None
        self._plan_message = ""
        self._last_twist = {"v": 0.0, "w": 0.0}
        self._distance_to_goal_mm = None
        self.motor_commander = MotorCommander(max_v=0.18, max_w=0.50)

        self.stats = {
            "jobs_completed": 0,
            "jobs_failed": 0,
            "paths_planned": 0,
            "paths_failed": 0,
        }

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print("[MOTION] MotionExecutor started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)
        print("[MOTION] MotionExecutor stopped")

    def set_goal(self, goal_x_mm: float, goal_y_mm: float, job_id: str = None) -> Dict:
        if not self.slam_service.is_ready():
            return {"ok": False, "error": "SLAM not ready — no scans received yet"}

        map_size_mm = self.slam_service.map_size_m * 1000
        if not (0 <= goal_x_mm <= map_size_mm and 0 <= goal_y_mm <= map_size_mm):
            return {"ok": False, "error": f"Goal must be within the {int(map_size_mm)} mm map"}

        current_pose = self.slam_service.get_pose()

        try:
            # Use static map for planning so dynamic obstacles (people) don't block paths
            mapbytes = self.slam_service.get_planning_map()
            grid = OccupancyGrid(
                mapbytes,
                self.slam_service.map_pixels,
                self.slam_service.map_pixels,
                self.slam_service.map_size_m,
                robot_radius_mm=config.ROBOT_RADIUS_MM,
            )
            path = AStarPathfinder(grid).plan(
                (current_pose["x_mm"], current_pose["y_mm"]),
                (goal_x_mm, goal_y_mm),
            )
            planning_mode = "astar"
            plan_message = "A* path planned"
        except Exception as e:
            print(f"[MOTION] Path planning error: {e}")
            path = None
            planning_mode = "direct_fallback"
            plan_message = f"A* error, using direct demo path: {e}"

        if path is None:
            path = self._direct_path(
                (current_pose["x_mm"], current_pose["y_mm"]),
                (goal_x_mm, goal_y_mm),
            )
            planning_mode = "direct_fallback"
            plan_message = "A* could not find a route, using direct demo path"

        if len(path) < 2:
            self.stats["paths_failed"] += 1
            return {"ok": False, "error": "Goal is too close or invalid"}

        with self._lock:
            self.current_goal_mm = (goal_x_mm, goal_y_mm)
            self.planned_path = path
            self.path_index = 1 if len(path) > 1 else 0
            self.current_job_id = job_id
            self._executing = False   # planned, not yet executing
            self._blocked_by_obstacle = False
            self._planning_mode = planning_mode
            self._plan_message = plan_message
            self._last_twist = {"v": 0.0, "w": 0.0}
            self._distance_to_goal_mm = math.dist(
                (current_pose["x_mm"], current_pose["y_mm"]),
                (goal_x_mm, goal_y_mm),
            )
            self.motor_commander.reset()

        self.stats["paths_planned"] += 1
        return {
            "ok": True,
            "goal": (goal_x_mm, goal_y_mm),
            "path_length": len(path),
            "path": path,
            "planning_mode": planning_mode,
            "message": plan_message,
        }

    def go(self) -> Dict:
        with self._lock:
            if not self.planned_path or self.current_goal_mm is None:
                return {"ok": False, "error": "No path planned — call /api/autonomy/goal first"}
            self.path_index = max(1, min(self.path_index, len(self.planned_path) - 1))
            self._executing = True
            self.motor_commander.reset()
        return {"ok": True, "message": "Executing path", "planning_mode": self._planning_mode}

    def plan_and_go(self, goal_x_mm: float, goal_y_mm: float, job_id: str = None) -> Dict:
        result = self.set_goal(goal_x_mm, goal_y_mm, job_id)
        if not result.get("ok"):
            return result
        go_result = self.go()
        result["executing"] = go_result.get("ok", False)
        if not go_result.get("ok"):
            result["ok"] = False
            result["error"] = go_result.get("error", "Could not start execution")
        return result

    def cancel(self):
        with self._lock:
            self._executing = False
            self.current_goal_mm = None
            self.planned_path = None
            self.path_index = 0
            self.current_job_id = None
            self._blocked_by_obstacle = False
            self._planning_mode = None
            self._plan_message = ""
            self._last_twist = {"v": 0.0, "w": 0.0}
            self._distance_to_goal_mm = None
            self.motor_commander.reset()
        # Stop the robot immediately (ttl_ms=0 = keep stopped)
        self.mqtt_bus.publish_twist(v=0.0, w=0.0, ttl_ms=0, mode="autonomy")

    def get_path(self) -> Optional[List[Tuple[float, float]]]:
        with self._lock:
            return list(self.planned_path) if self.planned_path else None

    def get_goal(self) -> Optional[Tuple[float, float]]:
        with self._lock:
            return self.current_goal_mm

    def get_status(self) -> Dict:
        with self._lock:
            return {
                "executing": self._executing,
                "has_path": self.planned_path is not None,
                "goal": self.current_goal_mm,
                "path_index": self.path_index,
                "path_length": len(self.planned_path) if self.planned_path else 0,
                "blocked_by_obstacle": self._blocked_by_obstacle,
                "planning_mode": self._planning_mode,
                "message": self._plan_message,
                "last_twist": dict(self._last_twist),
                "distance_to_goal_mm": self._distance_to_goal_mm,
            }

    def _run(self):
        PUBLISH_INTERVAL_MS = 100   # 10 Hz control loop
        last_publish_ms = 0

        while self._running:
            time.sleep(0.01)

            now_ms = int(time.time() * 1000)
            if now_ms - last_publish_ms < PUBLISH_INTERVAL_MS:
                continue
            last_publish_ms = now_ms

            twist = None
            goal_done = False
            done_job_id = None

            with self._lock:
                if not self._executing or not self.planned_path or self.current_goal_mm is None:
                    continue
                current_pose = self.slam_service.get_pose()
                robot_status = self.slam_service.get_robot_status()
                obstacle = bool(robot_status.get("obstacle", 0))

                if self.motor_commander.is_at_waypoint(current_pose, self.current_goal_mm, tolerance_mm=200):
                    done_job_id = self.current_job_id
                    goal_done = True
                    self._executing = False
                    self._blocked_by_obstacle = False
                    self._last_twist = {"v": 0.0, "w": 0.0}
                    self._distance_to_goal_mm = 0.0
                    self.current_goal_mm = None
                    self.planned_path = None
                    self.current_job_id = None
                    self.motor_commander.reset()
                else:
                    self._distance_to_goal_mm = math.dist(
                        (current_pose["x_mm"], current_pose["y_mm"]),
                        self.current_goal_mm,
                    )
                    self._advance_waypoint(current_pose)
                    waypoint = self._find_lookahead_point(current_pose, self.planned_path)
                    twist = self.motor_commander.compute_twist(current_pose, waypoint)
                    if obstacle and twist["v"] > 0.01:
                        self._blocked_by_obstacle = True
                        twist = {"v": 0.0, "w": 0.0}
                    else:
                        self._blocked_by_obstacle = False
                    self._last_twist = twist

            if goal_done:
                print("[MOTION] Goal reached")
                self.mqtt_bus.publish_twist(v=0.0, w=0.0, ttl_ms=0, mode="autonomy")
                if done_job_id:
                    try:
                        self.mqtt_bus.publish_done(done_job_id, clear_job=True)
                        self.db.mark_done(done_job_id)
                        self.stats["jobs_completed"] += 1
                    except Exception as e:
                        print(f"[MOTION] Job completion error: {e}")
            elif twist is not None:
                # ttl_ms=200: robot coasts to a stop if the server loop ever stalls
                self.mqtt_bus.publish_twist(v=twist["v"], w=twist["w"], ttl_ms=200, mode="autonomy")

    def _advance_waypoint(self, current_pose: Dict):
        if not self.planned_path:
            return
        last_index = len(self.planned_path) - 1
        while self.path_index < last_index:
            if not self.motor_commander.is_at_waypoint(
                current_pose, self.planned_path[self.path_index], tolerance_mm=300
            ):
                break
            self.path_index += 1

    def _find_lookahead_point(
        self,
        current_pose: Dict,
        path: List[Tuple[float, float]],
        lookahead_mm: float = 550,
    ) -> Tuple[float, float]:
        prev_x, prev_y = current_pose["x_mm"], current_pose["y_mm"]
        accum = 0.0
        for i in range(self.path_index, len(path)):
            wx, wy = path[i]
            seg = math.dist((prev_x, prev_y), (wx, wy))
            if accum + seg >= lookahead_mm:
                t = (lookahead_mm - accum) / max(seg, 1e-9)
                return (prev_x + t * (wx - prev_x), prev_y + t * (wy - prev_y))
            accum += seg
            prev_x, prev_y = wx, wy
        return path[-1]

    def _direct_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> List[Tuple[float, float]]:
        return [start, goal]
