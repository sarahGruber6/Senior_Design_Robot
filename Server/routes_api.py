from flask import Blueprint, request, jsonify
from datetime import datetime, timezone
import io
import base64

from .config import ROBOT_ID, TOPIC_JOB, TOPIC_TWIST, TOPIC_DONE, TOPIC_TELEMETRY, NAMED_LOCATIONS
from .db import enqueue_job, list_jobs, get_active_job, claim_next_job, mark_done, list_places, save_place, delete_place
from .mqtt_client import STATE, MqttBus
from .admin import archive_db

api = Blueprint("api", __name__)

def now_iso():
    return datetime.now(timezone.utc).isoformat()

def _resolve_destination(destination: str):
    """Return (x_mm, y_mm) from a named location, DB place, or 'x,y' string, or None."""
    if destination in NAMED_LOCATIONS:
        return NAMED_LOCATIONS[destination]
    for place in list_places():
        if place["name"] == destination:
            return place["x_mm"], place["y_mm"]
    try:
        parts = destination.split(",")
        return float(parts[0]), float(parts[1])
    except Exception:
        return None


def _try_start_motion(motion_executor, slam_service, job: dict):
    """If SLAM is ready and destination resolves, plan + start autonomous execution."""
    if not motion_executor or not slam_service or not slam_service.is_ready():
        return
    coords = _resolve_destination(job.get("destination", ""))
    if not coords:
        return
    result = motion_executor.set_goal(coords[0], coords[1], job.get("job_id"))
    if result.get("ok"):
        motion_executor.go()


def bind_api(bus: MqttBus, slam_service=None, motion_executor=None):
    @api.get("/health")
    def health():
        return jsonify({"ok": True, "robot_id": ROBOT_ID})

    @api.get("/status")
    def status():
        return jsonify({
            "ok": True,
            "robot_id": ROBOT_ID,
            "topics": {"jobs": TOPIC_JOB, "done": TOPIC_DONE, "telemetry": TOPIC_TELEMETRY},
            "state": STATE,
        })

    @api.get("/api/jobs")
    def api_list_jobs():
        return jsonify({"ok": True, "jobs": list_jobs()})

    @api.post("/api/jobs")
    def api_create_job():
        data = request.get_json(force=True)
        for k in ["job_id", "destination", "items"]:
            if k not in data:
                return jsonify({"error": f"Missing field: {k}"}), 400

        payload = {
            "job_id": data["job_id"],
            "destination": data["destination"],
            "items": data["items"],
            "note": data.get("note", ""),
            "created_by": data.get("created_by", "unknown"),
            "created_at": now_iso(),
        }
        ok, err = enqueue_job(payload)
        if not ok:
            return jsonify({"error": err}), 400
        
        active = get_active_job()
        if not active:
            nxt = claim_next_job()
            if nxt:
                bus.publish_job(nxt)
                _try_start_motion(motion_executor, slam_service, nxt)
        return jsonify({"ok": True, "status": "queued", "job_id": payload["job_id"]})

    @api.post("/api/robot/claim_next")
    def api_claim_next():
        active = get_active_job()
        if active:
            bus.publish_job(active)
            return jsonify({"ok": True, "message": "Active job already assigned", "active": active})

        nxt = claim_next_job()
        if not nxt:
            return jsonify({"ok": False, "message": "No queued jobs"}), 400

        bus.publish_job(nxt)
        _try_start_motion(motion_executor, slam_service, nxt)
        return jsonify({"ok": True, "message": "Claimed next job", "claimed": nxt})
    
    @api.post("/api/robot/finish_active")
    def finish_active():
        active = get_active_job()
        if not active:
            return jsonify({"ok": False, "error": "No active job"}), 400
        job_id = active.get("job_id", "").strip()
        if not job_id:
            return jsonify({"ok": False, "error": "Active job missing job_id"}), 500
        
        bus.publish_done(job_id, clear_job=True)
        mark_done(job_id)
        return jsonify({"ok": True, "job_id": job_id})


    @api.post("/api/robot/done")
    def robot_done():
        data = request.get_json(force=True)
        job_id = data.get("job_id", "").strip()
        if not job_id:
            return jsonify({"ok": False, "error": "missing job_id"}), 400

        mark_done(job_id)
        bus.clear_retained_job() 
        return jsonify({"ok": True, "job_id": job_id})        

    @api.get("/api/robot/active_or_next")
    def active_or_next():
        active = get_active_job()
        if active:
            _try_start_motion(motion_executor, slam_service, active)
            return jsonify({"ok": True, "job": active})
        nxt = claim_next_job()  # otherwise claim next queued job
        if not nxt:
            return jsonify({"ok": True, "job": None})
        bus.publish_job(nxt)
        _try_start_motion(motion_executor, slam_service, nxt)
        return jsonify({"ok": True, "job": nxt})    

    @api.get("/api/manual/status")
    def manual_status():
        return jsonify({
            "ok": True,
            "robot_id": ROBOT_ID,
            "topic_twist": TOPIC_TWIST
        })
    
    @api.post("/api/manual/cmd")
    def manual_cmd():
        data = request.get_json(force=True)

        # input is ex. {"action":"forward"} or {"v":0.2,"w":0.0,"ttl_ms":300}
        action = (data.get("action") or "").strip().lower()
        ttl_ms = int(data.get("ttl_ms",300))

        # actions to v,w
        SPEED = float(data.get("speed", 0.3))  # m/s
        TURN = float(data.get("turn", 1.2))     # rad/s

        if action == "forward":
            v,w = SPEED,0.0
        elif action == "backward":
            v,w = -SPEED,0.0
        elif action == "left":
            v,w = 0.0,TURN
        elif action == "right":
            v,w = 0.0,-TURN
        elif action == "stop":
            v,w = 0.0,0.0
            ttl_ms = 0
        else:
            # direct
            v = float(data.get("v", 0.0))
            w = float(data.get("w", 0.0))

        result = bus.publish_twist_and_wait_ack(
            v=v,
            w=w,
            ttl_ms=ttl_ms,
            mode="manual",
        )
        status = 200 if result.get("ok") else 504
        return jsonify(result), status

    @api.post("/api/admin/archive_db")
    def api_archive():
        ok, msg = archive_db(bus)
        return jsonify({"ok": ok, "message": msg}), (200 if ok else 409)

    @api.get("/api/slam/pose")
    def api_slam_pose():
        if not slam_service:
            return jsonify({"error": "SLAM service not available"}), 503

        pose = slam_service.get_pose()
        return jsonify({
            "ok": True,
            "x_mm": pose["x_mm"],
            "y_mm": pose["y_mm"],
            "theta_deg": pose["theta_deg"],
            "ready": slam_service.is_ready(),
        })

    @api.get("/api/slam/map")
    def api_slam_map():
        if not slam_service:
            return jsonify({"error": "SLAM service not available"}), 503

        if not slam_service.is_ready():
            return jsonify({"error": "SLAM not ready, no scans received yet"}), 503

        mapbytes = slam_service.get_map()
        return jsonify({
            "ok": True,
            "width": slam_service.map_pixels,
            "height": slam_service.map_pixels,
            "data": base64.b64encode(mapbytes).decode("utf-8"),
        })

    @api.get("/api/slam/stats")
    def api_slam_stats():
        if not slam_service:
            return jsonify({"error": "SLAM service not available"}), 503

        stats = slam_service.get_stats()
        return jsonify({
            "ok": True,
            "ready": slam_service.is_ready(),
            "has_static_map": slam_service.has_static_map(),
            "stats": stats,
        })

    @api.get("/api/robot/status")
    def api_robot_status():
        if not slam_service:
            return jsonify({"ok": False, "obstacle": 0})
        rs = slam_service.get_robot_status()
        return jsonify({"ok": True, **rs})

    @api.post("/api/slam/reset")
    def api_reset_slam():
        if not slam_service:
            return jsonify({"error": "SLAM service not available"}), 503
        if motion_executor:
            motion_executor.cancel()
        result = slam_service.reset()
        return jsonify(result), (200 if result.get("ok") else 500)

    @api.post("/api/slam/save_map")
    def api_save_map():
        if not slam_service:
            return jsonify({"error": "SLAM service not available"}), 503
        if not slam_service.is_ready():
            return jsonify({"ok": False, "error": "SLAM not ready — no scans received yet"}), 400
        result = slam_service.save_static_map()
        return jsonify(result), (200 if result.get("ok") else 500)

    @api.delete("/api/slam/save_map")
    def api_clear_map():
        if not slam_service:
            return jsonify({"error": "SLAM service not available"}), 503
        result = slam_service.clear_static_map()
        return jsonify(result), (200 if result.get("ok") else 500)

    @api.post("/api/autonomy/goal")
    def api_set_goal():
        if not motion_executor:
            return jsonify({"error": "Motion executor not available"}), 503

        data = request.get_json(force=True)
        goal_x = float(data.get("x_mm", 0))
        goal_y = float(data.get("y_mm", 0))
        job_id = data.get("job_id", None)

        result = motion_executor.set_goal(goal_x, goal_y, job_id)
        status = 200 if result.get("ok") else 400
        return jsonify(result), status

    @api.get("/api/autonomy/path")
    def api_get_path():
        if not motion_executor:
            return jsonify({"error": "Motion executor not available"}), 503

        path = motion_executor.get_path()
        current_goal = motion_executor.get_goal()

        return jsonify({
            "ok": True,
            "goal": current_goal,
            "path": path,
            "path_length": len(path) if path else 0,
        })

    @api.post("/api/autonomy/go")
    def api_go():
        if not motion_executor:
            return jsonify({"error": "Motion executor not available"}), 503

        result = motion_executor.go()
        return jsonify(result), (200 if result.get("ok") else 400)

    @api.post("/api/autonomy/plan_go")
    def api_plan_go():
        if not motion_executor:
            return jsonify({"error": "Motion executor not available"}), 503

        data = request.get_json(force=True)
        goal_x = float(data.get("x_mm", 0))
        goal_y = float(data.get("y_mm", 0))
        job_id = data.get("job_id", None)

        result = motion_executor.plan_and_go(goal_x, goal_y, job_id)
        return jsonify(result), (200 if result.get("ok") else 400)

    @api.get("/api/autonomy/status")
    def api_autonomy_status():
        if not motion_executor:
            return jsonify({"error": "Motion executor not available"}), 503

        status = motion_executor.get_status()
        return jsonify({"ok": True, **status})

    @api.post("/api/autonomy/cancel")
    def api_cancel_goal():
        if not motion_executor:
            return jsonify({"error": "Motion executor not available"}), 503

        motion_executor.cancel()
        return jsonify({"ok": True, "message": "Goal cancelled"})

    @api.get("/api/places")
    def api_list_places():
        return jsonify({"ok": True, "places": list_places()})

    @api.post("/api/places")
    def api_save_place():
        data = request.get_json(force=True)
        name = (data.get("name") or "").strip()
        if not name:
            return jsonify({"ok": False, "error": "name required"}), 400
        try:
            x_mm = float(data["x_mm"])
            y_mm = float(data["y_mm"])
        except (KeyError, ValueError, TypeError):
            return jsonify({"ok": False, "error": "x_mm and y_mm required"}), 400
        ok, err = save_place(name, x_mm, y_mm, now_iso())
        return jsonify({"ok": ok, "error": err}), (200 if ok else 400)

    @api.delete("/api/places/<name>")
    def api_delete_place(name):
        deleted = delete_place(name)
        return jsonify({"ok": deleted, "error": None if deleted else "not found"})

    return api
