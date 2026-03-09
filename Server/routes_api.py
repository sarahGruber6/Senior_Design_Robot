from flask import Blueprint, request, jsonify
from datetime import datetime, timezone

from .config import ROBOT_ID, TOPIC_JOB, TOPIC_TWIST, TOPIC_DONE, TOPIC_TELEMETRY
from .db import enqueue_job, list_jobs, get_active_job, claim_next_job, mark_done
from .mqtt_client import STATE, MqttBus
from .admin import archive_db

api = Blueprint("api", __name__)

def now_iso():
    return datetime.now(timezone.utc).isoformat()

def bind_api(bus: MqttBus):
    # -------- Status --------

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

    # -------- Jobs --------

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
        return jsonify({"ok": True, "status": "queued", "job_id": payload["job_id"]})

    # -------- Robot Stuff --------

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
            return jsonify({"ok": True, "job": active})
        nxt = claim_next_job()  # otherwise claim next queued job
        if not nxt:
            return jsonify({"ok": True, "job": None})
        bus.publish_job(nxt)
        return jsonify({"ok": True, "job": nxt})    

    # -------- Admin --------

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
        SPEED = float(data.get("speed", 0.25))  # m/s
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
        
        result = bus.publish_twist_and_wait_ack(v=v,w=w,ttl_ms=ttl_ms,mode="manual")
        status = 200 if result.get("ok") else 504
        return jsonify(result), status

    @api.post("/api/admin/archive_db")
    def api_archive():
        ok, msg = archive_db(bus)
        return jsonify({"ok": ok, "message": msg}), (200 if ok else 409)

    return api
