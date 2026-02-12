from flask import Blueprint, request, jsonify
from datetime import datetime, timezone

from .config import ROBOT_ID, TOPIC_JOB, TOPIC_DONE, TOPIC_TELEMETRY
from .db import enqueue_job, list_jobs, get_active_job, claim_next_job
from .mqtt_client import STATE, MqttBus
from .admin import archive_db

api = Blueprint("api", __name__)

def now_iso():
    return datetime.now(timezone.utc).isoformat()

def bind_api(bus: MqttBus):
    # Health / status
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

    # Jobs
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
        return jsonify({"ok": True, "status": "queued", "job_id": payload["job_id"]})

    # Robot claim
    @api.post("/api/robot/claim_next")
    def api_claim_next():
        active = get_active_job()
        if active:
            bus.publish_job(active)
            return jsonify({"ok": True, "message": "Active job already assigned", "active": active})

        nxt = claim_next_job()
        if not nxt:
            return jsonify({"ok": False, "message": "No queued jobs"})

        bus.publish_job(nxt)
        return jsonify({"ok": True, "message": "Claimed next job", "claimed": nxt})

    # Admin archive
    @api.post("/api/admin/archive_db")
    def api_archive():
        ok, msg = archive_db(bus)
        return jsonify({"ok": ok, "message": msg}), (200 if ok else 409)

    return api
