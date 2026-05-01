from flask import Blueprint, render_template, request, Response
from datetime import datetime, timezone
import json

from .db import enqueue_job, list_jobs
from .config import ROBOT_ID, TOPIC_JOB, TOPIC_TWIST

pages = Blueprint("pages", __name__)

def now_iso():
    return datetime.now(timezone.utc).isoformat()

@pages.get("/")
def index():
    return render_template("order.html", robot_id=ROBOT_ID)

@pages.get("/admin")
def admin_page():
    return render_template("admin.html", robot_id=ROBOT_ID)
 
@pages.post("/submit")
def submit_form():
    job_id = request.form.get("job_id", "").strip()
    destination = request.form.get("destination", "").strip()
    note = request.form.get("note", "").strip()
    created_by = request.form.get("created_by", "").strip() or "unknown"

    raw_items = request.form.get("items", "")
    items = []
    for line in raw_items.splitlines():
        line = line.strip()
        if not line:
            continue
        if "," not in line:
            return Response(f"Bad line (missing comma): {line}", status=400)
        part, qty = line.split(",", 1)
        part = part.strip()
        qty = qty.strip()
        try:
            qty_i = int(qty)
        except ValueError:
            return Response(f"Bad qty for line: {line}", status=400)
        items.append({"part": part, "qty": qty_i})

    if not job_id or not destination or not items:
        return Response("Missing job_id, destination, or items", status=400)

    payload = {
        "job_id": job_id,
        "destination": destination,
        "items": items,
        "note": note,
        "created_by": created_by,
        "created_at": now_iso(),
    }

    ok, err = enqueue_job(payload)
    if not ok:
        return Response(err, status=400)

    payload["status"] = "queued"

    return render_template("queued.html", job=payload)

@pages.get("/queue")
def queue_page():
    status_filter = request.args.get("status", "").strip().lower()
    jobs = list_jobs()

    if status_filter in {"queued", "active", "done"}:
        jobs = [j for j in jobs if j.get("status") == status_filter]

    return render_template("view_queue.html", jobs=jobs, status_filter=(status_filter or "all"))
