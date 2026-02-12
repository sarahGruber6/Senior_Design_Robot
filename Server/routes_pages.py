from flask import Blueprint, render_template, request, Response
from datetime import datetime, timezone
import json

from .db import enqueue_job
from .config import ROBOT_ID, TOPIC_JOB

pages = Blueprint("pages", __name__)

def now_iso():
    return datetime.now(timezone.utc).isoformat()

@pages.get("/")
def index():
    return render_template("index.html", robot_id=ROBOT_ID, topic_job=TOPIC_JOB)

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

    return Response(
        f"<p>Queued job <b>{job_id}</b> (status: <b>queued</b>).</p>"
        f"<p>Robot will receive it when it claims the next job.</p>"
        f"<p><a href='/'>Back</a></p>"
        f"<pre>{json.dumps(payload, indent=2)}</pre>",
        mimetype="text/html",
    )
