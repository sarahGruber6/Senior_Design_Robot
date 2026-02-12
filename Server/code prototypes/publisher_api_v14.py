# v1.4 adds "archive DB" and "Job Queue" buttons for easy testing and archive functionality!

import json
import os
from datetime import datetime, timezone
import shutil   # for archiving db
from flask import Flask, request, jsonify, Response
import paho.mqtt.client as mqtt
import sqlite3 
from pathlib import Path

BROKER_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
BROKER_PORT = int(os.getenv("MQTT_PORT", "1883"))
ROBOT_ID = os.getenv("ROBOT_ID", "r1")
DB_PATH = Path("jobs.db") # local for now? or forever?
ARCHIVE_DIR = Path("jobs_archive")

TOPIC_JOB = f"robot/{ROBOT_ID}/cmd/job"             # command
TOPIC_DONE = f"robot/{ROBOT_ID}/evt/done"           # event
TOPIC_TELEMETRY = f"robot/{ROBOT_ID}/telemetry"     # timer will send to this on the robot

STATE = {
    "last_published_job": None,
    "last_done": None,
    "last_telemetry": None,
}

app = Flask(__name__)

mqttc = mqtt.Client(client_id="job_publisher_api")

# later to enable username/password:
# mqttc.username_pw_set(os.getenv("MQTT_USER"), os.getenv("MQTT_PASS"))


# ----------------------- helper functions -----------------------

def get_db():
    conn = sqlite3.connect(DB_PATH,timeout=5)
    conn.row_factory = sqlite3.Row
    # write ahead logging to allow concurrency! 
    conn.execute("PRAGMA journal_mode=WAL;")
    conn.execute("PRAGMA synchronous=NORMAL;")
    conn.execute("PRAGMA busy_timeout=5000;")  # ms
    return conn

def init_db():
    conn = get_db()
    # crazy execute method takes in strings as commands, "?" indicate variables you also can pass in (see jobs and claim_next endpoints)
    conn.execute("""    
        CREATE TABLE IF NOT EXISTS jobs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            job_id TEXT UNIQUE,
            destination TEXT,
            items TEXT,
            note TEXT,
            created_by TEXT,
            created_at TEXT,
            status TEXT
        )
    """)
    conn.commit()
    conn.close()

def archive_db():
    # doesnt archive if theres an active job
    conn = get_db()
    active = conn.execute("SELECT 1 FROM jobs WHERE status='active' LIMIT 1").fetchone()
    conn.close()

    if active:
        return False, "Cannot archive DB while a job is active."
    if not DB_PATH.exists():
        # nothing to archive so just re-init
        init_db()
        clear_retained_job()
        return True, "No existing DB found; initialized a fresh DB."
    
    ts = datetime.now().strftime("%m-%d-%Y_%H%M%S")  # Month-Day-Year_HourMinuteSeconds , ex. 2-11-2026_183133
    archived_db = ARCHIVE_DIR / f"jobs_{ts}.db"
    archived_wal = ARCHIVE_DIR / f"jobs_{ts}.db-wal"
    archived_shm = ARCHIVE_DIR / f"jobs_{ts}.db-shm"

    # overwrite protection
    if archived_db.exists():
        return False, f"Archive file already exists: {archived_db.name}"
    # rename/move
    try:
        ARCHIVE_DIR.mkdir(exist_ok=True)
        shutil.move(str(DB_PATH), str(archived_db))
        wal = DB_PATH.with_suffix(DB_PATH.suffix + "-wal")
        shm = DB_PATH.with_suffix(DB_PATH.suffix + "-shm")
        if wal.exists():
            shutil.move(str(wal), str(archived_wal))
        if shm.exists():
            shutil.move(str(shm), str(archived_shm))
    except PermissionError:
        return False,(
            "DB is in use (locked). Close any SQLite viewers/VSCode DB extensions, "
            "and make sure you don't have a second Flask instance running. "
            "Then try again.")
    # create new
    init_db()
    clear_retained_job()
    return True, f"Archived to {archived_db.name} and created fresh jobs.db"

def clear_retained_job():
    # just publish empty retained message to clear job
    mqttc.publish(TOPIC_JOB, payload=b"", qos=1, retain=True)

def on_connect(client, userdata, flags, rc):
    # subscribe when connected or reconnected
    client.subscribe([(TOPIC_DONE,1), (TOPIC_TELEMETRY,0)])
    # rc is return code. 0 = success, 1 = incorrect protocol version, 2 = invalid client identifier,
    # 3 = server unavailable, 4 = bad username or password, 5 = not authorized
    print(f"[MQTT] Connected rc = {rc}. Subscribed to {TOPIC_DONE} and {TOPIC_TELEMETRY}")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload_raw = msg.payload.decode("utf-8", errors="replace").strip()

    if topic == TOPIC_DONE:
        # expect {"job_id":"Jxxx"} or just "Jxxx"
        job_id = None
        try:                            # json format
            data = json.loads(payload_raw) if payload_raw else {}
            job_id = data.get("job_id")
        except json.JSONDecodeError:    # plain text format
            job_id = payload_raw

        if job_id:
            conn = get_db()
            conn.execute("""
            UPDATE jobs SET status = 'done'
            WHERE job_id = ?
        """, (job_id,))
            conn.commit()
            conn.close()

        STATE["last_done"] = {"job_id": job_id, "raw": payload_raw, "at": now_iso()}
        print(f"[MQTT] DONE received job_id = {job_id}. Clearing retained job.")
        clear_retained_job()

    elif topic == TOPIC_TELEMETRY:
        STATE["last_telemetry"] = {"raw": payload_raw, "at": now_iso()}

def now_iso():
    # uses UTC for now
    return datetime.now(timezone.utc).isoformat()

def enqueue_job(payload: dict):
    conn = get_db()
    try:
        conn.execute("""
            INSERT INTO jobs (job_id, destination, items, note, created_by, created_at, status)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, (
            payload["job_id"],
            payload["destination"],
            json.dumps(payload["items"]),
            payload["note"],
            payload["created_by"],
            payload["created_at"],
            "queued"
        ))
        conn.commit()
    except sqlite3.IntegrityError:
        conn.close()
        return False, "job_id already exists"
    conn.close()
    return True, None

def publish_job(payload: dict):
    msg = json.dumps(payload, separators=(",", ":"))
    # retain so robot reconnect instantly sees latest job
    mqttc.publish(TOPIC_JOB, msg, qos=1, retain=True)
    STATE["last_published_job"] = payload   # store in state        

# ----------------------- initializations/events -----------------------

init_db()
mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
mqttc.loop_start()

# ----------------------- endpoints -----------------------

@app.get("/health")
def health():
    return jsonify({"ok": True, "mqtt_host": BROKER_HOST, "mqtt_port": BROKER_PORT, "robot_id": ROBOT_ID})

@app.get("/status")
def status():
    return jsonify({
        "ok": True,
        "robot_id": ROBOT_ID,
        "topics": {"jobs": TOPIC_JOB, "done": TOPIC_DONE, "telemetry": TOPIC_TELEMETRY},
        "state": STATE,
    })

@app.get("/")
def index():
    # simple HTML: destination + multiline items "name,qty" per line
    # chat made this, sorry its a bit ugly
    # good luck Alicia 
    html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Robot Job Publisher</title>
  <style>
    body {{ font-family: system-ui, Arial; margin: 24px; max-width: 900px; }}
    label {{ display:block; margin-top: 12px; font-weight: 600; }}
    input, textarea {{ width: 100%; padding: 10px; font-size: 14px; }}
    button {{ margin-top: 16px; padding: 12px 16px; font-size: 14px; cursor: pointer; }}
    .hint {{ color: #666; font-size: 13px; margin-top: 6px; }}
    .box {{ background:#f7f7f7; padding:16px; border-radius:12px; }}
  </style>
</head>
<body>
  <h2>Robot Job Publisher (Robot: {ROBOT_ID})</h2>
  <div class="box">
    <form method="post" action="/submit">
      <label>Job ID</label>
      <input name="job_id" value="J001"/>

      <label>Destination</label>
      <input name="destination" value="Bin_A3"/>

      <label>Items (one per line: Part Name,Qty)</label>
      <textarea name="items" rows="6">
M3x10 screw,12
10k resistor,50
      </textarea>
      <div class="hint">
        Example line: <code>10k resistor,50</code>
      </div>

      <label>Note (optional)</label>
      <input name="note" value="Load into tote #2"/>

      <label>Created By</label>
      <input name="created_by" value="labtech1"/>

      <button type="submit">Queue Job</button>
    </form>
    <form method="get" action="/api/jobs">
        <button type="submit">View Job Queue</button>
    </form>
    <hr style="margin:18px 0;">
        <form method="post" action="/api/robot/claim_next">
            <button type="submit">Claim Next Job</button>
        </form>
        <form method="post" action="/api/admin/archive_db">
            <button type="submit">Archive DB (wipe queue)</button>
        </form>
  </div>
  <p class="hint">This publishes to MQTT topic: <code>{TOPIC_JOB}</code> (QoS1, retained)</p>
</body>
</html>"""
    return Response(html, mimetype="text/html")

@app.post("/submit")
def submit_form():
    # convert form to same JSON format as API
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
        return Response(f"{err}",status=400)

    return Response(
        f"<p>Queued job <b>{job_id}</b> (status: <b>queued</b>).</p>"
        f"<p>Robot will receive it when it claims the next job.</p>"
        f"<p><a href='/'>Publish another</a></p>"
        f"<pre>{json.dumps(payload, indent=2)}</pre>",
        mimetype="text/html"
    )

# ----------------------- API endpoints -----------------------
@app.post("/api/jobs")
def create_job_json():
    # expects JSON   
    data = request.get_json(force=True)

    # minimal validation
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

    conn = get_db()
    try:
        conn.execute("""
            INSERT INTO jobs (job_id, destination, items, note, created_by, created_at, status)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, (
            payload["job_id"],
            payload["destination"],
            json.dumps(payload["items"]),
            payload["note"],
            payload["created_by"],
            payload["created_at"],
            "queued"
        ))
        conn.commit()
    except sqlite3.IntegrityError:
        conn.close()
        return jsonify({"error": "job_id already exists"}), 400
    
    conn.close()
    # doesn't immediately publish anymore, just adds to queue
    return jsonify({"ok": True, "status": "queued", "job_id": payload["job_id"]})

@app.get("/api/jobs")
def list_jobs():
    conn = get_db()
    rows = conn.execute("""
        SELECT job_id, destination, items, note, created_by, created_at, status
        FROM jobs
        ORDER BY id DESC
        LIMIT 200
    """).fetchall()
    conn.close()

    out = []
    for r in rows:
        out.append({
            "job_id": r["job_id"],
            "destination": r["destination"],
            "items": json.loads(r["items"]),
            "note": r["note"],
            "created_by": r["created_by"],
            "created_at": r["created_at"],
            "status": r["status"],        
        })
    return jsonify({"ok": True, "jobs": out})

@app.post("/api/robot/claim_next")
def claim_next():
    conn = get_db()

    # check if job is already active
    active = conn.execute("""
            SELECT * FROM jobs
            WHERE status = 'active'
            ORDER BY id ASC
            LIMIT 1
        """).fetchone()
    if active:
        payload = {
            "job_id": active["job_id"],
            "destination": active["destination"],
            "items": json.loads(active["items"]),
            "note": active["note"],
            "created_by": active["created_by"],
            "created_at": active["created_at"],
        }
        publish_job(payload)
        conn.close()
        return jsonify({"ok": True, "message": "Active job already assigned", "active": payload})

    # gets here if no current active job, claims next queued job
    row = conn.execute("""
        SELECT * FROM jobs
        WHERE status = 'queued'
        ORDER BY id ASC
        LIMIT 1
    """).fetchone()

    if not row:
        conn.close()
        return jsonify({"ok": False, "message": "No queued jobs"})
    
    conn.execute("""
        UPDATE jobs SET status = 'active'
        WHERE id = ?
    """, (row["id"],))
    conn.commit()

    payload = {
        "job_id": row["job_id"],
        "destination": row["destination"],
        "items": json.loads(row["items"]),
        "note": row["note"],
        "created_by": row["created_by"],
        "created_at": row["created_at"],
    }
    publish_job(payload)
    conn.close()

    return jsonify({"ok": True, "claimed": payload})

@app.post("/api/admin/archive_db")
def api_archive_db():
    ok,msg = archive_db()
    code = 200 if ok else 409
    return jsonify({"ok": ok, "message": msg}), code

if __name__ == "__main__":
    # Host 0.0.0.0 so lab machines can reach it on LAN
    app.run(host="0.0.0.0", port=5000, debug=True)
