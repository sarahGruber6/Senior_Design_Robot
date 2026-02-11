import json
import os
from datetime import datetime, timezone
from flask import Flask, request, jsonify
import paho.mqtt.client as mqtt

BROKER_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
BROKER_PORT = int(os.getenv("MQTT_PORT", "1883"))
ROBOT_ID = os.getenv("ROBOT_ID", "r1")

TOPIC_JOB = f"robot/{ROBOT_ID}/cmd/job"             # command

app = Flask(__name__)

mqttc = mqtt.Client(client_id="job_publisher_api")

# later to enable username/password:
# mqttc.username_pw_set(os.getenv("MQTT_USER"), os.getenv("MQTT_PASS"))

mqttc.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
mqttc.loop_start()

def now_iso_local():
    # uses UTC for now
    return datetime.now(timezone.utc).isoformat()

@app.post("/api/jobs")
def create_job():
    """
    Expects JSON:
    {
      "job_id": "...",
      "destination": "...",
      "items": [{"part": "...", "qty": 1}, ...],
      "note": "...",
      "created_by": "..."
    }
    """
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
        "created_at": now_iso_local(),
    }

    msg = json.dumps(payload, separators=(",", ":"))
    # retain = true so robot can reconnect and immediately see the current job
    res = mqttc.publish(TOPIC_JOB, msg, qos=1, retain=True)

    return jsonify({"ok": True, "topic": TOPIC_JOB, "mid": res.mid, "payload": payload})

@app.get("/health")
def health():
    return jsonify({"ok": True, "mqtt_host": BROKER_HOST, "mqtt_port": BROKER_PORT})

if __name__ == "__main__":
    # Host 0.0.0.0 so lab machines can reach it on LAN
    app.run(host="0.0.0.0", port=5000, debug=True)
