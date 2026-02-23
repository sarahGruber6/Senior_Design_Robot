import json
import sqlite3
from typing import Any, Dict, List, Optional, Tuple
from .config import DB_PATH

def get_db():
    conn = sqlite3.connect(DB_PATH, timeout=5)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL;")
    conn.execute("PRAGMA synchronous=NORMAL;")
    conn.execute("PRAGMA busy_timeout=5000;")
    return conn

def init_db():
    conn = get_db()
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

def enqueue_job(payload: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
    conn = get_db()
    try:
        conn.execute("""
            INSERT INTO jobs (job_id, destination, items, note, created_by, created_at, status)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, (
            payload["job_id"],
            payload["destination"],
            json.dumps(payload["items"]),
            payload.get("note", ""),
            payload.get("created_by", "unknown"),
            payload["created_at"],
            "queued"
        ))
        conn.commit()
    except sqlite3.IntegrityError:
        conn.close()
        return False, "job_id already exists"
    conn.close()
    return True, None

def list_jobs(limit: int = 200) -> List[Dict[str, Any]]:
    conn = get_db()
    rows = conn.execute("""
        SELECT job_id, destination, items, note, created_by, created_at, status
        FROM jobs
        ORDER BY id DESC
        LIMIT ?
    """, (limit,)).fetchall()
    conn.close()

    out: List[Dict[str, Any]] = []
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
    return out

def get_active_job() -> Optional[Dict[str, Any]]:
    conn = get_db()
    row = conn.execute("""
        SELECT * FROM jobs
        WHERE status='active'
        ORDER BY id ASC
        LIMIT 1
    """).fetchone()
    conn.close()
    if not row:
        return None
    return {
        "job_id": row["job_id"],
        "destination": row["destination"],
        "items": json.loads(row["items"]),
        "note": row["note"],
        "created_by": row["created_by"],
        "created_at": row["created_at"],
    }

def claim_next_job() -> Optional[Dict[str, Any]]:
    """
    Returns next queued job and marks it active (FIFO). Returns None if none queued.
    """
    conn = get_db()
    row = conn.execute("""
        SELECT * FROM jobs
        WHERE status='queued'
        ORDER BY id ASC
        LIMIT 1
    """).fetchone()

    if not row:
        conn.close()
        return None

    conn.execute("UPDATE jobs SET status='active' WHERE id=?", (row["id"],))
    conn.commit()
    conn.close()

    return {
        "job_id": row["job_id"],
        "destination": row["destination"],
        "items": json.loads(row["items"]),
        "note": row["note"],
        "created_by": row["created_by"],
        "created_at": row["created_at"],
    }

def has_active_job() -> bool:
    conn = get_db()
    active = conn.execute("SELECT 1 FROM jobs WHERE status='active' LIMIT 1").fetchone()
    conn.close()
    return active is not None

def mark_done(job_id: str) -> None:
    conn = get_db()
    conn.execute("UPDATE jobs SET status='done' WHERE job_id=?", (job_id,))
    conn.commit()
    conn.close()
