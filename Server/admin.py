import shutil
from datetime import datetime
from pathlib import Path

from .config import DB_PATH, ARCHIVE_DIR
from .db import init_db, has_active_job
from .mqtt_client import STATE, MqttBus

def archive_db(bus: MqttBus):
    if has_active_job():
        return False, "Cannot archive DB while a job is active."

    if not DB_PATH.exists():
        init_db()
        bus.clear_retained_job()
        return True, "No existing DB found; initialized a fresh DB."

    ARCHIVE_DIR.mkdir(exist_ok=True)

    ts = datetime.now().strftime("%m-%d-%Y_%H%M%S")
    archived_db = ARCHIVE_DIR / f"jobs_{ts}.db"
    archived_wal = ARCHIVE_DIR / f"jobs_{ts}.db-wal"
    archived_shm = ARCHIVE_DIR / f"jobs_{ts}.db-shm"

    if archived_db.exists():
        return False, f"Archive file already exists: {archived_db.name}"

    try:
        shutil.move(str(DB_PATH), str(archived_db))

        wal = DB_PATH.parent / (DB_PATH.name + "-wal")
        shm = DB_PATH.parent / (DB_PATH.name + "-shm")

        if wal.exists():
            shutil.move(str(wal), str(archived_wal))
        if shm.exists():
            shutil.move(str(shm), str(archived_shm))

    except PermissionError:
        return False, (
            "DB is in use (locked). Close any SQLite viewers/VSCode DB extensions, "
            "and make sure you don't have a second Flask instance running. Then try again."
        )

    init_db()
    bus.clear_retained_job()

    # Optional: clear runtime state
    STATE["last_published_job"] = None
    STATE["last_done"] = None

    return True, f"Archived to {archived_db.name} and created fresh jobs.db"
