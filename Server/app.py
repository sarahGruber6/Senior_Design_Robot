from flask import Flask
from .db import init_db
from .mqtt_client import MqttBus
from .slam_service import SlamService
from .motion_executor import MotionExecutor
from .routes_api import bind_api
from .routes_pages import pages

def create_app():
    app = Flask(__name__)

    init_db()

    bus = MqttBus()
    bus.start()

    slam_service = SlamService(bus)
    slam_service.start()

    from . import db
    motion_executor = MotionExecutor(bus, slam_service, db)
    motion_executor.start()

    app.slam_service = slam_service
    app.motion_executor = motion_executor

    app.register_blueprint(pages)
    app.register_blueprint(bind_api(bus, slam_service, motion_executor))

    return app

if __name__ == "__main__":
    app = create_app()
    # Windows: avoid reloader dup-process issues
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)
