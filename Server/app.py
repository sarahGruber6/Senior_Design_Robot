from flask import Flask
from .db import init_db
from .mqtt_client import MqttBus
from .routes_api import bind_api
from .routes_pages import pages

def create_app():
    app = Flask(__name__)

    # Init DB first
    init_db()

    # Start MQTT bus
    bus = MqttBus()
    bus.start()

    # Register routes
    app.register_blueprint(pages)
    app.register_blueprint(bind_api(bus))

    return app

if __name__ == "__main__":
    app = create_app()
    # Windows: avoid reloader dup-process issues
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)
