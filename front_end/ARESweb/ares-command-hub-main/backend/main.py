from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from pathlib import Path
import time
import math

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

START_TIME = time.time()
BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"

app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

latest_telemetry = {
    "camera_stream_url": "http://127.0.0.1:8000/static/demo.mp4",
    "camera_status": "online",
    "linear_velocity": 0.0,
    "angular_velocity": 0.0,
    "odom_x": 0.0,
    "odom_y": 0.0,
    "odom_yaw": 0.0,
    "odometry_status": "online",
    "velocity_status": "online",
    "connection_status": "connected",
    "last_update": "--:--:--"
}

class TelemetryPayload(BaseModel):
    linear_velocity: float
    angular_velocity: float
    odom_x: float
    odom_y: float
    odom_yaw: float
    camera_status: str = "online"
    odometry_status: str = "online"
    velocity_status: str = "online"
    connection_status: str = "connected"

@app.get("/api/telemetry")
def get_telemetry():
    t = time.time() - START_TIME

    latest_telemetry["linear_velocity"] = round(0.4 + 0.1 * math.sin(t), 3)
    latest_telemetry["angular_velocity"] = round(0.1 * math.cos(t / 2), 3)
    latest_telemetry["odom_x"] = round(1.5 + 0.5 * math.cos(t / 4), 3)
    latest_telemetry["odom_y"] = round(2.0 + 0.5 * math.sin(t / 4), 3)
    latest_telemetry["odom_yaw"] = round((t / 5) % 6.28, 3)
    latest_telemetry["last_update"] = time.strftime("%H:%M:%S")

    return latest_telemetry

@app.post("/api/telemetry")
def update_telemetry(payload: TelemetryPayload):
    latest_telemetry.update(payload.model_dump())
    latest_telemetry["last_update"] = time.strftime("%H:%M:%S")
    return {"ok": True, "message": "Telemetry updated"}