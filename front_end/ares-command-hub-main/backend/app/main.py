from __future__ import annotations

from pathlib import Path
from typing import Annotated

from fastapi import Body, FastAPI, HTTPException, Query, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from app.core.config import CHANNEL_PRIMARY_TOPICS
from app.models.schemas import (
    EstopRequest,
    GoalPoseRequest,
    InitialPoseRequest,
    SubscribeRequest,
    UnsubscribeRequest,
)
from app.ros.ros_manager import ros_bridge
from app.websocket.manager import websocket_manager

BASE_DIR = Path(__file__).resolve().parents[1]
STATIC_DIR = BASE_DIR / "static"

app = FastAPI(title="A.R.E.S. Command Hub ROS2 Bridge", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


@app.on_event("startup")
def startup() -> None:
    ros_bridge.start()


@app.on_event("shutdown")
def shutdown() -> None:
    ros_bridge.shutdown()


@app.get("/health")
def health() -> dict:
    return ros_bridge.health()


@app.get("/api/telemetry")
def get_telemetry() -> dict:
    return ros_bridge.get_telemetry()


@app.get("/api/rtabmap/status")
def get_rtabmap_status() -> dict:
    return ros_bridge.get_rtabmap_status()


@app.get("/ros/topics")
def get_ros_topics() -> dict:
    return ros_bridge.list_topics()


@app.get("/ros/nodes")
def get_ros_nodes() -> dict:
    return ros_bridge.list_nodes()


@app.get("/ros/topic/{topic_path:path}")
def get_ros_topic(topic_path: str) -> dict:
    topic = f"/{topic_path.lstrip('/')}"
    ros_bridge.ensure_subscription(topic)
    return ros_bridge.get_topic_payload(topic)


@app.post("/ros/subscribe")
def subscribe_to_topic(payload: SubscribeRequest) -> dict:
    result = ros_bridge.ensure_subscription(payload.topic, payload.message_type)
    if not result.get("ok"):
        raise HTTPException(status_code=400, detail=result)
    return result


@app.delete("/ros/subscribe")
def unsubscribe_from_topic(
    topic: Annotated[str | None, Query()] = None,
    payload: Annotated[UnsubscribeRequest | None, Body()] = None,
) -> dict:
    selected_topic = topic or (payload.topic if payload else None)
    if not selected_topic:
        raise HTTPException(status_code=400, detail="topic is required")
    return ros_bridge.remove_subscription(selected_topic)


@app.post("/api/navigation/goal")
def publish_goal(payload: GoalPoseRequest) -> dict:
    try:
        return ros_bridge.publish_goal(
            x=payload.x,
            y=payload.y,
            yaw=payload.yaw,
            frame_id=payload.frame_id,
            topic=payload.topic,
        )
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.post("/api/navigation/initial_pose")
def publish_initial_pose(payload: InitialPoseRequest) -> dict:
    try:
        return ros_bridge.publish_initial_pose(
            x=payload.x,
            y=payload.y,
            yaw=payload.yaw,
            frame_id=payload.frame_id,
            covariance_xy=payload.covariance_xy,
            covariance_yaw=payload.covariance_yaw,
        )
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.post("/api/estop")
def set_estop(payload: EstopRequest) -> dict:
    command = "emergency_stop" if payload.active else "resume"
    try:
        return ros_bridge.send_bt_command(command=command, source=payload.source, target="")
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.websocket("/ws/telemetry")
async def ws_telemetry(websocket: WebSocket) -> None:
    def telemetry_packet() -> dict:
        data = ros_bridge.get_telemetry()
        return {
            "type": "telemetry",
            "topic": "/api/telemetry",
            "stamp": None,
            "data": data,
            "status": data["connection_status"],
        }

    await websocket_manager.stream(
        websocket,
        telemetry_packet,
        interval=0.5,
    )


async def _stream_channel(websocket: WebSocket, channel: str, *, interval: float = 0.5) -> None:
    await websocket_manager.stream(websocket, lambda: ros_bridge.get_channel_payload(channel), interval=interval)


@app.websocket("/ws/camera")
async def ws_camera(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "camera", interval=1.0 / 12.0)


@app.websocket("/ws/depth")
async def ws_depth(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "depth", interval=0.2)


@app.websocket("/ws/map")
async def ws_map(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "map", interval=0.5)


@app.websocket("/ws/path")
async def ws_path(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "path", interval=0.5)


@app.websocket("/ws/pointcloud")
async def ws_pointcloud(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "pointcloud", interval=0.5)


@app.websocket("/ws/markers")
async def ws_markers(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "markers", interval=1.0)


@app.websocket("/ws/rtabmap/status")
async def ws_rtabmap_status(websocket: WebSocket) -> None:
    await websocket_manager.stream(
        websocket,
        lambda: {
            "type": "rtabmap/status",
            "topic": "/api/rtabmap/status",
            "stamp": None,
            "data": ros_bridge.get_rtabmap_status(),
            "status": ros_bridge.get_rtabmap_status()["tracking_status"],
        },
        interval=1.0,
    )


@app.websocket("/ws/rtabmap/rgb")
async def ws_rtabmap_rgb(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "camera", interval=1.0 / 12.0)


@app.websocket("/ws/rtabmap/depth")
async def ws_rtabmap_depth(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "depth", interval=0.2)


@app.websocket("/ws/rtabmap/odom")
async def ws_rtabmap_odom(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "odom", interval=0.2)


@app.websocket("/ws/rtabmap/map")
async def ws_rtabmap_map(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "map", interval=0.5)


@app.websocket("/ws/rtabmap/cloud")
async def ws_rtabmap_cloud(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "pointcloud", interval=0.5)


@app.websocket("/ws/rtabmap/imu")
async def ws_rtabmap_imu(websocket: WebSocket) -> None:
    await _stream_channel(websocket, "imu", interval=0.3)


@app.websocket("/ws/topic/{topic_path:path}")
async def ws_topic(websocket: WebSocket, topic_path: str) -> None:
    topic = f"/{topic_path.lstrip('/')}"
    ros_bridge.ensure_subscription(topic)
    await websocket_manager.stream(websocket, lambda: ros_bridge.get_topic_payload(topic), interval=0.5)


@app.get("/api/stream-defaults")
def get_stream_defaults() -> dict:
    return {"topics": CHANNEL_PRIMARY_TOPICS}
