from __future__ import annotations

from pydantic import BaseModel, Field


class SubscribeRequest(BaseModel):
    topic: str
    message_type: str | None = None


class UnsubscribeRequest(BaseModel):
    topic: str


class GoalPoseRequest(BaseModel):
    x: float
    y: float
    yaw: float
    frame_id: str = "map"
    topic: str = "/goal_pose"


class InitialPoseRequest(BaseModel):
    x: float
    y: float
    yaw: float
    frame_id: str = "map"
    covariance_xy: float = Field(default=0.25, ge=0.0)
    covariance_yaw: float = Field(default=0.0685, ge=0.0)


class EstopRequest(BaseModel):
    active: bool = True
    source: str = "gui"
