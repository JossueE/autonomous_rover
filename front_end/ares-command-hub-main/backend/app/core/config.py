from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TopicSpec:
    topic: str
    message_type: str
    channel: str


API_HOST = "127.0.0.1"
API_PORT = 8000
STALE_SECONDS = 3.0
POINTCLOUD_MAX_POINTS = 5000
IMAGE_MAX_WIDTH = 640
IMAGE_MAX_HEIGHT = 480
MAP_IMAGE_MAX_WIDTH = 700
MAP_IMAGE_MAX_HEIGHT = 700
JPEG_QUALITY = 50
HEAVY_TOPIC_MIN_INTERVALS = {
    "camera": 0.2,
    "depth": 0.2,
    "map": 0.5,
    "pointcloud": 0.5,
}

TELEMETRY_VIDEO_FALLBACK = f"http://{API_HOST}:{API_PORT}/static/demo.mp4"

DEFAULT_TOPICS: tuple[TopicSpec, ...] = (
    TopicSpec("/k4a/rgb/image_raw/compressed", "sensor_msgs/msg/CompressedImage", "camera"),
    TopicSpec("/k4a/rgb/camera_info", "sensor_msgs/msg/CameraInfo", "camera_info"),
    TopicSpec("/rtabmap/odom", "nav_msgs/msg/Odometry", "odom"),
    TopicSpec("/k4a/imu_filtered", "sensor_msgs/msg/Imu", "imu"),
    TopicSpec("/all_available_paths", "visualization_msgs/msg/MarkerArray", "markers"),
    TopicSpec("/scan", "sensor_msgs/msg/LaserScan", "scan"),
    TopicSpec("/goal_pose", "geometry_msgs/msg/PoseStamped", "goal"),
    TopicSpec("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", "initialpose"),
)

CHANNEL_PRIMARY_TOPICS = {
    "camera": "/k4a/rgb/image_raw/compressed",
    "depth": "/k4a/depth_to_rgb/image_raw",
    "camera_info": "/k4a/rgb/camera_info",
    "odom": "/rtabmap/odom",
    "imu": "/k4a/imu_filtered",
    "map": "/rtabmap/grid_prob_map",
    "path": "/sdv_trajectory",
    "pointcloud": "/k4a/points2",
    "markers": "/all_available_paths",
    "scan": "/scan",
    "goal": "/goal_pose",
    "initialpose": "/initialpose",
}

CHANNEL_FALLBACK_TOPICS = {
    "pointcloud": (
        "/points_rotated_notground",
        "/depth_camera/points",
        "/rtabmap/cloud_obstacles",
    ),
}

ALLOWED_GOAL_TOPICS = {"/goal_pose", "/goal"}
INITIAL_POSE_TOPIC = "/initialpose"
