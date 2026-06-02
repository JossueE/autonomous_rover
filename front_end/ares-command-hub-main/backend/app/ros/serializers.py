from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from app.core.config import POINTCLOUD_MAX_POINTS
from app.utils.geometry_utils import (
    quaternion_to_dict,
    quaternion_to_yaw,
    stamp_to_seconds,
    vector3_to_dict,
)


def _header_dict(msg: Any) -> dict[str, Any]:
    header = getattr(msg, "header", None)
    if header is None:
        return {}
    return {
        "frame_id": str(getattr(header, "frame_id", "")),
        "stamp": stamp_to_seconds(getattr(header, "stamp", None)),
    }


def _pose_dict(pose: Any) -> dict[str, Any]:
    return {
        "position": vector3_to_dict(pose.position),
        "orientation": quaternion_to_dict(pose.orientation),
        "yaw": quaternion_to_yaw(pose.orientation),
    }


def _point_dict(point: Any) -> dict[str, float]:
    return {
        "x": float(getattr(point, "x", 0.0)),
        "y": float(getattr(point, "y", 0.0)),
        "z": float(getattr(point, "z", 0.0)),
    }


def _color_dict(color: Any) -> dict[str, float]:
    return {
        "r": float(getattr(color, "r", 0.0)),
        "g": float(getattr(color, "g", 0.0)),
        "b": float(getattr(color, "b", 0.0)),
        "a": float(getattr(color, "a", 0.0)),
    }


def _sanitize_for_json(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _sanitize_for_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize_for_json(item) for item in value]
    if isinstance(value, bytes):
        return list(value[:256])
    if hasattr(value, "tolist"):
        return _sanitize_for_json(value.tolist())
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    return str(value)


def _fallback_to_dict(msg: Any) -> Any:
    try:
        from rosidl_runtime_py.convert import message_to_ordereddict

        return _sanitize_for_json(message_to_ordereddict(msg))
    except Exception:
        if hasattr(msg, "get_fields_and_field_types"):
            return {
                field: _fallback_to_dict(getattr(msg, field))
                for field in msg.get_fields_and_field_types()
            }
        if isinstance(msg, (list, tuple)):
            return [_fallback_to_dict(item) for item in msg]
        if isinstance(msg, (str, int, float, bool)) or msg is None:
            return msg
        return str(msg)


def serialize_odometry(msg: Any) -> dict[str, Any]:
    pose = msg.pose.pose
    twist = msg.twist.twist
    return {
        "header": _header_dict(msg),
        "position": vector3_to_dict(pose.position),
        "orientation": quaternion_to_dict(pose.orientation),
        "yaw": quaternion_to_yaw(pose.orientation),
        "linear_velocity": float(twist.linear.x),
        "angular_velocity": float(twist.angular.z),
        "twist": {
            "linear": vector3_to_dict(twist.linear),
            "angular": vector3_to_dict(twist.angular),
        },
    }


def serialize_imu(msg: Any) -> dict[str, Any]:
    return {
        "header": _header_dict(msg),
        "orientation": quaternion_to_dict(msg.orientation),
        "yaw": quaternion_to_yaw(msg.orientation),
        "angular_velocity": vector3_to_dict(msg.angular_velocity),
        "linear_acceleration": vector3_to_dict(msg.linear_acceleration),
    }


def serialize_camera_info(msg: Any) -> dict[str, Any]:
    k = list(getattr(msg, "k", []))
    return {
        "header": _header_dict(msg),
        "width": int(msg.width),
        "height": int(msg.height),
        "fx": float(k[0]) if len(k) > 0 else 0.0,
        "fy": float(k[4]) if len(k) > 4 else 0.0,
        "cx": float(k[2]) if len(k) > 2 else 0.0,
        "cy": float(k[5]) if len(k) > 5 else 0.0,
        "distortion_model": str(getattr(msg, "distortion_model", "")),
        "distortion": [float(value) for value in getattr(msg, "d", [])],
    }


def serialize_image(msg: Any, *, depth: bool = False) -> dict[str, Any]:
    from app.utils.image_utils import image_to_jpeg_payload

    return {
        "header": _header_dict(msg),
        **image_to_jpeg_payload(msg, depth=depth),
    }


def serialize_compressed_image(msg: Any) -> dict[str, Any]:
    from app.utils.image_utils import compressed_image_to_payload

    return {
        "header": _header_dict(msg),
        **compressed_image_to_payload(msg),
    }


def serialize_occupancy_grid(msg: Any) -> dict[str, Any]:
    image = None
    try:
        from app.utils.image_utils import occupancy_grid_to_image_payload

        image = occupancy_grid_to_image_payload(msg)
    except Exception as exc:
        image = {"error": str(exc)}

    return {
        "header": _header_dict(msg),
        "width": int(msg.info.width),
        "height": int(msg.info.height),
        "resolution": float(msg.info.resolution),
        "origin": _pose_dict(msg.info.origin),
        "data": list(msg.data),
        "image": image,
    }


def serialize_path(msg: Any) -> dict[str, Any]:
    poses = []
    for pose_stamped in msg.poses:
        pose = pose_stamped.pose
        poses.append(
            {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "yaw": quaternion_to_yaw(pose.orientation),
            }
        )
    return {
        "header": _header_dict(msg),
        "count": len(poses),
        "poses": poses,
    }


def serialize_pointcloud(msg: Any) -> dict[str, Any]:
    from app.utils.pointcloud_utils import pointcloud_to_points

    return {
        "header": _header_dict(msg),
        **pointcloud_to_points(msg, max_points=POINTCLOUD_MAX_POINTS),
    }


def serialize_laserscan(msg: Any) -> dict[str, Any]:
    valid = [float(value) for value in msg.ranges if math.isfinite(float(value))]
    return {
        "header": _header_dict(msg),
        "angle_min": float(msg.angle_min),
        "angle_max": float(msg.angle_max),
        "angle_increment": float(msg.angle_increment),
        "range_min": float(msg.range_min),
        "range_max": float(msg.range_max),
        "valid_count": len(valid),
        "min_range": min(valid) if valid else None,
        "max_range": max(valid) if valid else None,
        "sample": valid[:360],
    }


def serialize_marker(msg: Any) -> dict[str, Any]:
    return {
        "header": _header_dict(msg),
        "ns": str(getattr(msg, "ns", "")),
        "id": int(getattr(msg, "id", 0)),
        "type": int(getattr(msg, "type", 0)),
        "action": int(getattr(msg, "action", 0)),
        "pose": _pose_dict(msg.pose),
        "scale": vector3_to_dict(msg.scale),
        "color": _color_dict(msg.color),
        "points": [_point_dict(point) for point in getattr(msg, "points", [])],
        "text": str(getattr(msg, "text", "")),
    }


def serialize_marker_array(msg: Any) -> dict[str, Any]:
    markers = [serialize_marker(marker) for marker in msg.markers]
    return {
        "count": len(markers),
        "markers": markers,
    }


def serialize_message(topic: str, message_type: str, msg: Any) -> dict[str, Any]:
    try:
        if message_type == "nav_msgs/msg/Odometry":
            data = serialize_odometry(msg)
        elif message_type == "sensor_msgs/msg/Imu":
            data = serialize_imu(msg)
        elif message_type == "sensor_msgs/msg/CameraInfo":
            data = serialize_camera_info(msg)
        elif message_type == "sensor_msgs/msg/Image":
            encoding = str(getattr(msg, "encoding", ""))
            data = serialize_image(
                msg,
                depth="depth" in topic.lower() or "16UC1" in encoding or "32FC1" in encoding,
            )
        elif message_type == "sensor_msgs/msg/CompressedImage":
            data = serialize_compressed_image(msg)
        elif message_type == "nav_msgs/msg/OccupancyGrid":
            data = serialize_occupancy_grid(msg)
        elif message_type == "nav_msgs/msg/Path":
            data = serialize_path(msg)
        elif message_type == "sensor_msgs/msg/PointCloud2":
            data = serialize_pointcloud(msg)
        elif message_type == "sensor_msgs/msg/LaserScan":
            data = serialize_laserscan(msg)
        elif message_type == "visualization_msgs/msg/Marker":
            data = serialize_marker(msg)
        elif message_type == "visualization_msgs/msg/MarkerArray":
            data = serialize_marker_array(msg)
        else:
            data = _fallback_to_dict(msg)
        status = "online"
        error = None
    except Exception as exc:
        data = {"error": str(exc)}
        status = "degraded"
        error = str(exc)

    return {
        "type": message_type,
        "topic": topic,
        "stamp": _header_dict(msg).get("stamp"),
        "data": data,
        "status": status,
        "error": error,
    }
