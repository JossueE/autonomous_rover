from __future__ import annotations

import threading
import time
import os
from pathlib import Path
from typing import Any

from app.core.config import (
    ALLOWED_GOAL_TOPICS,
    CHANNEL_FALLBACK_TOPICS,
    CHANNEL_PRIMARY_TOPICS,
    DEFAULT_TOPICS,
    HEAVY_TOPIC_MIN_INTERVALS,
    INITIAL_POSE_TOPIC,
    STALE_SECONDS,
    TELEMETRY_VIDEO_FALLBACK,
)
from app.ros.serializers import serialize_message
from app.utils.geometry_utils import yaw_to_quaternion

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    ROS_IMPORT_ERROR: str | None = None
except Exception as exc:  # pragma: no cover - depends on ROS environment
    rclpy = None
    MultiThreadedExecutor = None
    QoSProfile = None
    qos_profile_sensor_data = None
    get_message = None
    ROS_IMPORT_ERROR = str(exc)


def _normalize_topic(topic: str) -> str:
    topic = topic.strip()
    if not topic:
        raise ValueError("topic is required")
    return topic if topic.startswith("/") else f"/{topic}"


class RosBridgeManager:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._node = None
        self._executor = None
        self._thread: threading.Thread | None = None
        self._subscriptions: dict[str, Any] = {}
        self._publishers: dict[tuple[str, str], Any] = {}
        self._clients: dict[tuple[str, str], Any] = {}
        self._latest: dict[str, dict[str, Any]] = {}
        self._last_seen: dict[str, float] = {}
        self._last_serialized: dict[str, float] = {}
        self._topic_types: dict[str, str] = {spec.topic: spec.message_type for spec in DEFAULT_TOPICS}
        self._topic_channels: dict[str, str] = {spec.topic: spec.channel for spec in DEFAULT_TOPICS}
        self._started = False
        self._error: str | None = ROS_IMPORT_ERROR

    @property
    def ros_available(self) -> bool:
        return ROS_IMPORT_ERROR is None and self._node is not None

    def start(self) -> None:
        if self._started:
            return
        self._started = True

        if ROS_IMPORT_ERROR is not None:
            self._error = ROS_IMPORT_ERROR
            return

        try:
            os.environ.setdefault("ROS_LOG_DIR", "/tmp/roslog")
            Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = rclpy.create_node("ares_command_hub_bridge")
            self._executor = MultiThreadedExecutor(num_threads=4)
            self._executor.add_node(self._node)
            for spec in DEFAULT_TOPICS:
                self.ensure_subscription(spec.topic, spec.message_type)
            self._thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._thread.start()
            self._error = None
        except Exception as exc:
            self._error = str(exc)

    def shutdown(self) -> None:
        executor = None
        node = None
        thread = None
        with self._lock:
            executor = self._executor
            node = self._node
            thread = self._thread
            self._executor = None
            self._node = None
            self._thread = None
            self._subscriptions.clear()
            self._publishers.clear()
            self._clients.clear()
            self._last_serialized.clear()
            self._started = False

        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except TypeError:
                executor.shutdown()
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()

    def _qos_for(self, message_type: str) -> Any:
        if message_type in {
            "sensor_msgs/msg/Image",
            "sensor_msgs/msg/CompressedImage",
            "sensor_msgs/msg/PointCloud2",
        }:
            profile = QoSProfile(depth=1)
            profile.reliability = qos_profile_sensor_data.reliability
            profile.durability = qos_profile_sensor_data.durability
            profile.history = qos_profile_sensor_data.history
            return profile
        if message_type.startswith("sensor_msgs/msg/") or message_type == "nav_msgs/msg/Odometry":
            profile = QoSProfile(depth=5)
            profile.reliability = qos_profile_sensor_data.reliability
            profile.durability = qos_profile_sensor_data.durability
            profile.history = qos_profile_sensor_data.history
            return profile
        return QoSProfile(depth=10)

    def _message_type_for_topic(self, topic: str) -> str | None:
        topic = _normalize_topic(topic)
        if topic in self._topic_types:
            return self._topic_types[topic]
        if self._node is None:
            return None
        for name, types in self._node.get_topic_names_and_types():
            if name == topic and types:
                self._topic_types[topic] = types[0]
                return types[0]
        return None

    def ensure_subscription(self, topic: str, message_type: str | None = None) -> dict[str, Any]:
        topic = _normalize_topic(topic)
        with self._lock:
            if topic in self._subscriptions:
                return {"ok": True, "topic": topic, "message_type": self._topic_types.get(topic)}

            if self._node is None or get_message is None:
                return {"ok": False, "topic": topic, "error": self._error or "ROS2 is not available"}

            resolved_type = message_type or self._message_type_for_topic(topic)
            if not resolved_type:
                return {"ok": False, "topic": topic, "error": "topic type is unknown"}

            try:
                message_class = get_message(resolved_type)
                subscription = self._node.create_subscription(
                    message_class,
                    topic,
                    lambda msg, t=topic, mt=resolved_type: self._on_message(t, mt, msg),
                    self._qos_for(resolved_type),
                )
            except Exception as exc:
                return {"ok": False, "topic": topic, "message_type": resolved_type, "error": str(exc)}

            self._subscriptions[topic] = subscription
            self._topic_types[topic] = resolved_type
            return {"ok": True, "topic": topic, "message_type": resolved_type}

    def remove_subscription(self, topic: str) -> dict[str, Any]:
        topic = _normalize_topic(topic)
        with self._lock:
            subscription = self._subscriptions.pop(topic, None)
            if subscription is None:
                return {"ok": True, "topic": topic, "removed": False}
            if self._node is not None:
                self._node.destroy_subscription(subscription)
            return {"ok": True, "topic": topic, "removed": True}

    def _min_interval_for(self, topic: str, message_type: str) -> float:
        channel = self._topic_channels.get(topic)
        if channel in HEAVY_TOPIC_MIN_INTERVALS:
            return HEAVY_TOPIC_MIN_INTERVALS[channel]
        if message_type in {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}:
            return HEAVY_TOPIC_MIN_INTERVALS["depth" if "depth" in topic.lower() else "camera"]
        if message_type == "sensor_msgs/msg/PointCloud2":
            return HEAVY_TOPIC_MIN_INTERVALS["pointcloud"]
        if message_type == "nav_msgs/msg/OccupancyGrid":
            return HEAVY_TOPIC_MIN_INTERVALS["map"]
        return 0.0

    def _on_message(self, topic: str, message_type: str, msg: Any) -> None:
        now = time.time()
        min_interval = self._min_interval_for(topic, message_type)
        with self._lock:
            self._last_seen[topic] = now
            last_serialized = self._last_serialized.get(topic)
            if last_serialized is not None and now - last_serialized < min_interval:
                return
            self._last_serialized[topic] = now

        packet = serialize_message(topic, message_type, msg)
        with self._lock:
            self._latest[topic] = packet

    def _fresh(self, topic: str, timeout: float = STALE_SECONDS) -> bool:
        with self._lock:
            last_seen = self._last_seen.get(topic)
        return last_seen is not None and time.time() - last_seen <= timeout

    def _best_topic(self, channel: str) -> str:
        primary = CHANNEL_PRIMARY_TOPICS[channel]
        candidates = (primary, *CHANNEL_FALLBACK_TOPICS.get(channel, ()))
        with self._lock:
            available = [(self._last_seen.get(topic, 0.0), topic) for topic in candidates]
        available.sort(reverse=True)
        return available[0][1] if available else primary

    def get_topic_payload(self, topic: str) -> dict[str, Any]:
        topic = _normalize_topic(topic)
        now = time.time()
        with self._lock:
            packet = self._latest.get(topic)
            message_type = self._topic_types.get(topic)
            last_seen = self._last_seen.get(topic)
            last_serialized = self._last_serialized.get(topic)
            subscribed = topic in self._subscriptions
        last_seen_age = now - last_seen if last_seen else None
        last_serialized_age = now - last_serialized if last_serialized else None
        stale = last_seen_age is None or last_seen_age > STALE_SECONDS
        if packet:
            packet = dict(packet)
            packet["message_type"] = message_type
            packet["subscribed"] = subscribed
            packet["last_seen"] = last_seen
            packet["last_seen_age"] = last_seen_age
            packet["last_serialized"] = last_serialized
            packet["last_serialized_age"] = last_serialized_age
            packet["stale"] = stale
            if stale:
                packet["status"] = "offline"
            return packet
        return {
            "type": message_type or "unknown",
            "topic": topic,
            "stamp": None,
            "data": None,
            "status": "offline",
            "message_type": message_type,
            "subscribed": subscribed,
            "last_seen": last_seen,
            "last_seen_age": last_seen_age,
            "last_serialized": last_serialized,
            "last_serialized_age": last_serialized_age,
            "stale": True,
            "error": None if self.ros_available else self._error,
        }

    def get_channel_payload(self, channel: str) -> dict[str, Any]:
        topic = self._best_topic(channel)
        self.ensure_subscription(topic)
        payload = self.get_topic_payload(topic)
        payload["channel"] = channel
        return payload

    def list_topics(self) -> dict[str, Any]:
        discovered: dict[str, list[str]] = {}
        if self._node is not None:
            try:
                discovered = {name: types for name, types in self._node.get_topic_names_and_types()}
            except Exception as exc:
                self._error = str(exc)

        with self._lock:
            for topic, message_type in self._topic_types.items():
                discovered.setdefault(topic, [message_type])
            topics = [
                {
                    "name": name,
                    "types": types,
                    "subscribed": name in self._subscriptions,
                    "status": "online" if self._fresh(name) else "offline",
                    "channel": self._topic_channels.get(name),
                    "last_seen": self._last_seen.get(name),
                }
                for name, types in sorted(discovered.items())
            ]
        return {"ros_available": self.ros_available, "error": self._error, "topics": topics}

    def list_nodes(self) -> dict[str, Any]:
        if self._node is None:
            return {"ros_available": False, "error": self._error, "nodes": []}
        try:
            nodes = [
                {"name": name, "namespace": namespace}
                for name, namespace in self._node.get_node_names_and_namespaces()
            ]
            return {"ros_available": True, "nodes": nodes}
        except Exception as exc:
            return {"ros_available": True, "error": str(exc), "nodes": []}

    def get_telemetry(self) -> dict[str, Any]:
        odom_topic = self._best_topic("odom")
        odom_packet = self.get_topic_payload(odom_topic)
        odom_data = odom_packet.get("data") or {}
        odom_online = odom_packet.get("status") == "online" and self._fresh(odom_topic)
        camera_online = self._fresh(self._best_topic("camera"))

        linear_velocity = float(odom_data.get("linear_velocity", 0.0))
        angular_velocity = float(odom_data.get("angular_velocity", 0.0))
        position = odom_data.get("position") or {}

        if not self.ros_available:
            connection_status = "offline"
        elif odom_online:
            connection_status = "connected"
        else:
            connection_status = "degraded"

        return {
            "camera_stream_url": TELEMETRY_VIDEO_FALLBACK,
            "camera_status": "online" if camera_online else "offline",
            "linear_velocity": round(linear_velocity, 3),
            "angular_velocity": round(angular_velocity, 3),
            "odom_x": round(float(position.get("x", 0.0)), 3),
            "odom_y": round(float(position.get("y", 0.0)), 3),
            "odom_yaw": round(float(odom_data.get("yaw", 0.0)), 3),
            "odometry_status": "online" if odom_online else "offline",
            "velocity_status": "online" if odom_online else "offline",
            "connection_status": connection_status,
            "last_update": time.strftime("%H:%M:%S"),
        }

    def get_rtabmap_status(self) -> dict[str, Any]:
        rgb_topic = self._best_topic("camera")
        depth_topic = self._best_topic("depth")
        odom_topic = self._best_topic("odom")
        map_topic = self._best_topic("map")
        pointcloud_topic = self._best_topic("pointcloud")
        imu_topic = self._best_topic("imu")

        odom_online = self._fresh(odom_topic)
        map_online = self._fresh(map_topic, timeout=10.0)
        rgb_online = self._fresh(rgb_topic)
        depth_online = self._fresh(depth_topic)
        cloud_online = self._fresh(pointcloud_topic)
        imu_online = self._fresh(imu_topic)

        if odom_online and (map_online or rgb_online or depth_online):
            tracking_status = "ok"
        elif self.ros_available:
            tracking_status = "degraded"
        else:
            tracking_status = "offline"

        return {
            "rtabmap_online": map_online or odom_online,
            "localization_mode": None,
            "visual_odometry": odom_online,
            "icp_odometry": cloud_online,
            "tracking_status": tracking_status,
            "frame_id": "base_footprint",
            "rgb_topic": rgb_topic,
            "depth_topic": depth_topic,
            "pointcloud_topic": pointcloud_topic,
            "imu_topic": imu_topic,
            "camera_info_topic": CHANNEL_PRIMARY_TOPICS["camera_info"],
            "last_odom_update": self._last_seen.get(odom_topic),
            "last_map_update": self._last_seen.get(map_topic),
            "sources": {
                "rgb": "online" if rgb_online else "offline",
                "depth": "online" if depth_online else "offline",
                "map": "online" if map_online else "offline",
                "pointcloud": "online" if cloud_online else "offline",
                "imu": "online" if imu_online else "offline",
            },
        }

    def health(self) -> dict[str, Any]:
        return {
            "ok": self.ros_available,
            "ros_available": self.ros_available,
            "error": self._error,
            "subscriptions": sorted(self._subscriptions.keys()),
            "time": time.strftime("%H:%M:%S"),
        }

    def _publisher(self, topic: str, message_type: str) -> Any:
        key = (topic, message_type)
        if key in self._publishers:
            return self._publishers[key]
        if self._node is None or get_message is None:
            raise RuntimeError(self._error or "ROS2 is not available")
        message_class = get_message(message_type)
        publisher = self._node.create_publisher(message_class, topic, 10)
        self._publishers[key] = publisher
        return publisher

    def _client(self, service: str, service_class: Any) -> Any:
        key = (service, service_class.__name__)
        if key in self._clients:
            return self._clients[key]
        if self._node is None:
            raise RuntimeError(self._error or "ROS2 is not available")
        client = self._node.create_client(service_class, service)
        self._clients[key] = client
        return client

    def _bt_priority(self, *, command: str, source: str) -> int:
        if command == "emergency_stop":
            return 0
        if source == "joycon":
            return 1
        if source == "voice":
            return 2
        if source == "gui":
            return 3
        return 4

    def _publish_bt_command(self, *, command: str, source: str, target: str) -> dict[str, Any]:
        from rover_bt.msg import Command

        if self._node is None:
            raise RuntimeError(self._error or "ROS2 is not available")

        msg = Command()
        msg.stamp = self._node.get_clock().now().to_msg()
        msg.source = source
        msg.command = command
        msg.target = target
        msg.priority = self._bt_priority(command=command, source=source)
        self._publisher("/rover_bt/commands", "rover_bt/msg/Command").publish(msg)
        return {
            "ok": True,
            "transport": "topic",
            "topic": "/rover_bt/commands",
            "command": command,
            "source": source,
            "target": target,
            "priority": int(msg.priority),
        }

    def send_bt_command(
        self,
        *,
        command: str,
        source: str = "gui",
        target: str = "",
        timeout_sec: float = 1.0,
    ) -> dict[str, Any]:
        if self._node is None:
            raise RuntimeError(self._error or "ROS2 is not available")

        service_error = None
        try:
            from rover_bt.srv import SendCommand

            client = self._client("/rover_bt/send_command", SendCommand)
            if client.wait_for_service(timeout_sec=0.25):
                request = SendCommand.Request()
                request.source = source
                request.command = command
                request.target = target
                future = client.call_async(request)
                deadline = time.time() + timeout_sec
                while time.time() < deadline:
                    if future.done():
                        result = future.result()
                        return {
                            "ok": bool(result.accepted),
                            "transport": "service",
                            "service": "/rover_bt/send_command",
                            "command": command,
                            "source": source,
                            "target": target,
                            "message": str(result.message),
                        }
                    time.sleep(0.01)
                service_error = "timeout waiting for /rover_bt/send_command response"
            else:
                service_error = "/rover_bt/send_command is not available"
        except Exception as exc:
            service_error = str(exc)

        result = self._publish_bt_command(command=command, source=source, target=target)
        result["service_error"] = service_error
        return result

    def publish_goal(self, *, x: float, y: float, yaw: float, frame_id: str, topic: str) -> dict[str, Any]:
        topic = _normalize_topic(topic)
        if topic not in ALLOWED_GOAL_TOPICS:
            raise ValueError(f"goal topic must be one of {sorted(ALLOWED_GOAL_TOPICS)}")
        if self._node is None:
            raise RuntimeError(self._error or "ROS2 is not available")
        from geometry_msgs.msg import PoseStamped

        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._publisher(topic, "geometry_msgs/msg/PoseStamped").publish(msg)
        return {"ok": True, "topic": topic}

    def publish_initial_pose(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        frame_id: str,
        covariance_xy: float,
        covariance_yaw: float,
    ) -> dict[str, Any]:
        if self._node is None:
            raise RuntimeError(self._error or "ROS2 is not available")
        from geometry_msgs.msg import PoseWithCovarianceStamped

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = covariance_xy
        msg.pose.covariance[7] = covariance_xy
        msg.pose.covariance[35] = covariance_yaw
        self._publisher(INITIAL_POSE_TOPIC, "geometry_msgs/msg/PoseWithCovarianceStamped").publish(msg)
        return {"ok": True, "topic": INITIAL_POSE_TOPIC}


ros_bridge = RosBridgeManager()
