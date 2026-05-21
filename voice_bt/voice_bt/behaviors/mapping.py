"""Action leaves for starting and stopping RTAB-Map.

RTAB-Map sensor topics are read from the voice_bt_node ROS parameters at
runtime so that the same code works on both hardware (Azure Kinect /k4a/*
topics) and simulation (/depth_camera/* topics bridged from Gazebo).
"""

import subprocess
import time

import py_trees
from std_srvs.srv import Empty


# Hardware defaults used when ROS params are not set (e.g. hardware launch).
_RTABMAP_DEFAULTS = {
    "rtabmap_rgb_topic":          "/k4a/rgb/image_raw",
    "rtabmap_depth_topic":        "/k4a/depth_to_rgb/image_raw",
    "rtabmap_camera_info_topic":  "/k4a/rgb/camera_info",
    "rtabmap_scan_cloud_topic":   "/k4a/points2",
    "rtabmap_imu_topic":          "/k4a/imu_filtered",
}

_RTABMAP_FIXED_ARGS = (
    "--delete_db_on_start --Reg/Force3DoF true "
    "--Grid/FromDepth false --Grid/3D false --Grid/RangeMax 4.5 "
    "--Grid/MaxGroundHeight 0.10 --Grid/MaxObstacleHeight 1.20 "
    "--Grid/CellSize 0.05"
)


def _build_rtabmap_cmd(ros_node) -> list:
    """Build the ros2 launch command, reading sensor topics from node params."""
    def _get(param):
        try:
            return ros_node.get_parameter(param).value
        except Exception:
            return _RTABMAP_DEFAULTS.get(param, "")

    use_sim_time = False
    try:
        use_sim_time = ros_node.get_parameter("use_sim_time").value
    except Exception:
        pass

    return [
        "ros2", "launch", "rtabmap_launch", "rtabmap.launch.py",
        f"rtabmap_args:={_RTABMAP_FIXED_ARGS}",
        f"rgb_topic:={_get('rtabmap_rgb_topic')}",
        f"depth_topic:={_get('rtabmap_depth_topic')}",
        f"camera_info_topic:={_get('rtabmap_camera_info_topic')}",
        f"scan_cloud_topic:={_get('rtabmap_scan_cloud_topic')}",
        "subscribe_scan_cloud:=true",
        f"imu_topic:={_get('rtabmap_imu_topic')}",
        "wait_imu_to_init:=true",
        "frame_id:=base_footprint",
        "approx_sync:=true",
        "approx_sync_max_interval:=0.05",
        "wait_for_transform:=0.3",
        "queue_size:=20",
        "qos:=2",
        f"use_sim_time:={'true' if use_sim_time else 'false'}",
        "rviz:=false",
    ]


class StartMapping(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node, name: str = "StartMapping"):
        super().__init__(name)
        self._node = ros_node
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key("rtabmap_proc", access=py_trees.common.Access.WRITE)
        self._bb.register_key("is_mapping",   access=py_trees.common.Access.WRITE)

    def update(self):
        existing = self._bb.get("rtabmap_proc") if self._bb.exists("rtabmap_proc") else None
        if existing is not None and existing.poll() is None:
            self._node.get_logger().info("StartMapping: RTAB-Map already running")
            return py_trees.common.Status.SUCCESS
        try:
            cmd = _build_rtabmap_cmd(self._node)
            proc = subprocess.Popen(cmd)
            self._bb.set("rtabmap_proc", proc)
            self._bb.set("is_mapping", True)
            self._node.get_logger().info(f"StartMapping: spawned RTAB-Map pid={proc.pid}")
            return py_trees.common.Status.SUCCESS
        except FileNotFoundError as e:
            self._node.get_logger().error(f"StartMapping: ros2 launch not found: {e}")
            return py_trees.common.Status.FAILURE
        except Exception as e:
            self._node.get_logger().error(f"StartMapping: failed to spawn: {e}")
            return py_trees.common.Status.FAILURE


class StopMapping(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node,
                 save_service: str = "/rtabmap/publish_map",
                 name: str = "StopMapping"):
        super().__init__(name)
        self._node = ros_node
        self._save_service = save_service
        self._client = None
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key("rtabmap_proc", access=py_trees.common.Access.WRITE)
        self._bb.register_key("is_mapping",   access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self._client = self._node.create_client(Empty, self._save_service)

    def update(self):
        # Best-effort save: only if the service is live.
        if self._client is not None and self._client.service_is_ready():
            self._client.call_async(Empty.Request())
            time.sleep(1.0)  # give RTAB-Map a moment to persist before SIGTERM

        proc = self._bb.get("rtabmap_proc") if self._bb.exists("rtabmap_proc") else None
        if proc is not None and proc.poll() is None:
            self._node.get_logger().info(f"StopMapping: terminating pid={proc.pid}")
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()

        self._bb.set("rtabmap_proc", None)
        self._bb.set("is_mapping", False)
        return py_trees.common.Status.SUCCESS
