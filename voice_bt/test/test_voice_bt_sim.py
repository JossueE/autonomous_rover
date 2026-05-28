import sys
import os
import time
import math
import pytest
import threading
from unittest.mock import MagicMock, patch

# 1. Mock heavy audio/external dependencies before importing voice_bt to run cleanly in headless CI.
sys.modules['sounddevice'] = MagicMock()
sys.modules['vosk'] = MagicMock()

# Mock the entire voice_bt.tts module to prevent speech output attempts or calling aplay
import voice_bt.tts
voice_bt.tts.PiperTTS = MagicMock()

# Mock BilingualASR completely to avoid setting up real Vosk models
import voice_bt.asr_thread
voice_bt.asr_thread.BilingualASR = MagicMock()

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
import py_trees

# Override _pkg_asset to resolve to local workspace files rather than installed shared directory.
import voice_bt.bt_node

def mock_pkg_asset(*parts):
    test_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.join(test_dir, '..')
    if parts[0] == 'config':
        return os.path.join(pkg_root, *parts)
    else:
        return os.path.join(pkg_root, 'voice_assets', *parts)

voice_bt.bt_node._pkg_asset = mock_pkg_asset

from voice_bt.bt_node import VoiceBTNode


class MockSimNode(Node):
    """Mocks the simulation environment and other ROS 2 stack components.

    Captures velocity commands, waypoint parameter requests, and publishes
    sensor data (AMCL poses and Odometry).
    """

    def __init__(self):
        super().__init__("mock_sim_node")
        self.cmd_vels = []
        self.param_requests = []

        # Subscriptions
        self.create_subscription(Twist, "/cmd_vel_safe", self.cmd_vel_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(String, "/bt_inject_command", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.amcl_pub = self.create_publisher(PoseWithCovarianceStamped, "/amcl_robot_pose", 10)

        # Service
        self.srv = self.create_service(
            SetParameters, "/path_planning_node/set_parameters", self.set_params_cb
        )

    def cmd_vel_cb(self, msg):
        self.cmd_vels.append(msg)

    def set_params_cb(self, request, response):
        self.param_requests.append(request)
        from rcl_interfaces.msg import SetParametersResult
        res = SetParametersResult()
        res.successful = True
        response.results = [res]
        return response


@pytest.fixture(scope="module")
def ros_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def test_env(ros_init):
    # Set up ROS parameters or node overrides
    node = VoiceBTNode()
    sim = MockSimNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sim)

    stop_evt = threading.Event()
    def spin():
        while not stop_evt.is_set():
            executor.spin_once(timeout_sec=0.01)

    t = threading.Thread(target=spin)
    t.start()

    yield node, sim

    # Cleanup
    stop_evt.set()
    t.join()
    node.destroy_node()
    sim.destroy_node()


def wait_for(condition, timeout=3.0, interval=0.1):
    start = time.time()
    while time.time() - start < timeout:
        if condition():
            return True
        time.sleep(interval)
    return False


def test_initial_state(test_env):
    node, sim = test_env
    # Check that initial mode is IDLE
    bb = py_trees.blackboard.Client(name="test_client")
    bb.register_key("mode", access=py_trees.common.Access.READ)
    assert bb.get("mode") == "IDLE"


def test_teleop_voice_movements(test_env):
    node, sim = test_env
    bb = py_trees.blackboard.Client(name="test_client")
    bb.register_key("mode", access=py_trees.common.Access.READ)
    bb.register_key("command", access=py_trees.common.Access.WRITE)

    # 1. Switch to teleop voice mode
    sim.cmd_pub.publish(String(data="teleop_voice"))
    assert wait_for(lambda: bb.get("mode") == "TELEOP_VOICE"), f"Failed to switch to TELEOP_VOICE, current mode is {bb.get('mode')}"

    # 2. Inject "forward" command
    sim.cmd_vels.clear()
    sim.cmd_pub.publish(String(data="forward"))
    # Wait for non-zero twist velocity
    assert wait_for(lambda: len(sim.cmd_vels) > 0 and sim.cmd_vels[-1].linear.x > 0.0)
    assert sim.cmd_vels[-1].linear.x == 0.3
    assert sim.cmd_vels[-1].angular.z == 0.0

    # 3. Inject "turn_left" command
    sim.cmd_vels.clear()
    sim.cmd_pub.publish(String(data="turn_left"))
    assert wait_for(lambda: len(sim.cmd_vels) > 0 and sim.cmd_vels[-1].angular.z > 0.0)
    assert sim.cmd_vels[-1].linear.x == 0.0
    assert sim.cmd_vels[-1].angular.z == 0.5

    # 4. Inject "turn_right" command
    sim.cmd_vels.clear()
    sim.cmd_pub.publish(String(data="turn_right"))
    assert wait_for(lambda: len(sim.cmd_vels) > 0 and sim.cmd_vels[-1].angular.z < 0.0)
    assert sim.cmd_vels[-1].linear.x == 0.0
    assert sim.cmd_vels[-1].angular.z == -0.5

    # 5. Inject "stop" command
    sim.cmd_vels.clear()
    sim.cmd_pub.publish(String(data="stop"))
    assert wait_for(lambda: len(sim.cmd_vels) > 0 and sim.cmd_vels[-1].linear.x == 0.0 and sim.cmd_vels[-1].angular.z == 0.0)


def test_emergency_stop_and_resume(test_env):
    node, sim = test_env
    bb = py_trees.blackboard.Client(name="test_client")
    bb.register_key("mode", access=py_trees.common.Access.READ)

    # 1. Switch to teleop voice mode
    sim.cmd_pub.publish(String(data="teleop_voice"))
    assert wait_for(lambda: bb.get("mode") == "TELEOP_VOICE")

    # 2. Inject emergency stop
    sim.cmd_vels.clear()
    sim.cmd_pub.publish(String(data="emergency_stop"))
    assert wait_for(lambda: bb.get("mode") == "EMERGENCY")
    # Verify that Twist is zeroed immediately on emergency stop
    assert len(sim.cmd_vels) > 0
    assert sim.cmd_vels[-1].linear.x == 0.0
    assert sim.cmd_vels[-1].angular.z == 0.0

    # 3. Verify commands are discarded in EMERGENCY mode
    sim.cmd_vels.clear()
    sim.cmd_pub.publish(String(data="forward"))
    time.sleep(0.5)
    assert len(sim.cmd_vels) == 0  # No movement commands should be processed

    # 4. Inject "resume" command
    sim.cmd_pub.publish(String(data="resume"))
    assert wait_for(lambda: bb.get("mode") == "IDLE")


def test_autonomous_navigation(test_env):
    node, sim = test_env
    bb = py_trees.blackboard.Client(name="test_client")
    bb.register_key("mode", access=py_trees.common.Access.READ)
    bb.register_key("command", access=py_trees.common.Access.WRITE)
    bb.register_key("target_waypoint", access=py_trees.common.Access.WRITE)

    # 1. Inject navigation command to "cochera"
    # To simulate the ASR output atomically setting both waypoint and command
    bb.set("target_waypoint", "cochera")
    bb.set("command", "navigate")

    # 2. Verify we transition to AUTONOMOUS mode and set the OSM lanelet name.
    assert wait_for(lambda: bb.get("mode") == "AUTONOMOUS")
    assert wait_for(lambda: len(sim.param_requests) > 0)
    
    param = sim.param_requests[-1].parameters[0]
    assert param.name == "end_lanelet_name"
    assert param.value.string_value == "home"

    # 3. Simulate the robot reaching the target waypoint
    # Publish AMCL pose at the target location (cochera is at x=3.5, y=1.2)
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = "map"
    pose_msg.pose.pose.position.x = 3.5
    pose_msg.pose.pose.position.y = 1.2
    sim.amcl_pub.publish(pose_msg)

    # 4. Verify the BT finishes navigation, zeroes velocity, and transitions back to IDLE
    assert wait_for(lambda: bb.get("mode") == "IDLE")
    assert len(sim.cmd_vels) > 0
    assert sim.cmd_vels[-1].linear.x == 0.0
    assert sim.cmd_vels[-1].angular.z == 0.0


def test_patrol_mode(test_env):
    node, sim = test_env
    bb = py_trees.blackboard.Client(name="test_client")
    bb.register_key("mode", access=py_trees.common.Access.READ)
    bb.register_key("command", access=py_trees.common.Access.WRITE)
    bb.register_key("patrol_index", access=py_trees.common.Access.READ)

    # 1. Inject patrol command
    bb.set("command", "patrol")

    # 2. Verify we transition to PATROL mode
    assert wait_for(lambda: bb.get("mode") == "PATROL")
    assert bb.get("patrol_index") == 0

    # First waypoint in list (waypoints keys are: inicio, cochera, shell)
    # The list is sorted or retrieved as defined in load_waypoints.
    # Let's see the order. Waypoints in config/waypoints.yaml:
    # 1. inicio (osm_name: station1, x: 0.0, y: 0.0)
    # 2. cochera (osm_name: home, x: 3.5, y: 1.2)
    # 3. shell (osm_name: end, x: 7.0, y: 0.5)
    # So waypoint names list = ['inicio', 'cochera', 'shell']

    # Wait for the service parameter to be set for the first waypoint (inicio)
    assert wait_for(lambda: len(sim.param_requests) > 0)
    assert sim.param_requests[-1].parameters[0].value.string_value == "station1"

    # 3. Simulate reaching 'inicio' waypoint
    sim.param_requests.clear()
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = "map"
    pose_msg.pose.pose.position.x = 0.0
    pose_msg.pose.pose.position.y = 0.0
    sim.amcl_pub.publish(pose_msg)

    # Verify that patrol index increments to 1, and it sets the next OSM lanelet name.
    assert wait_for(lambda: bb.get("patrol_index") == 1)
    assert wait_for(lambda: len(sim.param_requests) > 0)
    assert sim.param_requests[-1].parameters[0].value.string_value == "home"

    # 4. Inject stop command to cancel patrol
    sim.cmd_pub.publish(String(data="stop"))
    assert wait_for(lambda: bb.get("mode") == "IDLE")


@patch("subprocess.Popen")
def test_mapping_transitions(mock_popen, test_env):
    node, sim = test_env
    bb = py_trees.blackboard.Client(name="test_client")
    bb.register_key("is_mapping", access=py_trees.common.Access.READ)

    mock_proc = MagicMock()
    mock_proc.poll.return_value = None
    mock_popen.return_value = mock_proc

    # 1. Inject start mapping
    sim.cmd_pub.publish(String(data="start_mapping"))
    assert wait_for(lambda: bb.get("is_mapping") is True)
    assert mock_popen.called

    # 2. Inject stop mapping
    sim.cmd_pub.publish(String(data="stop_mapping"))
    assert wait_for(lambda: bb.get("is_mapping") is False)
    assert mock_proc.terminate.called or mock_proc.kill.called
