"""Voice-commanded behavior tree node.

Builds the BT, owns ROS publishers/subscribers, and starts the Vosk ASR
thread. Ticks the tree at a fixed rate.
"""

import os
import time

import rclpy
import yaml
from rclpy.node import Node

import py_trees
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from voice_bt.tts import PiperTTS
from voice_bt.asr_thread import BilingualASR
from voice_bt.behaviors.conditions import (
    CheckCommand, CheckMode, CheckFlag, CheckGoalReached,
)
from voice_bt.behaviors.state import (
    SetMode, ClearCommand, IncrementPatrolIndex, SetPatrolIndex,
)
from voice_bt.behaviors.motion import ZeroTwist, MoveRover, NavigateTo
from voice_bt.behaviors.speech import Speak, SpeakStatus, SpeakWaypointList
from voice_bt.behaviors.mapping import StartMapping, StopMapping


# All blackboard keys live in the global namespace (no prefix) so leaves and
# the node see the same flat keyspace.
BB_KEYS = [
    ("command", py_trees.common.Access.WRITE),
    ("mode", py_trees.common.Access.WRITE),
    ("is_mapping", py_trees.common.Access.WRITE),
    ("target_waypoint", py_trees.common.Access.WRITE),
    ("patrol_index", py_trees.common.Access.WRITE),
    ("robot_pose", py_trees.common.Access.WRITE),
    ("pose_source", py_trees.common.Access.WRITE),
    ("rtabmap_proc", py_trees.common.Access.WRITE),
]

MAP_POSE_TIMEOUT = 2.0  # seconds without /amcl_robot_pose → fall back to /odom


def load_waypoints(path):
    if not path or not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data.get("waypoints", {}) if data else {}


def _pkg_asset(*parts):
    """Return absolute path to a file in the installed voice_bt share directory."""
    from ament_index_python.packages import get_package_share_directory
    try:
        share = get_package_share_directory("voice_bt")
    except Exception:
        share = os.path.join(os.path.dirname(__file__), '..', '..', 'voice_assets')
    return os.path.join(share, *parts)


class VoiceBTNode(Node):
    def __init__(self):
        super().__init__("voice_bt_node")

        # Default paths resolve to the installed package share directory so any
        # teammate who runs `colcon build` gets working defaults automatically.
        self.declare_parameter(
            "vosk_model_es", _pkg_asset("voice_assets", "model_es"))
        self.declare_parameter(
            "vosk_model_en", _pkg_asset("voice_assets", "model_en"))
        self.declare_parameter(
            "piper_bin", _pkg_asset("voice_assets", "piper", "piper"))
        self.declare_parameter(
            "piper_model", _pkg_asset("voice_assets", "es_MX-claude-high.onnx"))
        self.declare_parameter(
            "waypoints_file", _pkg_asset("config", "waypoints.yaml"))
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("amcl_pose_topic", "/amcl_robot_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("bt_tick_rate", 10.0)

        # RTAB-Map sensor topic params — overridable for simulation.
        # Defaults are the hardware Azure Kinect topic names.
        self.declare_parameter("rtabmap_rgb_topic",          "/k4a/rgb/image_raw")
        self.declare_parameter("rtabmap_depth_topic",        "/k4a/depth_to_rgb/image_raw")
        self.declare_parameter("rtabmap_camera_info_topic",  "/k4a/rgb/camera_info")
        self.declare_parameter("rtabmap_scan_cloud_topic",   "/k4a/points2")
        self.declare_parameter("rtabmap_imu_topic",          "/k4a/imu_filtered")

        waypoints_file = self.get_parameter("waypoints_file").value
        self._waypoints = load_waypoints(waypoints_file)
        self.get_logger().info(f"Loaded {len(self._waypoints)} waypoints from {waypoints_file}")

        # ─── ROS interfaces ───
        self._cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self._last_amcl_time = 0.0
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("amcl_pose_topic").value,
            self._on_amcl_pose, 10)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self._on_odom, 10)
        self.create_subscription(
            String, "/bt_inject_command",
            self._on_inject_command, 10)
        self.get_logger().info("Listening for injected BT commands on /bt_inject_command")

        # ─── TTS ───
        self._tts = PiperTTS(
            self.get_logger(),
            self.get_parameter("piper_bin").value,
            self.get_parameter("piper_model").value,
        )

        # ─── Blackboard ───
        bb = py_trees.blackboard.Client(name="bt_node")
        for key, access in BB_KEYS:
            bb.register_key(key=key, access=access)
        bb.set("command", None)
        bb.set("mode", "IDLE")
        bb.set("is_mapping", False)
        bb.set("target_waypoint", None)
        bb.set("patrol_index", 0)
        bb.set("robot_pose", None)
        bb.set("pose_source", None)
        bb.set("rtabmap_proc", None)
        self._bb = bb

        # ─── Tree ───
        self._root = self._build_tree()
        self._tree = py_trees.trees.BehaviourTree(self._root)
        for behaviour in self._root.iterate():
            try:
                behaviour.setup(node=self)
            except TypeError:
                pass

        # ─── ASR thread ───
        self._asr = BilingualASR(
            logger=self.get_logger(),
            blackboard=_BBWriter(),
            es_model_path=self.get_parameter("vosk_model_es").value,
            en_model_path=self.get_parameter("vosk_model_en").value,
            waypoint_names=list(self._waypoints.keys()),
        )
        self._asr.start()

        # ─── Tick timer ───
        rate = float(self.get_parameter("bt_tick_rate").value)
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"Voice BT running at {rate} Hz")
        self._tts.speak("Sistema de control por voz listo.")

    # ─── Subscription callbacks ──────────────────────────────────────────

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self._bb.set("robot_pose",
                     (msg.pose.pose.position.x, msg.pose.pose.position.y))
        self._bb.set("pose_source", "map")
        self._last_amcl_time = time.monotonic()

    def _on_odom(self, msg: Odometry):
        if time.monotonic() - self._last_amcl_time > MAP_POSE_TIMEOUT:
            self._bb.set("robot_pose",
                         (msg.pose.pose.position.x, msg.pose.pose.position.y))
            self._bb.set("pose_source", "odom")

    def _on_inject_command(self, msg: String):
        """Accept a command string from /bt_inject_command for scripted testing."""
        if cmd := msg.data.strip().lower():
            self._bb.set("command", cmd)
            self.get_logger().info(f"BT command injected: '{cmd}'")

    # ─── BT tick ─────────────────────────────────────────────────────────

    def _tick(self):
        try:
            self._tree.tick()
        except Exception as e:
            self.get_logger().error(f"BT tick error: {e}")

    # ─── BT construction ─────────────────────────────────────────────────

    def _build_tree(self):
        wp = self._waypoints
        wp_names = list(wp.keys())
        pub = self._cmd_vel_pub
        tts = self._tts

        def seq(name, children, memory=True):
            return py_trees.composites.Sequence(name=name, memory=memory, children=children)

        def sel(name, children, memory=False):
            return py_trees.composites.Selector(name=name, memory=memory, children=children)

        def status_seq():
            return seq("StatusCmd", [
                CheckCommand("status"),
                ClearCommand(),
                SpeakStatus(tts),
            ])

        # ---- Emergency stop subtree (highest priority) ----
        emergency = sel("EmergencyGate", [
            py_trees.decorators.Inverter(
                name="NoEmergencyCmd", child=CheckCommand("emergency_stop")),
            seq("DoEmergency", [
                ZeroTwist(pub),
                SetMode("EMERGENCY"),
                Speak(tts, "Parada de emergencia"),
                ClearCommand(),
            ]),
        ])

        # ---- Mapping overlay (orthogonal to movement mode) ----
        mapping_overlay = sel("MappingOverlay", [
            seq("StartMappingSeq", [
                CheckCommand("start_mapping"),
                py_trees.decorators.Inverter(
                    name="NotAlreadyMapping", child=CheckFlag("is_mapping")),
                StartMapping(self),
                Speak(tts, "Iniciando mapeo. Muévete por el área."),
                ClearCommand(),
            ]),
            seq("StopMappingSeq", [
                CheckCommand("stop_mapping"),
                CheckFlag("is_mapping"),
                StopMapping(self),
                Speak(tts, "Mapa guardado."),
                ClearCommand(),
            ]),
            py_trees.behaviours.Success(name="MappingPassthrough"),
        ])

        # ---- Movement modes ----
        emergency_mode = seq("EmergencyMode", [
            CheckMode("EMERGENCY"),
            sel("EmergencyChoice", [
                seq("ResumeSeq", [
                    CheckCommand("resume"),
                    SetMode("IDLE"),
                    Speak(tts, "Operación reanudada"),
                    ClearCommand(),
                ]),
                ClearCommand(name="DiscardInEmergency"),
            ]),
        ])

        teleop_voice_mode = seq("TeleopVoiceMode", [
            CheckMode("TELEOP_VOICE"),
            sel("TeleopVoiceChoice", [
                seq("ExitToAuto", [
                    CheckCommand("autonomous"),
                    ZeroTwist(pub),
                    SetMode("IDLE"),
                    Speak(tts, "Modo autónomo"),
                    ClearCommand(),
                ]),
                seq("SwitchToJoycon", [
                    CheckCommand("teleop_joycon"),
                    ZeroTwist(pub),
                    SetMode("TELEOP_JOYCON"),
                    Speak(tts, "Modo joycon. Usa el control."),
                    ClearCommand(),
                ]),
                seq("StopV", [
                    CheckCommand("stop"),
                    ZeroTwist(pub),
                    Speak(tts, "Detenido"),
                    ClearCommand(),
                ]),
                # For directional moves: announce immediately, then move.
                # memory=True makes the sequence resume at MoveRover on later ticks.
                seq("ForwardV", [
                    CheckCommand("forward"),
                    Speak(tts, "Avanzando"),
                    ClearCommand(),
                    MoveRover(pub, linear=0.3, angular=0.0, duration=2.0),
                ], memory=True),
                seq("BackwardV", [
                    CheckCommand("backward"),
                    Speak(tts, "Retrocediendo"),
                    ClearCommand(),
                    MoveRover(pub, linear=-0.3, angular=0.0, duration=2.0),
                ], memory=True),
                seq("LeftV", [
                    CheckCommand("turn_left"),
                    Speak(tts, "Izquierda"),
                    ClearCommand(),
                    MoveRover(pub, linear=0.0, angular=0.5, duration=1.5),
                ], memory=True),
                seq("RightV", [
                    CheckCommand("turn_right"),
                    Speak(tts, "Derecha"),
                    ClearCommand(),
                    MoveRover(pub, linear=0.0, angular=-0.5, duration=1.5),
                ], memory=True),
                status_seq(),
                ClearCommand(name="DiscardInTeleopV"),
            ]),
        ])

        teleop_joycon_mode = seq("TeleopJoyconMode", [
            CheckMode("TELEOP_JOYCON"),
            sel("TeleopJoyconChoice", [
                seq("JoyExitAuto", [
                    CheckCommand("autonomous"),
                    SetMode("IDLE"),
                    Speak(tts, "Modo autónomo"),
                    ClearCommand(),
                ]),
                seq("JoyToVoice", [
                    CheckCommand("teleop_voice"),
                    SetMode("TELEOP_VOICE"),
                    Speak(tts, "Modo voz. Di avanza, retrocede, izquierda, derecha o detente."),
                    ClearCommand(),
                ]),
                status_seq(),
                ClearCommand(name="DiscardInJoycon"),
            ]),
        ])

        autonomous_mode = seq("AutonomousMode", [
            CheckMode("AUTONOMOUS"),
            sel("AutoChoice", [
                seq("GoalReached", [
                    CheckGoalReached(wp, tolerance=0.3),
                    ZeroTwist(pub),
                    Speak(tts, "Llegué a {waypoint}"),
                    SetMode("IDLE"),
                ]),
                seq("CancelNav", [
                    CheckCommand("stop"),
                    ZeroTwist(pub),
                    SetMode("IDLE"),
                    Speak(tts, "Navegación cancelada"),
                    ClearCommand(),
                ]),
                status_seq(),
                ClearCommand(name="DiscardInAuto"),
            ]),
        ])

        patrol_mode = seq("PatrolMode", [
            CheckMode("PATROL"),
            sel("PatrolChoice", [
                seq("PatrolStop", [
                    CheckCommand("stop"),
                    ZeroTwist(pub),
                    SetMode("IDLE"),
                    Speak(tts, "Patrulla detenida"),
                    ClearCommand(),
                ]),
                seq("PatrolAdvance", [
                    CheckGoalReached(wp, tolerance=0.3),
                    IncrementPatrolIndex(wp_names),
                    NavigateTo(self, wp),
                    Speak(tts, "Siguiente punto: {waypoint}"),
                ]),
                status_seq(),
                ClearCommand(name="DiscardInPatrol"),
            ]),
        ])

        idle_mode = seq("IdleMode", [
            CheckMode("IDLE"),
            sel("IdleChoice", [
                seq("StartNav", [
                    CheckCommand("navigate"),
                    NavigateTo(self, wp),
                    SetMode("AUTONOMOUS"),
                    Speak(tts, "Navegando a {waypoint}"),
                    ClearCommand(),
                ]),
                seq("StartPatrol", [
                    CheckCommand("patrol"),
                    SetPatrolIndex(wp_names, 0),
                    NavigateTo(self, wp),
                    SetMode("PATROL"),
                    Speak(tts, "Iniciando patrulla"),
                    ClearCommand(),
                ]),
                seq("EnterTeleopVoice", [
                    CheckCommand("teleop_voice"),
                    SetMode("TELEOP_VOICE"),
                    Speak(tts, "Modo teleop por voz. Di avanza, retrocede, izquierda, derecha o detente."),
                    ClearCommand(),
                ]),
                seq("EnterTeleopJoycon", [
                    CheckCommand("teleop_joycon"),
                    SetMode("TELEOP_JOYCON"),
                    Speak(tts, "Modo joycon activado. Usa el control para moverte."),
                    ClearCommand(),
                ]),
                status_seq(),
                seq("ListWp", [
                    CheckCommand("list_waypoints"),
                    ClearCommand(),
                    SpeakWaypointList(tts, wp),
                ]),
                ClearCommand(name="DiscardInIdle"),
            ]),
        ])

        mode_dispatch = sel("ModeDispatch", [
            emergency_mode,
            teleop_voice_mode,
            teleop_joycon_mode,
            autonomous_mode,
            patrol_mode,
            idle_mode,
        ])

        return seq("VoiceBTRoot", [
            emergency,
            mapping_overlay,
            mode_dispatch,
        ], memory=False)

    # ─── Shutdown ────────────────────────────────────────────────────────

    def destroy_node(self):
        try:
            self._asr.stop()
        except Exception:
            pass
        try:
            self._tts.stop()
        except Exception:
            pass
        super().destroy_node()


class _BBWriter:
    """Tiny blackboard-write client used by the ASR thread."""

    def __init__(self):
        self._client = py_trees.blackboard.Client(name="asr_writer")
        for key in ("command", "target_waypoint"):
            self._client.register_key(key=key, access=py_trees.common.Access.WRITE)

    def set(self, key, value):
        self._client.set(key, value)


def main():
    rclpy.init()
    node = VoiceBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
