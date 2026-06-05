#!/usr/bin/env python3
"""
Person detector node: YOLOv8 detection + OSNet Re-ID + reactive Twist control.

Publishes:
  /cmd_vel_safe                        (Twist)              Direct velocity commands to rover
  /person_tracker/person_detected     (Bool)               True when target is visible
  /person_tracker/person_bbox         (Float32MultiArray)  [x1,y1,x2,y2,conf,cx_norm,cy_norm]
  /person_tracker/detections_image    (Image)              Annotated debug image
"""

import time
from collections import deque
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Twist
from ultralytics import YOLO

import torch
import torchreid


# ---------------------------------------------------------------------------
# State machine states
# ---------------------------------------------------------------------------
class State:
    SEARCHING         = 'SEARCHING'
    TRACKING          = 'TRACKING'
    LOST_WAITING      = 'LOST_WAITING'
    LOST_REVERSING    = 'LOST_REVERSING'
    LOST_TURNING_LEFT = 'LOST_TURNING_LEFT'
    LOST_TURNING_RIGHT= 'LOST_TURNING_RIGHT'
    LOST_STOPPED      = 'LOST_STOPPED'


class SubState:
    """Sub-state machine inside TRACKING: corner-approach + pivot."""
    NORMAL      = 'NORMAL'        # P-distance for linear, PD for angular
    APPROACHING = 'APPROACHING'   # drive straight to last centered distance
    PIVOTING    = 'PIVOTING'      # rotate smoothly toward last seen direction


def _cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    denom = np.linalg.norm(a) * np.linalg.norm(b)
    if denom < 1e-8:
        return 0.0
    return float(np.dot(a, b) / denom)


class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__('person_tracker_node')

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('model_path',                    'yolov8n.pt')
        self.declare_parameter('confidence_threshold',          0.5)
        self.declare_parameter('device',                        '0')
        self.declare_parameter('rgb_topic',                     '/k4a/rgb/image_raw')
        self.declare_parameter('depth_topic',                   '/k4a/depth_to_rgb/image_raw')
        self.declare_parameter('camera_info_topic',             '/k4a/rgb/camera_info')
        self.declare_parameter('target_class',                  'person')
        self.declare_parameter('publish_debug_image',           True)
        # Re-ID
        self.declare_parameter('reid_model',                    'osnet_x0_25')
        self.declare_parameter('reid_similarity_threshold',     0.7)
        self.declare_parameter('reid_ema_alpha',                0.9)
        # Tracking control
        self.declare_parameter('target_distance_m',             1.1)
        self.declare_parameter('max_linear_speed',              2.0)
        self.declare_parameter('max_reverse_speed',             0.5)
        self.declare_parameter('max_angular_speed',             0.50)
        self.declare_parameter('kp_linear',                     1.5)
        self.declare_parameter('kp_angular',                    1.0)
        self.declare_parameter('kd_angular',                    1.5)
        self.declare_parameter('cx_velocity_ema_alpha',         0.6)
        self.declare_parameter('centering_deadzone',            0.05)
        self.declare_parameter('centering_suppress_linear_zone',0.20)
        # Curve behavior (depth buffer when person is far off-center)
        self.declare_parameter('center_dist_buffer_seconds',    2.0)
        self.declare_parameter('center_dist_min_samples',       3)
        self.declare_parameter('depth_min_valid_m',             0.5)
        self.declare_parameter('depth_max_valid_m',             4.0)
        self.declare_parameter('curve_max_error_x',             0.5)
        self.declare_parameter('arc_min_speed',                 0.2)
        # Velocity ramps (reactive to distance error)
        self.declare_parameter('accel_base',                    0.8)
        self.declare_parameter('accel_peak',                    1.8)
        self.declare_parameter('decel_base',                    1.0)
        self.declare_parameter('decel_peak',                    2.0)
        self.declare_parameter('ramp_urgency_scale',            2.0)
        self.declare_parameter('emergency_distance',            0.7)
        self.declare_parameter('emergency_decel',               5.0)
        # Obstacle avoidance (depth-derived 1D scan + repulsion)
        self.declare_parameter('obstacle_avoidance_enabled',    False)
        self.declare_parameter('obstacle_scan_row_min',         150)
        self.declare_parameter('obstacle_scan_row_max',         290)
        self.declare_parameter('obstacle_repulsion_distance',   0.8)
        self.declare_parameter('obstacle_critical_distance',    0.35)
        self.declare_parameter('obstacle_gain_linear',          1.2)
        self.declare_parameter('obstacle_gain_angular',         1.5)
        self.declare_parameter('obstacle_forward_arc_deg',      60.0)
        self.declare_parameter('camera_hfov_deg',               75.0)
        self.declare_parameter('obstacle_tiebreaker_after_s',   1.0)
        self.declare_parameter('obstacle_tiebreaker_preference','left')
        # Recovery
        self.declare_parameter('wait_timeout_s',                5.0)
        self.declare_parameter('reverse_duration_s',            3.0)
        self.declare_parameter('search_turn_duration_s',        5.0)
        self.declare_parameter('reverse_speed',                 0.15)
        self.declare_parameter('search_turn_speed',             0.6)
        # Corner+pivot sub-state machine (within TRACKING)
        self.declare_parameter('imu_topic',                     '/k4a/imu')
        self.declare_parameter('imu_yaw_axis',                  'z')
        self.declare_parameter('corner_threshold',              0.45)
        self.declare_parameter('recover_zone',                  0.20)
        self.declare_parameter('approach_speed',                1.0)
        self.declare_parameter('approach_max_duration_s',       3.0)
        self.declare_parameter('pivot_max_angular_speed',       1.0)
        self.declare_parameter('pivot_ramp_duration_s',         0.3)
        self.declare_parameter('pivot_max_angle_rad',           2.5)

        p = self.get_parameter
        model_path          = str(p('model_path').value)
        self.conf_thresh    = float(p('confidence_threshold').value)
        self.device         = str(p('device').value)
        rgb_topic           = str(p('rgb_topic').value)
        depth_topic         = str(p('depth_topic').value)
        info_topic          = str(p('camera_info_topic').value)
        self.target_class   = str(p('target_class').value)
        self.publish_debug  = bool(p('publish_debug_image').value)

        reid_model_name         = str(p('reid_model').value)
        self.reid_threshold     = float(p('reid_similarity_threshold').value)
        self.reid_ema_alpha     = float(p('reid_ema_alpha').value)

        self.target_distance_m          = float(p('target_distance_m').value)
        self.max_linear_speed           = float(p('max_linear_speed').value)
        self.max_reverse_speed          = float(p('max_reverse_speed').value)
        self.max_angular_speed          = float(p('max_angular_speed').value)
        self.kp_linear                  = float(p('kp_linear').value)
        self.kp_angular                 = float(p('kp_angular').value)
        self.kd_angular                 = float(p('kd_angular').value)
        self.cx_velocity_ema_alpha      = float(p('cx_velocity_ema_alpha').value)
        self.centering_deadzone         = float(p('centering_deadzone').value)
        self.centering_suppress_zone    = float(p('centering_suppress_linear_zone').value)

        self.buffer_max_age_s       = float(p('center_dist_buffer_seconds').value)
        self.buffer_min_samples     = int(p('center_dist_min_samples').value)
        self.depth_min_valid        = float(p('depth_min_valid_m').value)
        self.depth_max_valid        = float(p('depth_max_valid_m').value)
        self.curve_max_error_x      = float(p('curve_max_error_x').value)
        self.arc_min_speed          = float(p('arc_min_speed').value)

        self.accel_base             = float(p('accel_base').value)
        self.accel_peak             = float(p('accel_peak').value)
        self.decel_base             = float(p('decel_base').value)
        self.decel_peak             = float(p('decel_peak').value)
        self.ramp_urgency_scale     = float(p('ramp_urgency_scale').value)
        self.emergency_distance     = float(p('emergency_distance').value)
        self.emergency_decel        = float(p('emergency_decel').value)

        self.obstacle_avoidance_enabled = bool(p('obstacle_avoidance_enabled').value)
        self.obstacle_scan_row_min      = int(p('obstacle_scan_row_min').value)
        self.obstacle_scan_row_max      = int(p('obstacle_scan_row_max').value)
        self.obstacle_repulsion_distance= float(p('obstacle_repulsion_distance').value)
        self.obstacle_critical_distance = float(p('obstacle_critical_distance').value)
        self.obstacle_gain_linear       = float(p('obstacle_gain_linear').value)
        self.obstacle_gain_angular      = float(p('obstacle_gain_angular').value)
        self.obstacle_forward_arc_rad   = np.deg2rad(float(p('obstacle_forward_arc_deg').value))
        self.camera_hfov_rad            = np.deg2rad(float(p('camera_hfov_deg').value))
        self.obstacle_tiebreaker_after_s= float(p('obstacle_tiebreaker_after_s').value)
        self.obstacle_tiebreaker_preference = str(p('obstacle_tiebreaker_preference').value).lower()

        self.wait_timeout_s         = float(p('wait_timeout_s').value)
        self.reverse_duration_s     = float(p('reverse_duration_s').value)
        self.search_turn_duration_s = float(p('search_turn_duration_s').value)
        self.reverse_speed          = float(p('reverse_speed').value)
        self.search_turn_speed      = float(p('search_turn_speed').value)

        self.imu_topic                  = str(p('imu_topic').value)
        self.imu_yaw_axis               = str(p('imu_yaw_axis').value)
        self.corner_threshold           = float(p('corner_threshold').value)
        self.recover_zone               = float(p('recover_zone').value)
        self.approach_speed             = float(p('approach_speed').value)
        self.approach_max_duration_s    = float(p('approach_max_duration_s').value)
        self.pivot_max_angular_speed    = float(p('pivot_max_angular_speed').value)
        self.pivot_ramp_duration_s      = float(p('pivot_ramp_duration_s').value)
        self.pivot_max_angle_rad        = float(p('pivot_max_angle_rad').value)

        # ------------------------------------------------------------------ #
        # YOLO
        # ------------------------------------------------------------------ #
        self.get_logger().info(f'Loading YOLO model: {model_path} on device {self.device}')
        self.model = YOLO(model_path)

        self.target_cls_id = None
        for cls_id, name in self.model.names.items():
            if name == self.target_class:
                self.target_cls_id = cls_id
                break
        if self.target_cls_id is None:
            self.get_logger().warn(
                f'Class "{self.target_class}" not found in model. '
                f'Available: {list(self.model.names.values())}'
            )

        # ------------------------------------------------------------------ #
        # OSNet Re-ID
        # ------------------------------------------------------------------ #
        self.get_logger().info(f'Loading OSNet Re-ID model: {reid_model_name}')
        reid_device = 'cuda' if (self.device != 'cpu' and torch.cuda.is_available()) else 'cpu'
        self.reid_extractor = torchreid.utils.FeatureExtractor(
            model_name=reid_model_name,
            model_path='auto',       # downloads Market-1501 pretrained weights
            device=reid_device
        )
        self.target_embedding: np.ndarray | None = None

        # Centered-depth buffer (timestamps, distance_m) — feeds curved off-center motion
        self.center_dist_buffer: deque = deque()

        # Velocity-ramp state: last published linear, last publish time, latest distance estimate.
        self.last_linear_cmd: float = 0.0
        self.last_cmd_time: float | None = None
        self._latest_ref_distance: float | None = None
        self._was_in_emergency: bool = False

        # Obstacle repulsion state (computed each depth frame, applied each twist publish)
        self._latest_repulsion: tuple[float, float] = (0.0, 0.0)   # (Fx, Fy) in rover frame
        self._obstacle_pressure: float = 0.0                         # max strength in scan band [0..1]
        self._stuck_start_time: float | None = None
        self._was_in_obstacle: bool = False

        # Lateral velocity of cx_norm (image-space derivative, EMA-smoothed) — drives the D-term.
        self.prev_cx_norm: float = 0.5
        self.prev_cx_time: float | None = None
        self.cx_velocity_filtered: float = 0.0

        # ─── Corner+Pivot sub-state machine ────────────────────────────────
        self.tracking_substate: str = SubState.NORMAL
        self.substate_start_time: float = time.time()
        self.last_centered_distance: float | None = None
        self.pivot_angular_sign: float = 0.0          # +1 = CCW (person was left), -1 = CW (person was right)
        self.distance_driven: float = 0.0
        self.last_approach_time: float | None = None
        self.pivot_angle_accumulated: float = 0.0      # integrated from IMU (rad, absolute)
        self.last_imu_time: float | None = None
        self.latest_yaw_rate: float = 0.0
        self._tracking_give_up: bool = False           # set by PIVOTING when it has searched enough

        # ------------------------------------------------------------------ #
        # Camera state
        # ------------------------------------------------------------------ #
        self.latest_depth_image = None
        self.cam_info_k         = None
        self.scale_x            = 1.0
        self.scale_y            = 1.0

        # ------------------------------------------------------------------ #
        # State machine
        # ------------------------------------------------------------------ #
        self.state            = State.SEARCHING
        self.state_start_time = time.time()
        self.get_logger().info(f'Initial state: {self.state}')

        # ------------------------------------------------------------------ #
        # QoS
        # ------------------------------------------------------------------ #
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ------------------------------------------------------------------ #
        # Subscribers
        # ------------------------------------------------------------------ #
        self.create_subscription(Image,      rgb_topic,   self.rgb_callback,   sensor_qos)
        self.create_subscription(Image,      depth_topic, self.depth_callback, sensor_qos)
        self.create_subscription(CameraInfo, info_topic,  self.info_callback,  sensor_qos)

        # IMU: high-rate, best-effort QoS suits the Kinect IMU stream (~1.6 kHz).
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, imu_qos)

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self.pub_cmd_vel  = self.create_publisher(Twist,              '/cmd_vel_safe',                        10)
        self.pub_detected = self.create_publisher(Bool,               '/person_tracker/person_detected',     10)
        self.pub_bbox     = self.create_publisher(Float32MultiArray,   '/person_tracker/person_bbox',         10)
        if self.publish_debug:
            self.pub_debug = self.create_publisher(Image, '/person_tracker/detections_image', 10)

        self.get_logger().info(
            f'PersonDetector ready. RGB: {rgb_topic} | Depth: {depth_topic} | '
            f'Class: "{self.target_class}" | Target dist: {self.target_distance_m}m'
        )

    # ---------------------------------------------------------------------- #
    # cv_bridge-free image conversions (NumPy 2.x compatible)
    # ---------------------------------------------------------------------- #

    @staticmethod
    def _imgmsg_to_numpy(msg: Image, desired_encoding: str) -> np.ndarray:
        if desired_encoding == '32FC1':
            return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
        src = msg.encoding.lower().replace('-', '')
        arr3 = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1).copy()
        if desired_encoding == 'bgr8':
            if src in ('bgr8', 'bgr'):
                return arr3
            if src in ('rgb8', 'rgb'):
                return cv2.cvtColor(arr3, cv2.COLOR_RGB2BGR)
            if src in ('bgra8', 'bgra'):
                return cv2.cvtColor(arr3, cv2.COLOR_BGRA2BGR)
            if src in ('rgba8', 'rgba'):
                return cv2.cvtColor(arr3, cv2.COLOR_RGBA2BGR)
        raise ValueError(f'Unsupported conversion: {msg.encoding} → {desired_encoding}')

    @staticmethod
    def _numpy_to_imgmsg(arr: np.ndarray, encoding: str = 'bgr8') -> Image:
        msg = Image()
        msg.height, msg.width = arr.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(arr.strides[0])
        msg.data = arr.tobytes()
        return msg

    # ---------------------------------------------------------------------- #
    # Sensor callbacks
    # ---------------------------------------------------------------------- #

    def depth_callback(self, msg: Image):
        try:
            depth = self._imgmsg_to_numpy(msg, '32FC1')
            self.latest_depth_image = cv2.resize(depth, (640, 360), interpolation=cv2.INTER_NEAREST)
            if self.obstacle_avoidance_enabled:
                self._update_obstacle_repulsion()
        except Exception as e:
            self.get_logger().warn(f'depth callback error: {e}')

    def info_callback(self, msg: CameraInfo):
        self.cam_info_k = msg.k
        self.scale_x = 640.0 / msg.width  if msg.width  > 0 else 1.0
        self.scale_y = 360.0 / msg.height if msg.height > 0 else 1.0

    # ---------------------------------------------------------------------- #
    # Re-ID helpers
    # ---------------------------------------------------------------------- #

    def _extract_embedding(self, frame: np.ndarray, box: list) -> np.ndarray | None:
        """Crop person from frame using xyxy box and extract OSNet embedding."""
        x1, y1, x2, y2 = [int(v) for v in box]
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 - x1 < 10 or y2 - y1 < 10:
            return None
        crop = frame[y1:y2, x1:x2]
        crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        try:
            feat = self.reid_extractor(crop_rgb)   # returns (1, D) tensor
            return feat.cpu().numpy()[0]
        except Exception as e:
            self.get_logger().warn(f'OSNet extraction error: {e}')
            return None

    def _update_target_embedding(self, new_emb: np.ndarray):
        """Exponential moving average update of the target embedding."""
        alpha = self.reid_ema_alpha
        self.target_embedding = alpha * self.target_embedding + (1.0 - alpha) * new_emb
        norm = np.linalg.norm(self.target_embedding)
        if norm > 1e-8:
            self.target_embedding /= norm

    # ---------------------------------------------------------------------- #
    # Distance buffer helpers (smooth curving when off-center)
    # ---------------------------------------------------------------------- #

    def _prune_dist_buffer(self):
        now = time.time()
        while self.center_dist_buffer and (now - self.center_dist_buffer[0][0]) > self.buffer_max_age_s:
            self.center_dist_buffer.popleft()

    def _buffer_median(self) -> float | None:
        """Median of recent centered-depth samples, or None if not enough fresh data."""
        self._prune_dist_buffer()
        if len(self.center_dist_buffer) < self.buffer_min_samples:
            return None
        return float(np.median([d for _, d in self.center_dist_buffer]))

    # ---------------------------------------------------------------------- #
    # IMU callback — integrates yaw rate while in PIVOTING
    # ---------------------------------------------------------------------- #

    def imu_callback(self, msg: Imu):
        try:
            yaw_rate = float(getattr(msg.angular_velocity, self.imu_yaw_axis))
        except AttributeError:
            return
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_time is not None and self.tracking_substate == SubState.PIVOTING:
            dt = now - self.last_imu_time
            if 0.0 < dt < 0.1:
                # Track total magnitude rotated, regardless of sign.
                self.pivot_angle_accumulated += abs(yaw_rate) * dt
        self.last_imu_time = now
        self.latest_yaw_rate = yaw_rate

    # ---------------------------------------------------------------------- #
    # Sub-state transitions (corner-approach + pivot)
    # ---------------------------------------------------------------------- #

    def _enter_substate_normal(self):
        if self.tracking_substate != SubState.NORMAL:
            self.get_logger().info(f'SubState: {self.tracking_substate} → NORMAL')
        self.tracking_substate = SubState.NORMAL
        self.substate_start_time = time.time()

    def _enter_substate_approaching(self, cx_norm: float, person_found: bool):
        self.get_logger().info(f'SubState: {self.tracking_substate} → APPROACHING')
        self.tracking_substate = SubState.APPROACHING
        self.substate_start_time = time.time()
        self.distance_driven = 0.0
        self.last_approach_time = None
        # Remember which way the person went; only flip if we have fresh visual info.
        if person_found:
            # Positive error_x = person to the right → rover needs to turn right (angular_z < 0).
            self.pivot_angular_sign = -1.0 if (cx_norm - 0.5) > 0 else 1.0

    def _enter_substate_pivoting(self):
        self.get_logger().info(f'SubState: {self.tracking_substate} → PIVOTING')
        self.tracking_substate = SubState.PIVOTING
        self.substate_start_time = time.time()
        self.pivot_angle_accumulated = 0.0

    def _update_tracking_substate(self, cx_norm: float, ref_distance: float | None, person_found: bool):
        """Decide whether to switch substate based on visual + integrated state."""
        abs_err = abs(cx_norm - 0.5) if person_found else 1.0

        sub = self.tracking_substate

        if sub == SubState.NORMAL:
            # Remember the last distance at which we had the person centered.
            if person_found and abs_err < self.centering_deadzone and ref_distance is not None:
                self.last_centered_distance = ref_distance
            # Trigger corner approach when the person leaves the safe zone.
            if abs_err > self.corner_threshold:
                self._enter_substate_approaching(cx_norm, person_found)

        elif sub == SubState.APPROACHING:
            # Person came back near center → resume normal following.
            if person_found and abs_err < self.recover_zone:
                self._enter_substate_normal()
                return
            # Have we covered the gap to where the person last was?
            if self.last_centered_distance is not None:
                target_gap = max(0.0, self.last_centered_distance - self.target_distance_m)
            else:
                target_gap = 0.5  # fallback if buffer never primed
            if self.distance_driven >= target_gap:
                self._enter_substate_pivoting()
                return
            # Safety: don't get stuck approaching forever.
            if (time.time() - self.substate_start_time) > self.approach_max_duration_s:
                self._enter_substate_pivoting()

        elif sub == SubState.PIVOTING:
            if person_found and abs_err < self.recover_zone:
                self._enter_substate_normal()
                return
            if self.pivot_angle_accumulated >= self.pivot_max_angle_rad:
                # Pivoted enough without finding; signal caller to bail to LOST recovery.
                self._tracking_give_up = True

    # ---------------------------------------------------------------------- #
    # Sub-state twist computations
    # ---------------------------------------------------------------------- #

    def _twist_normal(self, cx_norm: float, ref_distance: float | None) -> tuple[float, float]:
        """Pure P-distance for linear, PD for angular. No arc coupling."""
        error_x = cx_norm - 0.5
        abs_err = abs(error_x)

        p_term = 0.0 if abs_err < self.centering_deadzone else -self.kp_angular * error_x
        d_term = -self.kd_angular * self.cx_velocity_filtered
        angular_z = float(np.clip(p_term + d_term, -self.max_angular_speed, self.max_angular_speed))

        if ref_distance is None:
            return 0.0, angular_z
        error_dist = ref_distance - self.target_distance_m
        if abs(error_dist) < 0.1:
            linear_x = 0.0
        else:
            # Asymmetric cap: forward up to max_linear_speed, reverse limited to max_reverse_speed
            # (reversing fast away from a too-close person feels jarring).
            linear_x = float(np.clip(self.kp_linear * error_dist,
                                     -self.max_reverse_speed, self.max_linear_speed))
        return linear_x, angular_z

    def _twist_approaching(self) -> tuple[float, float]:
        """Drive straight forward toward the corner; integrate distance via dead reckoning."""
        now = time.time()
        if self.last_approach_time is not None:
            dt = now - self.last_approach_time
            if 0.0 < dt < 0.5:
                # Integrate the ACTUAL published linear (post-ramp) — best estimate of motion.
                self.distance_driven += abs(self.last_linear_cmd) * dt
        self.last_approach_time = now
        return float(self.approach_speed), 0.0

    def _twist_pivoting(self) -> tuple[float, float]:
        """Rotate smoothly toward last seen direction; angular ramps from 0 to max."""
        elapsed = time.time() - self.substate_start_time
        ramp = min(1.0, elapsed / max(0.001, self.pivot_ramp_duration_s))
        angular_z = self.pivot_max_angular_speed * ramp * self.pivot_angular_sign
        return 0.0, float(angular_z)

    def _update_cx_velocity(self, cx_norm: float):
        """Track lateral velocity of the bbox center (fraction/sec), EMA-smoothed."""
        now = time.time()
        if self.prev_cx_time is None:
            self.prev_cx_norm = cx_norm
            self.prev_cx_time = now
            self.cx_velocity_filtered = 0.0
            return
        dt = now - self.prev_cx_time
        if dt < 1e-3:
            return
        raw = (cx_norm - self.prev_cx_norm) / dt
        a = self.cx_velocity_ema_alpha
        self.cx_velocity_filtered = a * self.cx_velocity_filtered + (1.0 - a) * raw
        self.prev_cx_norm = cx_norm
        self.prev_cx_time = now

    def _reset_cx_velocity(self):
        """Forget tracked lateral velocity (target lost or first acquisition)."""
        self.prev_cx_time = None
        self.cx_velocity_filtered = 0.0

    def _maybe_update_dist_buffer(self, cx_norm: float, distance_m: float | None):
        """Store depth whenever person is in the trusted-depth zone (within suppress zone)."""
        if distance_m is None:
            return
        if not (self.depth_min_valid <= distance_m <= self.depth_max_valid):
            return
        if abs(cx_norm - 0.5) <= self.centering_suppress_zone:
            self.center_dist_buffer.append((time.time(), distance_m))

    # ---------------------------------------------------------------------- #
    # State machine transition helper
    # ---------------------------------------------------------------------- #

    def _transition(self, new_state: str):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state
        self.state_start_time = time.time()
        # Whenever we (re)enter TRACKING, start the sub-FSM fresh.
        if new_state == State.TRACKING:
            self._enter_substate_normal()

    def _elapsed(self) -> float:
        return time.time() - self.state_start_time

    # ---------------------------------------------------------------------- #
    # Velocity publisher helpers
    # ---------------------------------------------------------------------- #

    # ---------------------------------------------------------------------- #
    # Obstacle avoidance — depth-derived 1D scan + repulsion vector
    # ---------------------------------------------------------------------- #

    def _update_obstacle_repulsion(self):
        """Build (Fx, Fy) repulsion from a horizontal strip of the depth image."""
        depth = self.latest_depth_image
        if depth is None:
            self._latest_repulsion = (0.0, 0.0)
            self._obstacle_pressure = 0.0
            return

        h, w = depth.shape
        r_min = max(0, min(h - 1, self.obstacle_scan_row_min))
        r_max = max(r_min + 1, min(h, self.obstacle_scan_row_max))

        strip = depth[r_min:r_max, :]
        # Per-column min depth, ignoring invalid (≤ critical*0.7) values to skip camera housing.
        guard = self.obstacle_critical_distance * 0.7
        masked = np.where(strip > guard, strip, np.inf)
        col_min = masked.min(axis=0)   # shape (w,)

        # Angle per column: leftmost column → positive theta (left in robot frame)
        cols = np.arange(w)
        theta = (w / 2.0 - cols) / w * self.camera_hfov_rad

        # Filter: inside forward arc and inside repulsion range
        arc_half = self.obstacle_forward_arc_rad / 2.0
        forward_mask = np.abs(theta) <= arc_half
        close_mask = col_min < self.obstacle_repulsion_distance
        valid_mask = forward_mask & close_mask & np.isfinite(col_min)

        if not np.any(valid_mask):
            self._latest_repulsion = (0.0, 0.0)
            self._obstacle_pressure = 0.0
            return

        r = col_min[valid_mask]
        th = theta[valid_mask]

        # Linear strength: 1.0 at critical_distance (or closer), 0.0 at repulsion_distance.
        denom = self.obstacle_repulsion_distance - self.obstacle_critical_distance
        if denom <= 1e-6:
            strength = np.ones_like(r)
        else:
            strength = (self.obstacle_repulsion_distance - r) / denom
            strength = np.clip(strength, 0.0, 1.0)

        # Sum of unit vectors pointing AWAY from each obstacle column, weighted by strength.
        # Normalize by the number of columns INSIDE the forward arc to keep values bounded.
        arc_col_count = max(1, int(np.sum(forward_mask)))
        Fx = float(np.sum(-np.cos(th) * strength) / arc_col_count)
        Fy = float(np.sum(-np.sin(th) * strength) / arc_col_count)

        self._latest_repulsion = (Fx, Fy)
        self._obstacle_pressure = float(np.max(strength))

    def _apply_obstacle_repulsion(self, linear_x: float, angular_z: float) -> tuple[float, float]:
        """Inject repulsion into the desired twist and handle the symmetric tie-breaker."""
        if not self.obstacle_avoidance_enabled:
            return linear_x, angular_z

        Fx, Fy = self._latest_repulsion
        pressure = self._obstacle_pressure

        # Apply repulsion. Fx is ≤ 0 → reduces linear (brake/reverse).
        # Fy sign indicates which way to steer (positive = obstacle on right → push left → angular > 0).
        linear_x = linear_x + Fx * self.obstacle_gain_linear
        angular_z = angular_z + Fy * self.obstacle_gain_angular

        # Tie-breaker: if the rover is pressed by an obstacle but has no clear lateral push and
        # is essentially stopped, force a small angular toward a preferred side so we don't
        # get stuck at a perfectly symmetric local minimum.
        now = time.time()
        stuck = (pressure > 0.5
                 and abs(linear_x) < 0.10
                 and abs(angular_z) < 0.15)
        if stuck:
            if self._stuck_start_time is None:
                self._stuck_start_time = now
            elif (now - self._stuck_start_time) > self.obstacle_tiebreaker_after_s:
                sign = 1.0 if self.obstacle_tiebreaker_preference == 'left' else -1.0
                angular_z = sign * self.pivot_max_angular_speed * 0.5
        else:
            self._stuck_start_time = None

        # Log on entry/exit of "obstacle close" state.
        in_obstacle = pressure > 0.3
        if in_obstacle != self._was_in_obstacle:
            if in_obstacle:
                self.get_logger().info(
                    f'Obstáculo detectado (pressure={pressure:.2f}, F=({Fx:.2f},{Fy:.2f}))'
                )
            else:
                self.get_logger().info('Obstáculo despejado.')
            self._was_in_obstacle = in_obstacle

        # Clip angular to safe bounds (linear gets clipped by ramp).
        angular_z = float(np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed))
        return linear_x, angular_z

    def _apply_velocity_ramp(self, v_target: float) -> float:
        """Limit linear-velocity change per second. Limits scale with distance urgency:
        accel grows as the person gets farther; decel grows as the person gets closer."""
        now = time.time()
        dt = (now - self.last_cmd_time) if self.last_cmd_time is not None else 0.1
        # Clamp dt to handle startup, dropped frames, or long pauses sanely.
        dt = max(0.001, min(dt, 0.5))
        self.last_cmd_time = now

        v_curr = self.last_linear_cmd
        dv = v_target - v_curr

        if abs(dv) < 1e-4:
            self.last_linear_cmd = v_target
            return v_target

        ref_d = self._latest_ref_distance

        # Emergency override: bypass the slow ramp when the person is dangerously close OR
        # when an obstacle is pressing hard (so the rover can brake fast instead of "complying
        # with the ramp time").
        person_close = (ref_d is not None) and (ref_d < self.emergency_distance)
        obstacle_pressing = (
            self.obstacle_avoidance_enabled and self._obstacle_pressure > 0.7
        )
        in_emergency = person_close or obstacle_pressing
        if in_emergency != self._was_in_emergency:
            if in_emergency:
                if person_close:
                    cause = f'persona a {ref_d:.2f}m (<{self.emergency_distance:.2f}m)'
                else:
                    cause = f'obstáculo presionando (pressure={self._obstacle_pressure:.2f})'
                self.get_logger().warn(
                    f'EMERGENCY: {cause} — rampa bypass, decel = {self.emergency_decel:.1f} m/s²'
                )
            else:
                self.get_logger().info('Emergencia despejada, rampa normal reanudada.')
            self._was_in_emergency = in_emergency

        if in_emergency:
            max_a = self.emergency_decel
        else:
            if ref_d is not None:
                if dv > 0.0:
                    err = max(0.0, ref_d - self.target_distance_m)   # accelerating → urgency from being too far
                else:
                    err = max(0.0, self.target_distance_m - ref_d)   # decelerating → urgency from being too close
                urgency = min(1.0, err / max(1e-6, self.ramp_urgency_scale))
            else:
                urgency = 0.0

            if dv > 0.0:
                max_a = self.accel_base + (self.accel_peak - self.accel_base) * urgency
            else:
                max_a = self.decel_base + (self.decel_peak - self.decel_base) * urgency

        dv_limit = max_a * dt
        dv_clamped = max(-dv_limit, min(dv_limit, dv))
        v_new = v_curr + dv_clamped
        self.last_linear_cmd = v_new
        return v_new

    def _publish_twist(self, linear_x: float, angular_z: float):
        # Obstacle repulsion adjusts BOTH linear and angular before the linear ramp,
        # so the ramp smooths out the repulsion-induced brake too.
        linear_x, angular_z = self._apply_obstacle_repulsion(float(linear_x), float(angular_z))
        linear_ramped = self._apply_velocity_ramp(float(linear_x))
        msg = Twist()
        msg.linear.x  = float(linear_ramped)
        msg.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(msg)

    def _stop(self):
        self._publish_twist(0.0, 0.0)

    # ---------------------------------------------------------------------- #
    # Distance estimation from depth map
    # ---------------------------------------------------------------------- #

    def _estimate_distance(self, box: list, img_w: int, img_h: int) -> float | None:
        """Return median depth (m) of the bounding box region, or None if unavailable."""
        if self.latest_depth_image is None or self.cam_info_k is None:
            return None
        x1, y1, x2, y2 = box
        x1_px = max(0, min(img_w - 1, int(x1)))
        y1_px = max(0, min(img_h - 1, int(y1)))
        x2_px = max(0, min(img_w - 1, int(x2)))
        y2_px = max(0, min(img_h - 1, int(y2)))
        roi = self.latest_depth_image[y1_px:y2_px, x1_px:x2_px]
        valid = roi[roi > 0.3]   # ignore <0.3m (camera housing)
        if len(valid) == 0:
            return None
        return float(np.median(valid))

    # ---------------------------------------------------------------------- #
    # Main RGB callback
    # ---------------------------------------------------------------------- #

    def rgb_callback(self, msg: Image):
        try:
            frame = self._imgmsg_to_numpy(msg, 'bgr8')
            frame = cv2.resize(frame, (640, 360))
        except Exception as e:
            self.get_logger().warn(f'rgb callback error: {e}')
            return

        img_h, img_w = frame.shape[:2]

        # ---- YOLO inference (no built-in tracker needed; Re-ID handles identity) ----
        results = self.model(
            frame,
            conf=self.conf_thresh,
            device=self.device,
            classes=[self.target_cls_id] if self.target_cls_id is not None else None,
            verbose=False
        )

        result = results[0]
        boxes  = result.boxes

        # Build list of (box_xyxy, conf, embedding) for all person detections
        detections = []
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                cls_name = self.model.names[int(box.cls[0])]
                if cls_name != self.target_class:
                    continue
                xyxy = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                emb  = self._extract_embedding(frame, xyxy)
                detections.append((xyxy, conf, emb))

        # ---- Find best match for the current target ----
        target_box  = None
        target_conf = 0.0
        target_emb  = None
        target_dist = None

        if self.target_embedding is None:
            # SEARCHING: lock onto first detected person
            if detections:
                best = max(detections, key=lambda d: d[1])  # highest confidence
                xyxy, conf, emb = best
                if emb is not None:
                    target_box  = xyxy
                    target_conf = conf
                    target_emb  = emb
                    self.target_embedding = emb / (np.linalg.norm(emb) + 1e-8)
                    self._transition(State.TRACKING)
                    self.get_logger().info('Target locked via Re-ID.')
        else:
            # TRACKING / LOST_*: find detection with highest cosine similarity
            best_sim = -1.0
            for xyxy, conf, emb in detections:
                if emb is None:
                    continue
                sim = _cosine_similarity(self.target_embedding, emb)
                if sim > best_sim:
                    best_sim    = sim
                    target_box  = xyxy
                    target_conf = conf
                    target_emb  = emb

            if best_sim < self.reid_threshold:
                # No match found in this frame
                target_box = None
                target_emb = None
            else:
                self._update_target_embedding(target_emb)

        person_found = target_box is not None

        # ---- Estimate distance ----
        if person_found:
            target_dist = self._estimate_distance(target_box, img_w, img_h)
        self._latest_ref_distance = target_dist

        # ---- Publish detection topics ----
        detected_msg = Bool()
        detected_msg.data = person_found
        self.pub_detected.publish(detected_msg)

        if person_found:
            x1, y1, x2, y2 = target_box
            cx      = (x1 + x2) / 2.0
            cy      = (y1 + y2) / 2.0
            cx_norm = cx / img_w
            cy_norm = cy / img_h
            bbox_msg = Float32MultiArray()
            bbox_msg.data = [
                float(x1), float(y1), float(x2), float(y2),
                float(target_conf),
                float(cx_norm), float(cy_norm)
            ]
            self.pub_bbox.publish(bbox_msg)
            self._maybe_update_dist_buffer(cx_norm, target_dist)
            self._update_cx_velocity(cx_norm)
        else:
            cx_norm = 0.5   # unused but avoids unbound variable
            self._reset_cx_velocity()

        # ------------------------------------------------------------------ #
        # State machine + velocity commands
        # ------------------------------------------------------------------ #
        twist_linear  = 0.0
        twist_angular = 0.0

        if self.state == State.SEARCHING:
            # Wait for first lock — no motion
            self._stop()

        elif self.state == State.TRACKING:
            # Sub-state machine handles "person off-frame" cases (APPROACHING/PIVOTING)
            # so we don't bail to LOST_WAITING just because person_found is False.
            twist_linear, twist_angular = self._compute_tracking_twist(
                cx_norm, target_dist, person_found
            )
            if self._tracking_give_up:
                self._tracking_give_up = False
                self._enter_substate_normal()
                self._transition(State.LOST_WAITING)
                self._stop()

        elif self.state == State.LOST_WAITING:
            if person_found:
                self._transition(State.TRACKING)
            elif self._elapsed() >= self.wait_timeout_s:
                self._transition(State.LOST_REVERSING)
            else:
                self._stop()

        elif self.state == State.LOST_REVERSING:
            if person_found:
                self._transition(State.TRACKING)
            elif self._elapsed() >= self.reverse_duration_s:
                self._transition(State.LOST_TURNING_LEFT)
            else:
                twist_linear = -self.reverse_speed

        elif self.state == State.LOST_TURNING_LEFT:
            if person_found:
                self._transition(State.TRACKING)
            elif self._elapsed() >= self.search_turn_duration_s:
                self._transition(State.LOST_TURNING_RIGHT)
            else:
                twist_angular = self.search_turn_speed

        elif self.state == State.LOST_TURNING_RIGHT:
            if person_found:
                self._transition(State.TRACKING)
            elif self._elapsed() >= self.search_turn_duration_s:
                self._transition(State.LOST_STOPPED)
            else:
                twist_angular = -self.search_turn_speed

        elif self.state == State.LOST_STOPPED:
            if person_found:
                self._transition(State.TRACKING)
            else:
                self._stop()

        # Publish velocity (only if not already handled via _stop())
        if self.state not in (State.SEARCHING, State.LOST_WAITING, State.LOST_STOPPED):
            self._publish_twist(twist_linear, twist_angular)

        # ---- Debug image ----
        if self.publish_debug:
            self._publish_debug_image(result, frame, target_box, target_dist, img_w, img_h)

    # ---------------------------------------------------------------------- #
    # Control law
    # ---------------------------------------------------------------------- #

    def _compute_tracking_twist(
        self,
        cx_norm: float,
        distance_m: float | None,
        person_found: bool,
    ) -> tuple[float, float]:
        """Dispatcher: pick a twist based on the corner+pivot sub-state."""
        # Reference distance: buffer fallback when person off-center / not visible.
        buf_dist = self._buffer_median()
        abs_err = abs(cx_norm - 0.5) if person_found else 1.0
        if person_found and abs_err <= self.corner_threshold:
            ref_distance = distance_m if distance_m is not None else buf_dist
        else:
            ref_distance = buf_dist if buf_dist is not None else distance_m

        self._update_tracking_substate(cx_norm, ref_distance, person_found)

        if self.tracking_substate == SubState.NORMAL:
            return self._twist_normal(cx_norm, ref_distance)
        if self.tracking_substate == SubState.APPROACHING:
            return self._twist_approaching()
        if self.tracking_substate == SubState.PIVOTING:
            return self._twist_pivoting()
        return 0.0, 0.0

    # ---------------------------------------------------------------------- #
    # Debug image
    # ---------------------------------------------------------------------- #

    def _publish_debug_image(
        self,
        result,
        frame: np.ndarray,
        target_box: list | None,
        target_dist: float | None,
        img_w: int,
        img_h: int
    ):
        annotated = result.plot()

        if target_box is not None:
            x1, y1, x2, y2 = target_box
            cx_px = int((x1 + x2) / 2)
            cy_px = int((y1 + y2) / 2)

            # Green thick box for the locked target
            cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
            # Red dot at center
            cv2.circle(annotated, (cx_px, cy_px), 8, (0, 0, 255), -1)

        # State + distance overlay (top-left)
        lines = [f'State: {self.state}']
        if target_dist is not None:
            lines.append(f'Dist: {target_dist:.2f} m  (target {self.target_distance_m:.1f} m)')

        font       = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.65
        thickness  = 2
        y_offset   = 28
        for line in lines:
            (tw, th), baseline = cv2.getTextSize(line, font, font_scale, thickness)
            cv2.rectangle(annotated, (8, y_offset - th - 6), (8 + tw + 8, y_offset + baseline), (0, 0, 0), -1)
            cv2.putText(annotated, line, (12, y_offset), font, font_scale, (255, 255, 255), thickness)
            y_offset += th + baseline + 10

        self.pub_debug.publish(self._numpy_to_imgmsg(annotated, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unhandled exception: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
