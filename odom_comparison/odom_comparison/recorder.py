from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import OdomInfo
from std_srvs.srv import Trigger

from .artifacts import write_all_artifacts
from .metrics import PoseSample, yaw_from_quaternion


def _stamp_to_seconds(msg_stamp, fallback: float) -> float:
    stamp = float(msg_stamp.sec) + float(msg_stamp.nanosec) * 1e-9
    return stamp if stamp > 0.0 else fallback


class OdomCompareRecorder(Node):
    def __init__(self) -> None:
        super().__init__('odom_compare_recorder')

        self.declare_parameter('wheel_topic', '/wheel/odom')
        self.declare_parameter('rtabmap_topic', '/rtabmap/odom')
        self.declare_parameter('rtabmap_info_lite_topic', '/rtabmap/odom_info_lite')
        self.declare_parameter('rtabmap_info_topic', '/rtabmap/odom_info')
        self.declare_parameter('output_root', str(Path.cwd() / 'odom_comparison' / 'images'))
        self.declare_parameter('run_id', 'auto')
        self.declare_parameter('trial_name', 'manual')
        self.declare_parameter('reference_x_m', 0.0)
        self.declare_parameter('reference_y_m', 0.0)
        self.declare_parameter('reference_yaw_deg', 0.0)
        self.declare_parameter('notes', '')
        self.declare_parameter('max_sync_dt_s', 0.05)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('auto_stop_duration_s', 0.0)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('wheel_topic').value,
            self._wheel_cb,
            qos,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('rtabmap_topic').value,
            self._rtabmap_cb,
            qos,
        )
        self.create_subscription(
            OdomInfo,
            self.get_parameter('rtabmap_info_lite_topic').value,
            lambda msg: self._rtab_info_cb(msg, self.get_parameter('rtabmap_info_lite_topic').value),
            qos,
        )
        self.create_subscription(
            OdomInfo,
            self.get_parameter('rtabmap_info_topic').value,
            lambda msg: self._rtab_info_cb(msg, self.get_parameter('rtabmap_info_topic').value),
            qos,
        )

        self.create_service(Trigger, '~/start', self._start_service)
        self.create_service(Trigger, '~/stop', self._stop_service)

        self._recording = False
        self._wheel_samples: List[PoseSample] = []
        self._rtabmap_samples: List[PoseSample] = []
        self._rtabmap_info: List[Dict[str, object]] = []
        self._session: Dict[str, object] = {}
        self._auto_stop_timer = None

        if bool(self.get_parameter('auto_start').value):
            self._start_recording()
            duration = float(self.get_parameter('auto_stop_duration_s').value)
            if duration > 0.0:
                self._auto_stop_timer = self.create_timer(duration, self._auto_stop_once)

        self.get_logger().info(
            'Recorder ready. Use /odom_compare_recorder/start and /odom_compare_recorder/stop.'
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _snapshot_session(self) -> Dict[str, object]:
        trial_name = str(self.get_parameter('trial_name').value)
        run_id = str(self.get_parameter('run_id').value)
        if run_id == 'auto':
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            run_id = f'{stamp}_{trial_name}'
        output_root = Path(str(self.get_parameter('output_root').value)).expanduser()
        return {
            'trial_name': trial_name,
            'run_id': run_id,
            'output_dir': output_root / run_id,
            'reference_x_m': float(self.get_parameter('reference_x_m').value),
            'reference_y_m': float(self.get_parameter('reference_y_m').value),
            'reference_yaw_deg': float(self.get_parameter('reference_yaw_deg').value),
            'notes': str(self.get_parameter('notes').value),
            'max_sync_dt_s': float(self.get_parameter('max_sync_dt_s').value),
            'started_at': self._now_seconds(),
        }

    def _start_recording(self) -> str:
        self._wheel_samples.clear()
        self._rtabmap_samples.clear()
        self._rtabmap_info.clear()
        self._session = self._snapshot_session()
        self._recording = True
        message = (
            f"Started trial '{self._session['trial_name']}' "
            f"-> {self._session['output_dir']}"
        )
        self.get_logger().info(message)
        return message

    def _stop_recording(self) -> str:
        self._recording = False
        output_dir = Path(self._session['output_dir'])
        write_all_artifacts(
            output_dir=output_dir,
            trial_name=str(self._session['trial_name']),
            wheel_samples=self._wheel_samples,
            rtabmap_samples=self._rtabmap_samples,
            rtabmap_info_events=self._rtabmap_info,
            reference_x_m=float(self._session['reference_x_m']),
            reference_y_m=float(self._session['reference_y_m']),
            reference_yaw_deg=float(self._session['reference_yaw_deg']),
            max_sync_dt_s=float(self._session['max_sync_dt_s']),
            notes=str(self._session['notes']),
        )
        message = f'Wrote artifacts to {output_dir}'
        self.get_logger().info(message)
        return message

    def _start_service(self, _request, response):
        if self._recording:
            response.success = False
            response.message = 'Recorder is already running.'
            return response
        response.success = True
        response.message = self._start_recording()
        return response

    def _stop_service(self, _request, response):
        if not self._recording:
            response.success = False
            response.message = 'Recorder is not running.'
            return response
        response.success = True
        response.message = self._stop_recording()
        return response

    def _auto_stop_once(self) -> None:
        if self._auto_stop_timer is not None:
            self._auto_stop_timer.cancel()
        if self._recording:
            self._stop_recording()

    def _sample_from_msg(self, msg: Odometry) -> PoseSample:
        fallback = self._now_seconds()
        q = msg.pose.pose.orientation
        return PoseSample(
            t=_stamp_to_seconds(msg.header.stamp, fallback),
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w)),
            vx=float(msg.twist.twist.linear.x),
            wz=float(msg.twist.twist.angular.z),
        )

    def _wheel_cb(self, msg: Odometry) -> None:
        if self._recording:
            self._wheel_samples.append(self._sample_from_msg(msg))

    def _rtabmap_cb(self, msg: Odometry) -> None:
        if self._recording:
            self._rtabmap_samples.append(self._sample_from_msg(msg))

    def _rtab_info_cb(self, msg: OdomInfo, topic: str) -> None:
        if not self._recording:
            return
        self._rtabmap_info.append({
            'topic': topic,
            't': self._now_seconds(),
            'lost': bool(msg.lost),
            'matches': int(msg.matches),
            'inliers': int(msg.inliers),
            'features': int(msg.features),
            'icp_inliers_ratio': float(msg.icp_inliers_ratio),
            'local_map_size': int(msg.local_map_size),
        })

    def destroy_node(self) -> bool:
        if self._recording:
            try:
                self._stop_recording()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Failed to finalize recording on shutdown: {exc}')
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = OdomCompareRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
