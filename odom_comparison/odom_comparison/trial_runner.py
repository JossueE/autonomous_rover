from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Empty, Trigger

from .topic_guard import check_graph

try:
    import yaml
except ImportError:  # pragma: no cover - package.xml declares python3-yaml
    yaml = None


FALLBACK_TRIALS = {
    'straight_2m': {
        'description': 'Straight 2 m forward segment.',
        'reference': {'x_m': 2.0, 'y_m': 0.0, 'yaw_deg': 0.0},
        'commands': [{'duration_s': 8.0, 'linear_x': 0.25, 'angular_z': 0.0}],
    },
    'yaw_sweep_90': {
        'description': 'From zero, turn left 90 deg, return center, turn right 90 deg, return center.',
        'reference': {'x_m': 0.0, 'y_m': 0.0, 'yaw_deg': 0.0},
        'commands': [
            {'duration_s': 3.93, 'linear_x': 0.0, 'angular_z': 0.4},
            {'duration_s': 3.93, 'linear_x': 0.0, 'angular_z': -0.4},
            {'duration_s': 3.93, 'linear_x': 0.0, 'angular_z': -0.4},
            {'duration_s': 3.93, 'linear_x': 0.0, 'angular_z': 0.4},
        ],
    },
    'free_run': {
        'description': 'Free trajectory recorded until Ctrl+C.',
        'manual_stop': True,
        'reference': {'x_m': 0.0, 'y_m': 0.0, 'yaw_deg': 0.0},
        'commands': [],
    },
}


def _strip_ros_args(argv: List[str]) -> List[str]:
    args = remove_ros_args(args=argv)[1:]
    if args and args[0] == '--':
        args = args[1:]
    return args


def _default_config_path() -> Path:
    try:
        return Path(get_package_share_directory('odom_comparison')) / 'config' / 'trials.yaml'
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1] / 'config' / 'trials.yaml'


def load_trials(config_path: Optional[str]) -> Dict[str, Dict[str, object]]:
    path = Path(config_path).expanduser() if config_path else _default_config_path()
    if yaml is None or not path.exists():
        return FALLBACK_TRIALS
    with path.open('r') as handle:
        data = yaml.safe_load(handle) or {}
    return data.get('trials', FALLBACK_TRIALS)


class TrialRunner(Node):
    def __init__(self, args) -> None:
        super().__init__('odom_comparison_trial_runner')
        self.args = args
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)

    def _call_trigger(self, service_name: str, timeout_s: float = 5.0) -> Optional[str]:
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().error(f'Service not available: {service_name}')
            return None
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'Service call timed out: {service_name}')
            return None
        result = future.result()
        if not result.success:
            self.get_logger().error(f'{service_name} failed: {result.message}')
            return None
        return result.message

    def _call_empty_if_available(self, service_names: List[str], timeout_s: float = 0.5) -> None:
        for service_name in service_names:
            client = self.create_client(Empty, service_name)
            if client.wait_for_service(timeout_sec=timeout_s):
                future = client.call_async(Empty.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                self.get_logger().info(f'Reset RTAB-Map odometry with {service_name}')
                return
        self.get_logger().warn('No RTAB-Map odometry reset service found; continuing without it.')

    def _set_recorder_parameters(self, trial_name: str, trial: Dict[str, object]) -> bool:
        client = AsyncParameterClient(self, self.args.recorder_node)
        if not client.wait_for_services(timeout_sec=5.0):
            self.get_logger().error(f'Recorder parameter service not available: {self.args.recorder_node}')
            return False

        reference = trial['reference']
        run_id = self.args.run_id or 'auto'
        output_root = self.args.output_root or ''
        notes = self.args.notes or str(trial.get('description', ''))
        parameters = [
            Parameter('trial_name', Parameter.Type.STRING, trial_name),
            Parameter('run_id', Parameter.Type.STRING, run_id),
            Parameter('reference_x_m', Parameter.Type.DOUBLE, float(reference['x_m'])),
            Parameter('reference_y_m', Parameter.Type.DOUBLE, float(reference['y_m'])),
            Parameter('reference_yaw_deg', Parameter.Type.DOUBLE, float(reference['yaw_deg'])),
            Parameter('notes', Parameter.Type.STRING, notes),
            Parameter('max_sync_dt_s', Parameter.Type.DOUBLE, float(self.args.max_sync_dt)),
        ]
        if output_root:
            parameters.append(Parameter('output_root', Parameter.Type.STRING, output_root))

        future = client.set_parameters(parameters)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('Failed to set recorder parameters.')
            return False
        failed = [result.reason for result in future.result().results if not result.successful]
        if failed:
            self.get_logger().error(f'Recorder rejected parameters: {failed}')
            return False
        return True

    def _reset_odometry(self) -> None:
        if self.args.skip_reset:
            return
        client = self.create_client(Trigger, '/reset_odometry')
        if client.wait_for_service(timeout_sec=2.0):
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            self.get_logger().info('Reset wheel odometry with /reset_odometry')
        else:
            self.get_logger().warn('/reset_odometry not available; wheel odom was not reset.')

        if not self.args.no_reset_rtabmap:
            self._call_empty_if_available([
                '/rtabmap/reset_odom',
                '/rtabmap/icp_odometry/reset_odom',
                '/rtabmap/rgbd_odometry/reset_odom',
            ])

    def _publish_zero(self, duration_s: float = 1.0) -> None:
        msg = Twist()
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

    def _run_commands(self, commands: List[Dict[str, float]]) -> None:
        if self.args.skip_commands:
            total = sum(float(command['duration_s']) for command in commands)
            self.get_logger().info(f'Skip commands enabled; recording for {total:.1f}s.')
            deadline = time.monotonic() + total
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            return

        for index, command in enumerate(commands, start=1):
            duration = float(command['duration_s'])
            msg = Twist()
            msg.linear.x = float(command.get('linear_x', 0.0))
            msg.angular.z = float(command.get('angular_z', 0.0))
            self.get_logger().info(
                f"Segment {index}: duration={duration:.2f}s vx={msg.linear.x:.3f} wz={msg.angular.z:.3f}"
            )
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                self.cmd_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(1.0 / self.args.rate_hz)
        self._publish_zero()

    def _wait_for_manual_stop(self) -> None:
        self.get_logger().info('Free run recording. Press Ctrl+C to stop and write artifacts.')
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.2)
        except KeyboardInterrupt:
            self.get_logger().info('Free run stop requested.')

    def run_trial(self, trial_name: str, trial: Dict[str, object]) -> bool:
        if not self.args.skip_topic_guard:
            ok = check_graph(
                self,
                self.args.wheel_topic,
                self.args.rtabmap_topic,
                self.args.cmd_vel_topic,
                self.args.topic_timeout,
            )
            if not ok:
                return False

        self._reset_odometry()
        if not self._set_recorder_parameters(trial_name, trial):
            return False

        start_message = self._call_trigger(f'{self.args.recorder_node}/start')
        if start_message is None:
            return False
        self.get_logger().info(start_message)

        if bool(trial.get('manual_stop', False)):
            self._wait_for_manual_stop()
        else:
            self._run_commands(trial['commands'])

        stop_message = self._call_trigger(f'{self.args.recorder_node}/stop', timeout_s=20.0)
        if stop_message is None:
            return False
        self.get_logger().info(stop_message)
        return True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Run predefined odometry comparison trials.')
    parser.add_argument('--trial', default='straight_2m')
    parser.add_argument('--list', action='store_true', help='List available trials and exit.')
    parser.add_argument('--config', default=None)
    parser.add_argument('--cmd-topic', default='/cmd_vel_test')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--wheel-topic', default='/wheel/odom')
    parser.add_argument('--rtabmap-topic', default='/rtabmap/odom')
    parser.add_argument('--recorder-node', default='/odom_compare_recorder')
    parser.add_argument('--rate-hz', type=float, default=10.0)
    parser.add_argument('--topic-timeout', type=float, default=30.0)
    parser.add_argument('--max-sync-dt', type=float, default=0.05)
    parser.add_argument('--run-id', default='')
    parser.add_argument('--output-root', default='')
    parser.add_argument('--notes', default='')
    parser.add_argument('--skip-topic-guard', action='store_true')
    parser.add_argument('--skip-reset', action='store_true')
    parser.add_argument('--no-reset-rtabmap', action='store_true')
    parser.add_argument('--skip-commands', action='store_true')
    return parser


def main(argv: Optional[List[str]] = None) -> None:
    argv = sys.argv if argv is None else argv
    args = build_parser().parse_args(_strip_ros_args(argv))
    trials = load_trials(args.config)

    if args.list:
        for name, trial in trials.items():
            print(f"{name}: {trial.get('description', '')}")
        return

    if args.trial not in trials:
        print(f"Unknown trial '{args.trial}'. Use --list to inspect available trials.", file=sys.stderr)
        raise SystemExit(2)

    rclpy.init(args=argv)
    node = TrialRunner(args)
    try:
        ok = node.run_trial(args.trial, trials[args.trial])
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
