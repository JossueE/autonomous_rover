from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args


def _strip_ros_args(argv: List[str]) -> List[str]:
    args = remove_ros_args(args=argv)[1:]
    if args and args[0] == '--':
        args = args[1:]
    return args


def wait_for_publishers(
    node: Node,
    topics: Iterable[str],
    timeout_s: float,
) -> List[str]:
    deadline = time.monotonic() + timeout_s
    topics = list(topics)
    missing = topics
    while time.monotonic() < deadline:
        missing = [topic for topic in topics if node.count_publishers(topic) < 1]
        if not missing:
            return []
        rclpy.spin_once(node, timeout_sec=0.1)
    return missing


def check_graph(
    node: Node,
    wheel_topic: str,
    rtabmap_topic: str,
    cmd_vel_topic: str,
    timeout_s: float,
) -> bool:
    missing = wait_for_publishers(node, [wheel_topic, rtabmap_topic, cmd_vel_topic], timeout_s)
    if missing:
        node.get_logger().error(f'Missing publishers after {timeout_s:.1f}s: {missing}')
        return False

    cmd_publishers = node.count_publishers(cmd_vel_topic)
    if cmd_publishers != 1:
        node.get_logger().error(
            f'Expected exactly one publisher on {cmd_vel_topic}, found {cmd_publishers}.'
        )
        return False

    node.get_logger().info(
        f'Topic guard OK: {wheel_topic}, {rtabmap_topic}, {cmd_vel_topic} are ready.'
    )
    return True


class TopicGuard(Node):
    def __init__(self) -> None:
        super().__init__('odom_comparison_topic_guard')
        self.declare_parameter('wheel_topic', '/wheel/odom')
        self.declare_parameter('rtabmap_topic', '/rtabmap/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('timeout_s', 30.0)

    def run(self) -> bool:
        return check_graph(
            self,
            str(self.get_parameter('wheel_topic').value),
            str(self.get_parameter('rtabmap_topic').value),
            str(self.get_parameter('cmd_vel_topic').value),
            float(self.get_parameter('timeout_s').value),
        )


def main(argv: Optional[List[str]] = None) -> None:
    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser(description='Check odometry benchmark ROS graph.')
    parser.add_argument('--wheel-topic', default='/wheel/odom')
    parser.add_argument('--rtabmap-topic', default='/rtabmap/odom')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args(_strip_ros_args(argv))

    rclpy.init(args=argv)
    node = TopicGuard()
    node.set_parameters([
        Parameter('wheel_topic', Parameter.Type.STRING, args.wheel_topic),
        Parameter('rtabmap_topic', Parameter.Type.STRING, args.rtabmap_topic),
        Parameter('cmd_vel_topic', Parameter.Type.STRING, args.cmd_vel_topic),
        Parameter('timeout_s', Parameter.Type.DOUBLE, args.timeout),
    ])
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
