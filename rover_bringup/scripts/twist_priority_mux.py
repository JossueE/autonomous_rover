#!/usr/bin/env python3
"""Lightweight priority Twist multiplexer (stand-in for the `twist_mux` package).

The ZLAC `wheels_driver` enforces *exactly one* publisher on `/cmd_vel`
(zlac8015d_driver2_cpp/src/wheels_driver.cpp) and FATAL-exits if it sees more
than one. Both `nmpc_controller_node` (navigation velocities) and `rover_bt_node`
(ZeroTwist / teleop / emergency) want to drive the wheels, which is two
publishers. This node is the single publisher on the output topic and merges the
two sources by priority:

  - cmd_vel_safe   (rover_bt)       -> highest priority : stop/emergency authority wins
  - cmd_vel_person (person_tracker) -> middle  priority : person-follow, when BT is quiet
  - cmd_vel_nav    (NMPC)           -> lowest  priority : passes through when both quiet

person_tracker only publishes /cmd_vel_person while rover_bt has enabled it (in
PERSON_TRACK mode), and goes silent otherwise — so it never steals the bus from
navigation, yet rover_bt's /cmd_vel_safe (emergency / odom-loss ZeroTwist) still
overrides it instantly.

Each input is only considered while it has published within `timeout` seconds, so
a source that goes silent releases the bus (matches twist_mux semantics). Output
is republished at `rate_hz` while any input is fresh.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TwistPriorityMux(Node):
    def __init__(self):
        super().__init__("twist_priority_mux")

        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("high_topic", "/cmd_vel_safe")
        self.declare_parameter("person_topic", "/cmd_vel_person")
        self.declare_parameter("low_topic", "/cmd_vel_nav")
        self.declare_parameter("timeout", 0.5)      # s; input considered stale after this
        self.declare_parameter("rate_hz", 20.0)

        out = self.get_parameter("output_topic").value
        self.high_topic = self.get_parameter("high_topic").value
        self.person_topic = self.get_parameter("person_topic").value
        self.low_topic = self.get_parameter("low_topic").value
        self.timeout = float(self.get_parameter("timeout").value)
        rate = float(self.get_parameter("rate_hz").value)

        # priority -> (last Twist, last stamp seconds). Highest priority first.
        self._inputs = {
            self.high_topic: {"prio": 100, "msg": None, "t": -1e9},
            self.person_topic: {"prio": 50, "msg": None, "t": -1e9},
            self.low_topic: {"prio": 10, "msg": None, "t": -1e9},
        }

        self.create_subscription(Twist, self.high_topic,
                                 lambda m: self._on(self.high_topic, m), 10)
        self.create_subscription(Twist, self.person_topic,
                                 lambda m: self._on(self.person_topic, m), 10)
        self.create_subscription(Twist, self.low_topic,
                                 lambda m: self._on(self.low_topic, m), 10)

        self.pub = self.create_publisher(Twist, out, 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"twist_priority_mux: [{self.high_topic} p100] > [{self.person_topic} p50] "
            f"> [{self.low_topic} p10] -> {out} (timeout={self.timeout}s, rate={rate}Hz)")

    def _on(self, topic, msg):
        e = self._inputs[topic]
        # Compute the timestamp first (may release the GIL inside the C extension).
        # Storing t before msg means _tick never sees a fresh msg with a stale t=-1e9.
        t = self.get_clock().now().nanoseconds * 1e-9
        e["t"] = t
        e["msg"] = msg

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        # Pick the highest-priority input that is still fresh.
        chosen = None
        for entry in sorted(self._inputs.values(), key=lambda e: -e["prio"]):
            if entry["msg"] is not None and (now - entry["t"]) <= self.timeout:
                chosen = entry["msg"]
                break
        if chosen is not None:
            self.pub.publish(chosen)
        # No fresh input -> publish nothing (release the bus); wheels_driver holds.


def main():
    rclpy.init()
    node = TwistPriorityMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
