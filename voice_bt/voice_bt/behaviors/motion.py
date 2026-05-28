"""Action leaves that affect robot motion."""

import time

import py_trees
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters


class ZeroTwist(py_trees.behaviour.Behaviour):
    """Publish a zero Twist to /cmd_vel_safe. Always SUCCESS."""

    def __init__(self, cmd_vel_pub, name: str = "ZeroTwist"):
        super().__init__(name)
        self._pub = cmd_vel_pub

    def update(self):
        self._pub.publish(Twist())
        return py_trees.common.Status.SUCCESS


class MoveRover(py_trees.behaviour.Behaviour):
    """Publish a constant Twist for `duration` seconds, then zero. RUNNING during the move."""

    def __init__(self, cmd_vel_pub, linear: float, angular: float,
                 duration: float, name: str = None):
        super().__init__(name or f"MoveRover(lin={linear},ang={angular},dur={duration})")
        self._pub = cmd_vel_pub
        self._linear = linear
        self._angular = angular
        self._duration = duration
        self._start = None

    def initialise(self):
        self._start = time.monotonic()

    def update(self):
        elapsed = time.monotonic() - (self._start or time.monotonic())
        if elapsed >= self._duration:
            self._pub.publish(Twist())
            return py_trees.common.Status.SUCCESS
        twist = Twist()
        twist.linear.x = float(self._linear)
        twist.angular.z = float(self._angular)
        self._pub.publish(twist)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Always send a zero on exit so we don't drift on preemption.
        if new_status != py_trees.common.Status.SUCCESS:
            self._pub.publish(Twist())


class NavigateTo(py_trees.behaviour.Behaviour):
    """Set `end_lanelet_name` on the path_planning_node via SetParameters.

    Reads the target waypoint from the blackboard and sends its configured
    `osm_name` to the planner. If `osm_name` is absent, the waypoint key itself
    is used as the .osm lanelet `name` tag.
    """

    def __init__(self, ros_node, waypoints: dict,
                 service_name: str = "/path_planning_node/set_parameters",
                 name: str = "NavigateTo"):
        super().__init__(name)
        self._node = ros_node
        self._waypoints = waypoints
        self._service_name = service_name
        self._client = None
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key("target_waypoint", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self._client = self._node.create_client(SetParameters, self._service_name)

    def update(self):
        if not self._bb.exists("target_waypoint"):
            return py_trees.common.Status.FAILURE
        wp_name = self._bb.get("target_waypoint")
        if wp_name is None or wp_name not in self._waypoints:
            self._node.get_logger().warn(f"NavigateTo: unknown waypoint '{wp_name}'")
            return py_trees.common.Status.FAILURE

        lanelet_name = str(self._waypoints[wp_name].get("osm_name", wp_name))
        req = SetParameters.Request()
        p = Parameter()
        p.name = "end_lanelet_name"
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value=lanelet_name,
        )
        req.parameters = [p]

        if not self._client.service_is_ready():
            # Don't block the tick. Log once and report failure so the BT can recover.
            self._node.get_logger().warn(
                f"NavigateTo: service {self._service_name} not available yet")
            return py_trees.common.Status.FAILURE

        self._client.call_async(req)
        self._node.get_logger().info(
            f"NavigateTo: set end_lanelet_name='{lanelet_name}' for waypoint '{wp_name}'")
        return py_trees.common.Status.SUCCESS
