"""Condition leaves: read blackboard, return SUCCESS/FAILURE."""

import math
import py_trees


class CheckCommand(py_trees.behaviour.Behaviour):
    """SUCCESS if `voice_bt/command` equals the expected command."""

    def __init__(self, expected: str, name: str = None):
        super().__init__(name or f"CheckCmd({expected})")
        self._expected = expected
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key("command", access=py_trees.common.Access.READ)

    def update(self):
        cmd = self._bb.get("command") if self._bb.exists("command") else None
        return py_trees.common.Status.SUCCESS if cmd == self._expected else py_trees.common.Status.FAILURE


class CheckMode(py_trees.behaviour.Behaviour):
    """SUCCESS if `voice_bt/mode` equals the expected mode."""

    def __init__(self, expected: str, name: str = None):
        super().__init__(name or f"CheckMode({expected})")
        self._expected = expected
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key("mode", access=py_trees.common.Access.READ)

    def update(self):
        mode = self._bb.get("mode") if self._bb.exists("mode") else "IDLE"
        return py_trees.common.Status.SUCCESS if mode == self._expected else py_trees.common.Status.FAILURE


class CheckFlag(py_trees.behaviour.Behaviour):
    """SUCCESS if the named boolean blackboard key is truthy."""

    def __init__(self, key: str, name: str = None):
        super().__init__(name or f"CheckFlag({key})")
        self._key = key
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(key, access=py_trees.common.Access.READ)

    def update(self):
        val = self._bb.get(self._key) if self._bb.exists(self._key) else False
        return py_trees.common.Status.SUCCESS if val else py_trees.common.Status.FAILURE


class CheckGoalReached(py_trees.behaviour.Behaviour):
    """SUCCESS if robot_pose is within `tolerance` of the target waypoint's (x, y)."""

    def __init__(self, waypoints: dict, tolerance: float = 0.3, name: str = "CheckGoalReached"):
        super().__init__(name)
        self._waypoints = waypoints
        self._tolerance = tolerance
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key("robot_pose", access=py_trees.common.Access.READ)
        self._bb.register_key("target_waypoint", access=py_trees.common.Access.READ)

    def update(self):
        if not self._bb.exists("target_waypoint") or not self._bb.exists("robot_pose"):
            return py_trees.common.Status.FAILURE
        wp_name = self._bb.get("target_waypoint")
        if wp_name is None or wp_name not in self._waypoints:
            return py_trees.common.Status.FAILURE
        wp = self._waypoints[wp_name]
        pose = self._bb.get("robot_pose")
        if pose is None:
            return py_trees.common.Status.FAILURE
        dx = pose[0] - float(wp["x"])
        dy = pose[1] - float(wp["y"])
        dist = math.hypot(dx, dy)
        return py_trees.common.Status.SUCCESS if dist <= self._tolerance else py_trees.common.Status.FAILURE
