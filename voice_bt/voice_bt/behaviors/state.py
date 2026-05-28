"""Action leaves that mutate blackboard state. Always SUCCESS."""

import py_trees


class SetMode(py_trees.behaviour.Behaviour):
    def __init__(self, mode: str, name: str = None):
        super().__init__(name or f"SetMode({mode})")
        self._mode = mode
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key("mode", access=py_trees.common.Access.WRITE)

    def update(self):
        self._bb.set("mode", self._mode)
        return py_trees.common.Status.SUCCESS


class SetFlag(py_trees.behaviour.Behaviour):
    def __init__(self, key: str, value: bool, name: str = None):
        super().__init__(name or f"SetFlag({key}={value})")
        self._key = key
        self._value = value
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(key, access=py_trees.common.Access.WRITE)

    def update(self):
        self._bb.set(self._key, self._value)
        return py_trees.common.Status.SUCCESS


class ClearCommand(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "ClearCommand"):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key("command", access=py_trees.common.Access.WRITE)

    def update(self):
        self._bb.set("command", None)
        return py_trees.common.Status.SUCCESS


class IncrementPatrolIndex(py_trees.behaviour.Behaviour):
    """Advance patrol_index and set target_waypoint accordingly. Wraps around."""

    def __init__(self, waypoint_names: list, name: str = "IncrementPatrolIndex"):
        super().__init__(name)
        self._names = list(waypoint_names)
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key("patrol_index", access=py_trees.common.Access.WRITE)
        self._bb.register_key("target_waypoint", access=py_trees.common.Access.WRITE)

    def update(self):
        if not self._names:
            return py_trees.common.Status.FAILURE
        idx = self._bb.get("patrol_index") if self._bb.exists("patrol_index") else 0
        idx = (idx + 1) % len(self._names)
        self._bb.set("patrol_index", idx)
        self._bb.set("target_waypoint", self._names[idx])
        return py_trees.common.Status.SUCCESS


class SetPatrolIndex(py_trees.behaviour.Behaviour):
    """Set patrol_index to a value and target_waypoint to the corresponding name."""

    def __init__(self, waypoint_names: list, index: int = 0, name: str = None):
        super().__init__(name or f"SetPatrolIndex({index})")
        self._names = list(waypoint_names)
        self._index = index
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key("patrol_index", access=py_trees.common.Access.WRITE)
        self._bb.register_key("target_waypoint", access=py_trees.common.Access.WRITE)

    def update(self):
        if not self._names:
            return py_trees.common.Status.FAILURE
        idx = self._index % len(self._names)
        self._bb.set("patrol_index", idx)
        self._bb.set("target_waypoint", self._names[idx])
        return py_trees.common.Status.SUCCESS
