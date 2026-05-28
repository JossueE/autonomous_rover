"""Action leaves that produce Piper TTS feedback."""

import py_trees


class Speak(py_trees.behaviour.Behaviour):
    """Speak a fixed string. If `{waypoint}` appears, substitute target_waypoint."""

    def __init__(self, tts, text: str, name: str = None):
        super().__init__(name or f"Speak({text[:30]!r})")
        self._tts = tts
        self._text = text
        self._bb = self.attach_blackboard_client(name=self.name)
        if "{waypoint}" in text:
            self._bb.register_key("target_waypoint", access=py_trees.common.Access.READ)

    def update(self):
        text = self._text
        if "{waypoint}" in text:
            wp = self._bb.get("target_waypoint") if self._bb.exists("target_waypoint") else None
            text = text.replace("{waypoint}", str(wp) if wp else "destino")
        self._tts.speak(text)
        return py_trees.common.Status.SUCCESS


class SpeakStatus(py_trees.behaviour.Behaviour):
    """Speak the current pose, mode, and mapping flag."""

    def __init__(self, tts, name: str = "SpeakStatus"):
        super().__init__(name)
        self._tts = tts
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key("robot_pose", access=py_trees.common.Access.READ)
        self._bb.register_key("pose_source", access=py_trees.common.Access.READ)
        self._bb.register_key("mode", access=py_trees.common.Access.READ)
        self._bb.register_key("is_mapping", access=py_trees.common.Access.READ)

    def update(self):
        pose = self._bb.get("robot_pose") if self._bb.exists("robot_pose") else None
        src = self._bb.get("pose_source") if self._bb.exists("pose_source") else "ninguna"
        mode = self._bb.get("mode") if self._bb.exists("mode") else "IDLE"
        mapping = self._bb.get("is_mapping") if self._bb.exists("is_mapping") else False

        if pose is None:
            text = f"No tengo posición disponible. Modo: {mode}."
        elif src == "map":
            text = (f"Estoy en x={pose[0]:.1f}, y={pose[1]:.1f} según el mapa. "
                    f"Modo: {mode}. ")
        else:
            text = (f"Posición estimada sin mapa: x={pose[0]:.1f}, y={pose[1]:.1f}. "
                    f"Modo: {mode}. ")

        text += "Estoy mapeando." if mapping else "No estoy mapeando."
        self._tts.speak(text)
        return py_trees.common.Status.SUCCESS


class SpeakWaypointList(py_trees.behaviour.Behaviour):
    """Speak the list of known waypoint names."""

    def __init__(self, tts, waypoints: dict, name: str = "SpeakWaypointList"):
        super().__init__(name)
        self._tts = tts
        self._waypoints = waypoints

    def update(self):
        if not self._waypoints:
            self._tts.speak("No tengo destinos configurados.")
            return py_trees.common.Status.SUCCESS
        names = list(self._waypoints.keys())
        joined = ", ".join(names[:-1]) + (f" y {names[-1]}" if len(names) > 1 else names[0])
        self._tts.speak(f"Puedo ir a: {joined}.")
        return py_trees.common.Status.SUCCESS
