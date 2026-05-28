"""Maps recognized voice phrases (ES/EN) to canonical commands."""

COMMAND_MAP = {
    # Emergency
    "emergencia": "emergency_stop",
    "emergency": "emergency_stop",
    "reanuda": "resume",
    "resume": "resume",

    # Movement (TELEOP_VOICE)
    "avanza": "forward",
    "adelante": "forward",
    "forward": "forward",
    "retrocede": "backward",
    "atras": "backward",
    "atrás": "backward",
    "back": "backward",
    "izquierda": "turn_left",
    "gira a la izquierda": "turn_left",
    "turn left": "turn_left",
    "derecha": "turn_right",
    "gira a la derecha": "turn_right",
    "turn right": "turn_right",
    "detente": "stop",
    "alto": "stop",
    "para": "stop",
    "stop": "stop",
    "halt": "stop",

    # Mode switches
    "modo voz": "teleop_voice",
    "teleop voz": "teleop_voice",
    "voice": "teleop_voice",
    "modo joycon": "teleop_joycon",
    "joycon": "teleop_joycon",
    "modo autonomo": "autonomous",
    "modo autónomo": "autonomous",
    "autonomo": "autonomous",
    "autónomo": "autonomous",
    "autonomous": "autonomous",
    "patrulla": "patrol",
    "patrol": "patrol",

    # Mapping
    "inicia mapeo": "start_mapping",
    "start mapping": "start_mapping",
    "deten el mapeo": "stop_mapping",
    "detén el mapeo": "stop_mapping",
    "stop mapping": "stop_mapping",
    "guarda el mapa": "stop_mapping",

    # Info
    "estado": "status",
    "status": "status",
    "donde estas": "status",
    "dónde estás": "status",
    "a donde puedes ir": "list_waypoints",
    "a dónde puedes ir": "list_waypoints",
    "waypoints": "list_waypoints",
}

# Prefixes that indicate "navigate to <waypoint>". The remainder of the phrase
# is scanned for a waypoint name.
NAVIGATE_PREFIXES = ["ve a", "ir a", "go to", "navigate to"]


def parse(text: str, waypoint_names):
    """Parse recognized text. Returns (command, waypoint_name_or_None) or (None, None).

    waypoint_names: iterable of valid waypoint name strings.
    """
    if not text:
        return None, None
    t = text.strip().lower()

    # Navigation: try prefixes first since they consume the whole phrase
    for prefix in NAVIGATE_PREFIXES:
        if t.startswith(prefix + " ") or t == prefix:
            tail = t[len(prefix):].strip()
            # strip leading articles
            for art in ("la ", "el ", "los ", "las ", "the "):
                if tail.startswith(art):
                    tail = tail[len(art):]
                    break
            for name in waypoint_names:
                if name.lower() in tail:
                    return "navigate", name
            return None, None

    # Try longest keys first so multi-word keys win over substrings
    for key in sorted(COMMAND_MAP.keys(), key=len, reverse=True):
        if key in t:
            return COMMAND_MAP[key], None

    return None, None
