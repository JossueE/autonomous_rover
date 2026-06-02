#!/usr/bin/env python3
"""Voice command node: faster-whisper ASR + webrtcvad + "oye rover" wake-word.

Flow:
  IDLE  → VAD segment → tiny model (ES) → "rover" in transcript? → AWAKE
  AWAKE → VAD segment → base model (ES) → parse command → publish Command → IDLE
          or 6 s with no speech → IDLE
"""

import os
import queue
import threading
import time
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
import sounddevice as sd
import webrtcvad
from faster_whisper import WhisperModel

from rover_bt.msg import Command, RoverStatus

# ─── Spanish command map ───────────────────────────────────────────────────────
COMMAND_MAP = {
    "emergencia": "emergency_stop",
    "reanuda": "resume",
    "avanza": "forward",
    "abanza": "forward",    # b/v phonetic swap
    "adelante": "forward",
    "retrocede": "backward",
    "atras": "backward",
    "atrás": "backward",
    "izquierda": "turn_left",
    "gira a la izquierda": "turn_left",
    "derecha": "turn_right",
    "gira a la derecha": "turn_right",
    "detente": "stop",
    "alto": "stop",
    "para": "stop",
    "modo voz": "teleop_voice",
    "modo a voz": "teleop_voice",   # Whisper artifact
    "mata voz": "teleop_voice",     # Whisper artifact
    "teleop voz": "teleop_voice",
    # NOTE: joycon teleop has no voice command on purpose — the behavior tree
    # detects joystick use and switches to TELEOP_JOYCON automatically.
    "modo autonomo": "autonomous",
    "modo autónomo": "autonomous",
    "autonomo": "autonomous",
    "autónomo": "autonomous",
    "patrulla": "patrol",
    "inicia mapeo": "start_mapping",
    "modo slam": "start_slam",
    "mejora el mapa": "start_slam",
    "actualiza el mapa": "start_slam",
    "continua el mapeo": "start_slam",
    "continúa el mapeo": "start_slam",
    "deten el mapeo": "stop_mapping",
    "detén el mapeo": "stop_mapping",
    "guarda el mapa": "stop_mapping",
    "estado": "status",
    "donde estas": "status",
    "dónde estás": "status",
    "seguir persona": "person_track",
    "sigue a la persona": "person_track",
    "sígueme": "person_track",
    "sigueme": "person_track",
}

# Commands that take the rover out of voice teleop; after one of these the node
# stops staying continuously awake and requires the wake word again.
TELEOP_EXIT_COMMANDS = {"autonomous", "patrol", "person_track", "navigate"}

NAVIGATE_PREFIXES = ["ve a", "ir a", "ve para",
                     "ir para", "ve hacia", "ir hacia",
                     "navega a", "navega para", "navega hacia",
                     "dirígete a", "dirigete a", "dirígete para", "dirigete para",
                     "dirígete hacia", "dirigete hacia"]

# "Save this place as <name>": the tail after the prefix becomes a brand-new,
# free-form location name (it is NOT in the waypoint vocabulary yet — that is
# the point of saving it). Ordered longest-first so the more specific phrase
# wins. The command published is "save_location" with the name as target.
SAVE_PREFIXES = [
    "guarda este lugar como",
    "guarda esta ubicacion como",
    "guarda esta ubicación como",
    "guarda el lugar como",
    "guarda la ubicacion como",
    "guarda la ubicación como",
    "guarda aqui como",
    "guarda aquí como",
    "guarda este lugar",
    "guarda esta ubicacion",
    "guarda esta ubicación",
]

# Passed as initial_prompt to Whisper to bias toward known vocabulary.
# Keep in sync with COMMAND_MAP / NAVIGATE_PREFIXES so the decoder favours
# the words we can actually act on.
COMMAND_PROMPT = (
    "emergencia, reanuda, "
    "avanza, adelante, retrocede, atrás, "
    "derecha, gira a la derecha, "
    "detente, alto, para, "
    "modo voz, modo autónomo, autónomo, patrulla, "
    "inicia mapeo, modo slam, mejora el mapa, detén el mapeo, guarda el mapa, "
    "guarda este lugar como, "
    "estado, dónde estás, "
    "seguir persona, sígueme, ve a"
)

# Phonetic near-misses the tiny Whisper model produces for each wake word.
# Extend this if you observe new variants in the [IDLE] heard: logs.
WAKE_ALIASES: dict[str, list[str]] = {
    "comando": ["comando", "mando", "comendo"],
}

# Emergency stop bypasses the wake word: if any of these is heard in ANY state
# the rover stops immediately. Phonetic variants included for robustness.
EMERGENCY_ALIASES = ["emergencia", "emergensia", "emerhencia", "emergencias"]
EMERGENCY_COMMAND = "emergency_stop"

# ─── Audio constants ───────────────────────────────────────────────────────────
SAMPLE_RATE = 16000
FRAME_MS = 30
FRAME_SAMPLES = int(SAMPLE_RATE * FRAME_MS / 1000)  # 480 samples per frame
FRAME_BYTES = FRAME_SAMPLES * 2                      # int16 = 2 bytes/sample

SPEECH_TRIGGER_FRAMES = 3        # consecutive voiced frames → speech started
SILENCE_TRIGGER_FRAMES = 10      # consecutive silent frames → speech ended (~300 ms)
PRE_BUFFER_FRAMES = 10           # frames prepended before speech onset
MIN_SPEECH_FRAMES = int(200 / FRAME_MS)   # drop segments shorter than 200 ms
MAX_SPEECH_FRAMES = int(8000 / FRAME_MS)  # force-emit at 8 s


def parse_speech_command(text, waypoint_names):
    """Return (command_str, waypoint_or_None) from a transcription, or (None, None)."""
    if not text:
        return None, None
    t = text.strip().lower()

    # "Save this place as <name>" — checked before navigate/command maps so the
    # free-form name is captured verbatim. Longest prefix first.
    for prefix in sorted(SAVE_PREFIXES, key=len, reverse=True):
        if t.startswith(prefix):
            tail = t[len(prefix):].strip()
            tail = tail.strip(" .,!¡¿?")
            for art in ("la ", "el ", "los ", "las "):
                if tail.startswith(art):
                    tail = tail[len(art):]
                    break
            if tail:
                return "save_location", tail
            return None, None

    for prefix in NAVIGATE_PREFIXES:
        if t.startswith(prefix + " ") or t == prefix:
            tail = t[len(prefix):].strip()
            for art in ("la ", "el ", "los ", "las "):
                if tail.startswith(art):
                    tail = tail[len(art):]
                    break
            for name in waypoint_names:
                if name.lower() in tail:
                    return "navigate", name
            return None, None

    for key in sorted(COMMAND_MAP, key=len, reverse=True):
        if key in t:
            return COMMAND_MAP[key], None

    return None, None


class VoiceCommandNode(Node):
    IDLE = "IDLE"
    AWAKE = "AWAKE"

    def __init__(self):
        super().__init__("voice_command_node")

        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("rover_bt")

        self.declare_parameter("whisper_model_wake", "base")
        self.declare_parameter("whisper_model_cmd", "base")
        self.declare_parameter("whisper_device", "auto")
        self.declare_parameter("whisper_compute_type", "auto")
        self.declare_parameter("wake_word", "comando")
        self.declare_parameter("wake_timeout_sec", 10.0)
        self.declare_parameter("waypoints_file",
                               os.path.join(share, "config", "waypoints.yaml"))
        self.declare_parameter("command_topic", "/rover_bt/commands")
        self.declare_parameter("status_topic", "/rover_bt/status")

        wake_name = self.get_parameter("whisper_model_wake").value
        cmd_name = self.get_parameter("whisper_model_cmd").value
        device_pref = self.get_parameter("whisper_device").value
        compute_pref = self.get_parameter("whisper_compute_type").value
        self.wake_word = self.get_parameter("wake_word").value.lower()
        self.wake_timeout = self.get_parameter("wake_timeout_sec").value

        device, compute_type = self._resolve_backend(device_pref, compute_pref)
        self.get_logger().info(f"Whisper backend: device={device}, compute_type={compute_type}")

        cache_dir = os.path.join(share, "voice_assets", "whisper_cache")
        os.makedirs(cache_dir, exist_ok=True)

        self.get_logger().info(f"Loading wake model '{wake_name}' ...")
        self.wake_model = WhisperModel(wake_name, device=device,
                                       compute_type=compute_type,
                                       download_root=cache_dir)
        if cmd_name == wake_name:
            self.cmd_model = self.wake_model
        else:
            self.get_logger().info(f"Loading command model '{cmd_name}' ...")
            self.cmd_model = WhisperModel(cmd_name, device=device,
                                          compute_type=compute_type,
                                          download_root=cache_dir)

        wp_path = self.get_parameter("waypoints_file").value
        self.waypoint_names = self._load_waypoints(wp_path)
        self.get_logger().info(f"Waypoints vocabulary: {self.waypoint_names}")

        cmd_topic = self.get_parameter("command_topic").value
        self.cmd_pub = self.create_publisher(Command, cmd_topic, 10)

        # Track the rover's current mode so we can stay continuously awake while
        # teleoperating by voice: in TELEOP_VOICE the operator issues a stream of
        # short movement commands ("avanza", "izquierda", "detente"), and
        # requiring the wake word + a second transcription before each one adds a
        # multi-second delay. While in teleop we keep listening directly.
        self._current_mode = "IDLE"
        status_topic = self.get_parameter("status_topic").value
        self.status_sub = self.create_subscription(
            RoverStatus, status_topic, self._on_status, 10)

        self._state = self.IDLE
        self._state_lock = threading.Lock()
        self._wake_deadline = None
        self._segment_q = queue.Queue()

        threading.Thread(target=self._audio_loop, daemon=True).start()
        threading.Thread(target=self._process_loop, daemon=True).start()

        self.get_logger().info(
            f"Voice node ready — say '{self.wake_word}' to activate.")

    # ── Rover mode tracking ─────────────────────────────────────────────────────

    def _on_status(self, msg):
        self._current_mode = msg.mode

    def _in_teleop(self):
        return self._current_mode == "TELEOP_VOICE"

    # ── Activation beep ───────────────────────────────────────────────────────

    def _beep(self):
        t = np.linspace(0, 0.12, int(SAMPLE_RATE * 0.12), endpoint=False)
        tone = (np.sin(2 * np.pi * 880 * t) * 0.4 * 32767).astype(np.int16)
        try:
            sd.play(tone, SAMPLE_RATE)
        except Exception:
            pass

    # ── Backend resolution ─────────────────────────────────────────────────────

    def _resolve_backend(self, device_pref, compute_pref):
        if device_pref == "auto":
            device = "cpu"
            try:
                import ctranslate2
                supported = ctranslate2.get_supported_compute_types("cuda")
                if supported:
                    device = "cuda"
            except Exception:
                pass
        else:
            device = device_pref

        if compute_pref == "auto":
            compute_type = "float16" if device == "cuda" else "int8"
        else:
            compute_type = compute_pref

        return device, compute_type

    # ── Waypoint loading ───────────────────────────────────────────────────────

    def _load_waypoints(self, path):
        if not os.path.exists(path):
            return []
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
            return list(data.get("waypoints", {}).keys()) if data else []
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return []

    # ── Audio capture → VAD segmentation ──────────────────────────────────────

    def _audio_loop(self):
        vad = webrtcvad.Vad(3)  # aggressiveness 0–3 (3 = most aggressive silence cuts)
        ring = []
        speech_buf = []
        voiced_count = 0
        silence_count = 0
        in_speech = False

        def callback(indata, frames, time_info, status):
            nonlocal ring, speech_buf, voiced_count, silence_count, in_speech
            if status:
                self.get_logger().warn(f"Audio status: {status}")

            frame = bytes(indata)
            is_voiced = vad.is_speech(frame, SAMPLE_RATE)

            if not in_speech:
                ring.append(frame)
                if len(ring) > PRE_BUFFER_FRAMES:
                    ring.pop(0)
                voiced_count = voiced_count + 1 if is_voiced else 0
                if voiced_count >= SPEECH_TRIGGER_FRAMES:
                    in_speech = True
                    silence_count = 0
                    speech_buf = list(ring)
            else:
                speech_buf.append(frame)
                silence_count = 0 if is_voiced else silence_count + 1

                emit = (silence_count >= SILENCE_TRIGGER_FRAMES or
                        len(speech_buf) >= MAX_SPEECH_FRAMES)
                if emit:
                    if len(speech_buf) >= MIN_SPEECH_FRAMES:
                        audio = np.frombuffer(b"".join(speech_buf), dtype=np.int16)
                        self._segment_q.put(audio)
                    in_speech = False
                    ring = []
                    speech_buf = []
                    voiced_count = 0
                    silence_count = 0

        try:
            with sd.RawInputStream(samplerate=SAMPLE_RATE,
                                   blocksize=FRAME_SAMPLES,
                                   dtype="int16", channels=1,
                                   callback=callback):
                while rclpy.ok():
                    time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Audio loop crashed: {e}")

    # ── Segment transcription + state machine ──────────────────────────────────

    def _transcribe(self, model, audio_int16, prompt=None):
        audio_f32 = audio_int16.astype(np.float32) / 32768.0
        segments, _ = model.transcribe(
            audio_f32, language="es", beam_size=1,
            initial_prompt=prompt,
            condition_on_previous_text=False,
        )
        return " ".join(s.text for s in segments).strip().lower()

    def _process_loop(self):
        while rclpy.ok():
            try:
                audio = self._segment_q.get(timeout=0.5)
            except queue.Empty:
                with self._state_lock:
                    if (self._state == self.AWAKE and self._wake_deadline and
                            time.monotonic() > self._wake_deadline):
                        self._state = self.IDLE
                        self.get_logger().info(
                            f"Listening window expired — say '{self.wake_word}' "
                            "to activate")
                continue

            with self._state_lock:
                state = self._state

            if state == self.IDLE:
                # Bias the wake transcription toward the emergency word so it is
                # caught reliably even without the wake word.
                text = self._transcribe(self.wake_model, audio,
                                        prompt="emergencia, comando")
                self.get_logger().debug(f"[IDLE] heard: '{text}' (wake_word='{self.wake_word}')")

                # Emergency has priority — no wake word required.
                if any(a in text for a in EMERGENCY_ALIASES):
                    self._publish(EMERGENCY_COMMAND)
                    self.get_logger().warn("EMERGENCY detected without wake word — stopping.")
                    continue

                aliases = WAKE_ALIASES.get(self.wake_word, [self.wake_word])
                if any(a in text for a in aliases):
                    threading.Thread(target=self._beep, daemon=True).start()
                    # Check if the command was in the same utterance ("comando avanza")
                    cmd, wp = parse_speech_command(text, self.waypoint_names)
                    if cmd:
                        self._publish(cmd, wp)
                        self.get_logger().info("Wake+command in same utterance.")
                    else:
                        with self._state_lock:
                            self._state = self.AWAKE
                            self._wake_deadline = time.monotonic() + self.wake_timeout
                        self.get_logger().info(
                            f"Activated — listening for command ({self.wake_timeout:.0f}s window)...")
            else:  # AWAKE
                text = self._transcribe(self.cmd_model, audio, prompt=COMMAND_PROMPT)
                self.get_logger().debug(f"[AWAKE] heard: '{text}'")
                cmd, wp = parse_speech_command(text, self.waypoint_names)
                if cmd:
                    self._publish(cmd, wp)
                    with self._state_lock:
                        # In voice teleop, stay awake so the next movement command
                        # is acted on immediately — no wake word, no double
                        # transcription. Entering teleop ("modo voz") opts in;
                        # leaving it (autónomo / patrulla / seguir persona) drops
                        # back to requiring the wake word.
                        entering = cmd == "teleop_voice"
                        leaving = cmd in TELEOP_EXIT_COMMANDS
                        if (entering or self._in_teleop()) and not leaving:
                            self._wake_deadline = time.monotonic() + self.wake_timeout
                        else:
                            self._state = self.IDLE
                else:
                    self.get_logger().debug("No command matched — still listening...")
                    with self._state_lock:
                        self._wake_deadline = time.monotonic() + self.wake_timeout

    def _publish(self, command, target=None):
        msg = Command()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = "voice"
        msg.priority = 2
        msg.command = command
        msg.target = target if target else ""
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Command published: '{command}'" +
                               (f" → '{target}'" if target else ""))


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
