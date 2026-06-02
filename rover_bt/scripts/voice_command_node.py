#!/usr/bin/env python3
"""Standalone Python node for Bilingual Speech Recognition (ASR).

Translates recognized voice phrases into rover_bt/msg/Command messages
and publishes them to /rover_bt/commands. Decouples voice recognition from C++ BT.
"""

import os
import json
import queue
import threading
import sys
import yaml

import rclpy
from rclpy.node import Node
import sounddevice as sd
from vosk import Model, KaldiRecognizer

from rover_bt.msg import Command

# ─── Speech Commands Mapping ───
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

    # Person Tracking (Upcoming feature)
    "seguir persona": "person_track",
    "sigue a la persona": "person_track",
    "follow person": "person_track",
    "follow me": "person_track",
    "sigueme": "person_track",
    "sígueme": "person_track",
}

NAVIGATE_PREFIXES = ["ve a", "ir a", "go to", "navigate to"]


def parse_speech_command(text, waypoint_names):
    """Parse recognized text and return (command, waypoint_name_or_None)."""
    if not text:
        return None, None
    t = text.strip().lower()

    # Navigation: check prefixes first
    for prefix in NAVIGATE_PREFIXES:
        if t.startswith(prefix + " ") or t == prefix:
            tail = t[len(prefix):].strip()
            for art in ("la ", "el ", "los ", "las ", "the "):
                if tail.startswith(art):
                    tail = tail[len(art):]
                    break
            for name in waypoint_names:
                if name.lower() in tail:
                    return "navigate", name
            return None, None

    # Try longest command keys first
    for key in sorted(COMMAND_MAP.keys(), key=len, reverse=True):
        if key in t:
            return COMMAND_MAP[key], None

    return None, None


class ASRThread(threading.Thread):
    def __init__(self, node, es_model_path, en_model_path, waypoint_names, sample_rate=16000, blocksize=8000):
        super().__init__(daemon=True)
        self.node = node
        self.sample_rate = sample_rate
        self.blocksize = blocksize
        self.waypoint_names = list(waypoint_names)
        self.audio_q = queue.Queue()
        self.stop_evt = threading.Event()

        self.node.get_logger().info(f"Loading Spanish Vosk model from: {es_model_path}")
        self.model_es = Model(es_model_path)
        self.node.get_logger().info(f"Loading English Vosk model from: {en_model_path}")
        self.model_en = Model(en_model_path)

        self.rec_es = KaldiRecognizer(self.model_es, sample_rate)
        self.rec_en = KaldiRecognizer(self.model_en, sample_rate)

    def stop(self):
        self.stop_evt.set()

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            self.node.get_logger().warn(f"Audio status warning: {status}")
        self.audio_q.put(bytes(indata))

    def run(self):
        self.node.get_logger().info("Starting ASR audio recording stream...")
        try:
            with sd.RawInputStream(samplerate=self.sample_rate,
                                   blocksize=self.blocksize,
                                   dtype='int16', channels=1,
                                   callback=self.audio_callback):
                while not self.stop_evt.is_set():
                    try:
                        chunk = self.audio_q.get(timeout=0.5)
                    except queue.Empty:
                        continue

                    if self.rec_es.AcceptWaveform(chunk):
                        result = json.loads(self.rec_es.Result()).get("text", "")
                        if result:
                            self.dispatch(result, "ES")
                    if self.rec_en.AcceptWaveform(chunk):
                        result = json.loads(self.rec_en.Result()).get("text", "")
                        if result:
                            self.dispatch(result, "EN")
        except Exception as e:
            self.node.get_logger().error(f"ASR thread crashed: {e}")

    def dispatch(self, text, lang):
        cmd, wp = parse_speech_command(text, self.waypoint_names)
        if cmd is None:
            return
        
        self.node.get_logger().info(f"[{lang}] Speech Recognized: '{text}' -> cmd: {cmd}, wp: {wp}")
        
        # Publish to /rover_bt/commands
        msg = Command()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.source = "voice"
        msg.priority = 2
        msg.command = cmd
        msg.target = wp if wp is not None else ""
        self.node.publish_command(msg)


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__("voice_command_node")

        # Package share directories for default models
        from ament_index_python.packages import get_package_share_directory
        rover_bt_share = get_package_share_directory("rover_bt")

        self.declare_parameter("vosk_model_es", os.path.join(rover_bt_share, "voice_assets", "model_es"))
        self.declare_parameter("vosk_model_en", os.path.join(rover_bt_share, "voice_assets", "model_en"))
        self.declare_parameter("waypoints_file", os.path.join(rover_bt_share, "config", "waypoints.yaml"))
        self.declare_parameter("command_topic", "/rover_bt/commands")

        # Load Waypoints for parser vocabulary
        wp_path = self.get_parameter("waypoints_file").value
        waypoint_names = []
        if os.path.exists(wp_path):
            try:
                with open(wp_path, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and "waypoints" in data:
                        waypoint_names = list(data["waypoints"].keys())
            except Exception as e:
                self.get_logger().error(f"Failed to load waypoints: {e}")

        self.get_logger().info(f"Loaded waypoints vocabulary: {waypoint_names}")

        # Command Publisher
        cmd_topic = self.get_parameter("command_topic").value
        self.cmd_pub = self.create_publisher(Command, cmd_topic, 10)

        # Start ASR Thread
        es_model = self.get_parameter("vosk_model_es").value
        en_model = self.get_parameter("vosk_model_en").value
        
        self.asr_thread = ASRThread(self, es_model, en_model, waypoint_names)
        self.asr_thread.start()

    def publish_command(self, msg):
        self.cmd_pub.publish(msg)

    def destroy_node(self):
        self.asr_thread.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
