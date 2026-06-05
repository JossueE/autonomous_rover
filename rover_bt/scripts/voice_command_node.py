#!/usr/bin/env python3
"""ROS 2 bridge for wake word, STT commands, and Python TTS playback."""

from __future__ import annotations

import logging
import os
import queue
import re
import threading
import time
import unicodedata
from difflib import SequenceMatcher
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from rover_bt.msg import Command
from stt.audio_listener import AudioListener
from stt.speech_to_text import SpeechToText
from stt.wake_word import WakeWord
from tts.text_to_speech import TTS
from utils.utils import LoadModel


COMMAND_MAP = {
    "emergencia": "emergency_stop",
    "reanuda": "resume",
    "reanudar": "resume",
    "continua": "resume",
    "continúa": "resume",
    "avanza": "forward",
    "abanza": "forward",
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
    "modo a voz": "teleop_voice",
    "mata voz": "teleop_voice",
    "teleop voz": "teleop_voice",
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

NAVIGATE_PREFIXES = [
    "ve a", "ir a", "ve para", "ir para", "ve hacia", "ir hacia",
    "navega a", "navega para", "navega hacia",
    "dirígete a", "dirigete a", "dirígete para", "dirigete para",
    "dirígete hacia", "dirigete hacia",
]

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

EMERGENCY_ALIASES = ["emergencia", "emergensia", "emerhencia", "emergencias"]
EMERGENCY_COMMAND = "emergency_stop"
TRAILING_NOISE = " .,!¡¿?;:"
ARTICLES = ("la ", "el ", "los ", "las ", "al ")
LOCATION_ALIASES = {
    "repiza": [
        "repiza", "repisa", "re pisa", "re piza", "repizas", "repisas",
        "la repiza", "la repisa", "estante", "el estante", "anaquel",
        "el anaquel",
    ],
    "estación": [
        "estación", "estacion", "estaciones", "estasion", "estación.",
        "la estación", "la estacion", "parada", "la parada", "base",
        "la base",
    ],
    "estacion": [
        "estación", "estacion", "estaciones", "estasion", "la estación",
        "la estacion", "parada", "la parada", "base", "la base",
    ],
    "cocina": [
        "cocina", "cosina", "cozina", "la cocina", "cocinas", "concina",
    ],
    "inicio": [
        "inicio", "inició", "inicia", "el inicio", "punto inicial",
        "salida", "la salida",
    ],
    "fin": [
        "fin", "final", "el fin", "punto final", "meta", "la meta",
        "destino final",
    ],
    "banda": [
        "banda", "vanda", "la banda", "cinta", "la cinta", "banda transportadora",
    ],
}


def normalize_text(text: str) -> str:
    text = text.strip().lower()
    text = "".join(
        c for c in unicodedata.normalize("NFD", text)
        if unicodedata.category(c) != "Mn"
    )
    text = text.replace("ñ", "n")
    text = re.sub(r"[^a-z0-9]+", " ", text)
    text = re.sub(r"\s+", " ", text)
    return text.strip()


def strip_articles(text: str) -> str:
    cleaned = text.strip(TRAILING_NOISE).strip()
    normalized = normalize_text(cleaned)
    for art in ARTICLES:
        if normalized.startswith(normalize_text(art)):
            return cleaned[len(art):].strip(TRAILING_NOISE).strip()
    return cleaned


def _name_variants(name: str) -> set[str]:
    normalized = normalize_text(name)
    variants = {normalized}
    variants.add(normalized.replace("s", "z"))
    variants.add(normalized.replace("z", "s"))
    variants.add(normalized.replace("b", "v"))
    variants.add(normalized.replace("v", "b"))
    variants.add(normalized.replace(" ", ""))
    return {v for v in variants if v}


def resolve_waypoint_alias(target: str, waypoint_names: list[str]) -> Optional[str]:
    """Map noisy STT target text to a canonical waypoint/location name."""
    normalized_target = normalize_text(strip_articles(target))
    if not normalized_target:
        return None

    candidates: list[tuple[str, str]] = []
    known_names = list(dict.fromkeys([*waypoint_names, *LOCATION_ALIASES.keys()]))
    for name in known_names:
        for variant in _name_variants(name):
            candidates.append((variant, name))
        for alias in LOCATION_ALIASES.get(name, []):
            for variant in _name_variants(alias):
                candidates.append((variant, name))

    for variant, name in candidates:
        if normalized_target == variant:
            return name

    for variant, name in sorted(candidates, key=lambda item: len(item[0]), reverse=True):
        if variant and variant in normalized_target:
            return name

    best_name = None
    best_score = 0.0
    for variant, name in candidates:
        score = SequenceMatcher(None, normalized_target, variant).ratio()
        if score > best_score:
            best_name = name
            best_score = score
    if best_name and best_score >= 0.82:
        return best_name
    return None


def strip_wake_words(text: str, variants: list[str]) -> str:
    cleaned = text
    normalized = normalize_text(cleaned)
    for variant in sorted(variants, key=len, reverse=True):
        needle = normalize_text(variant)
        if needle and normalized.startswith(needle):
            return cleaned[len(variant):].strip(" .,!¡¿?")
    return cleaned


def parse_speech_command(text: str, waypoint_names: list[str]) -> tuple[Optional[str], Optional[str]]:
    """Return (command_str, target_or_None) from a transcription."""
    if not text:
        return None, None

    raw = text.strip().lower()
    t = normalize_text(raw)

    for alias in EMERGENCY_ALIASES:
        if normalize_text(alias) in t:
            return EMERGENCY_COMMAND, None

    for prefix in sorted(SAVE_PREFIXES, key=len, reverse=True):
        normalized_prefix = normalize_text(prefix)
        if t.startswith(normalized_prefix):
            tail = raw[len(prefix):].strip(" .,!¡¿?")
            for art in ("la ", "el ", "los ", "las "):
                if tail.startswith(art):
                    tail = tail[len(art):]
                    break
            if tail:
                return "save_location", tail
            return None, None

    for prefix in NAVIGATE_PREFIXES:
        normalized_prefix = normalize_text(prefix)
        if t.startswith(normalized_prefix + " ") or t == normalized_prefix:
            tail = strip_articles(raw[len(prefix):])
            resolved = resolve_waypoint_alias(tail, waypoint_names)
            if resolved:
                return "navigate", resolved
            if tail:
                return "navigate", tail.strip(TRAILING_NOISE).strip()
            return None, None

    for key in sorted(COMMAND_MAP, key=len, reverse=True):
        if normalize_text(key) in t:
            return COMMAND_MAP[key], None

    return None, None


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__("voice_command_node")
        logging.basicConfig(level=logging.INFO)

        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("rover_bt")

        self.declare_parameter("wake_word", "ok robot")
        self.declare_parameter("wake_variants", ["ok robot", "okay robot", "hey robot"])
        self.declare_parameter("wake_timeout_sec", 10.0)
        self.declare_parameter("stt_model_name", "base")
        self.declare_parameter(
            "waypoints_file",
            os.path.join(share, "config", "waypoints.yaml"),
        )
        self.declare_parameter("command_topic", "/rover_bt/commands")
        self.declare_parameter("tts_topic", "/rover_bt/tts/say")
        self.declare_parameter("audio_device_index", -1)
        self.declare_parameter("audio_sample_rate", 16000)
        self.declare_parameter("audio_channels", 1)

        self.wake_word = self.get_parameter("wake_word").value.lower()
        self.wake_variants = list(self.get_parameter("wake_variants").value)
        self.wake_timeout = float(self.get_parameter("wake_timeout_sec").value)
        self.stt_model_name = self.get_parameter("stt_model_name").value
        audio_device_index = self.get_parameter("audio_device_index").value
        audio_sample_rate = self.get_parameter("audio_sample_rate").value
        audio_channels = self.get_parameter("audio_channels").value

        self._stop_event = threading.Event()
        self._speaking = threading.Event()

        self.waypoint_names = self._load_waypoints(self.get_parameter("waypoints_file").value)
        self.get_logger().info(f"[voice] Waypoints vocabulary: {self.waypoint_names}")

        loader = LoadModel()
        wake_model = loader.ensure_model("wake_word")[0]
        stt_model = loader.ensure_model("stt")[1]
        tts_models = loader.ensure_model("tts")
        if len(tts_models) < 2:
            raise RuntimeError("La seccion 'tts' de models.yml debe incluir modelo .onnx y .json")

        self.audio_listener = AudioListener(
            device_index=audio_device_index,
            sample_rate=audio_sample_rate,
            channels=audio_channels,
        )
        self.wake_detector = WakeWord(
            str(wake_model),
            wake_word=self.wake_word,
            variants=self.wake_variants,
            listen_seconds=self.wake_timeout,
            sample_rate=audio_sample_rate,
        )
        self.wake_detector.on_say = (
            lambda text: self.get_logger().info(f"[wake_word] {text}")
        )
        self.stt = SpeechToText(str(stt_model), model_name=self.stt_model_name)
        self.tts = TTS(str(tts_models[0]), str(tts_models[1]))

        self.cmd_pub = self.create_publisher(
            Command,
            self.get_parameter("command_topic").value,
            10,
        )
        self.tts_sub = self.create_subscription(
            String,
            self.get_parameter("tts_topic").value,
            self._on_tts,
            10,
        )

        # Single-threaded TTS worker: serialises all playback so two concurrent
        # Speak nodes never share the PyAudio stream, which would corrupt it and
        # silently kill all subsequent speech.
        self._tts_queue: queue.Queue = queue.Queue()
        self._tts_thread = threading.Thread(target=self._tts_worker, daemon=True)
        self._tts_thread.start()

        self.audio_thread = threading.Thread(target=self._audio_loop, daemon=True)
        self.audio_thread.start()

        self.get_logger().info(
            f"[voice] Voice bridge ready - say '{self.wake_word}' to activate."
        )

    def destroy_node(self):
        self._stop_event.set()
        try:
            self.audio_listener.delete()
        except Exception:
            pass
        try:
            self.tts.stop_tts()
        except Exception:
            pass
        return super().destroy_node()

    def _on_tts(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self._tts_queue.put(text)

    def _tts_worker(self):
        while not self._stop_event.is_set():
            try:
                text = self._tts_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self._speak(text)

    def _speak(self, text: str):
        self._speaking.set()
        try:
            self.get_logger().info(f"[tts] {text}")
            audio = self.tts.synthesize(text)
            self.tts.play_audio_with_amplitude(audio)
        except Exception as exc:
            self.get_logger().error(f"TTS playback failed: {exc}")
        finally:
            self._speaking.clear()

    def _load_waypoints(self, path: str) -> list[str]:
        if not os.path.exists(path):
            return []
        try:
            with open(path, encoding="utf-8") as f:
                data = yaml.safe_load(f)
            return list(data.get("waypoints", {}).keys()) if data else []
        except Exception as exc:
            self.get_logger().error(f"Failed to load waypoints: {exc}")
            return []

    def _audio_loop(self):
        try:
            self.audio_listener.start_stream()
            while rclpy.ok() and not self._stop_event.is_set():
                if self._speaking.is_set():
                    time.sleep(0.05)
                    continue

                frame = self.audio_listener.read_frame(self.wake_detector.frame_samples)
                audio = self.wake_detector.wake_word_detector(frame)
                if audio is not None:
                    self.get_logger().info("[wake_word] audio captured; sending to STT")
                    self._process_audio(audio)
        except Exception as exc:
            self.get_logger().error(f"Audio bridge crashed: {exc}")
        finally:
            try:
                self.audio_listener.stop_stream()
            except Exception:
                pass

    def _process_audio(self, audio: bytes):
        self.get_logger().info("[stt] transcribing captured speech")
        text = self.stt.worker_loop(audio)
        if not text:
            self.get_logger().info("[stt] empty transcription")
            return

        text = strip_wake_words(text, self.wake_variants)
        self.get_logger().info(f"[stt] heard: '{text}'")
        cmd, target = parse_speech_command(text, self.waypoint_names)

        if cmd:
            self._publish(cmd, target)
            return

        self.get_logger().info("[voice] No command matched; wake word required again.")

    def _publish(self, command: str, target: Optional[str] = None):
        msg = Command()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = "voice"
        msg.priority = Command.PRIORITY_VOICE
        if command == EMERGENCY_COMMAND:
            msg.priority = Command.PRIORITY_EMERGENCY
        msg.command = command
        msg.target = target if target else ""
        self.cmd_pub.publish(msg)
        self.get_logger().info(
            f"[voice] Command published: '{command}'" + (f" -> '{target}'" if target else "")
        )


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
