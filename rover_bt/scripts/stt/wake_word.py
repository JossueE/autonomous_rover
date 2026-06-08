from __future__ import annotations

import json
import logging
import threading
from collections import deque
from typing import Iterable

import vosk
import webrtcvad

from config.settings import (
    ACTIVATION_PHRASE_WAKE_WORD,
    AUDIO_LISTENER_CHANNELS,
    AUDIO_LISTENER_SAMPLE_RATE,
    LISTEN_SECONDS_STT,
    MIN_SILENCE_MS_TO_DRAIN_STT,
    VARIANTS_WAKE_WORD,
)


class WakeWord:
    def __init__(
        self,
        model_path: str,
        wake_word: str | None = None,
        variants: Iterable[str] | None = None,
        listen_seconds: float | None = None,
        sample_rate: int | None = None,
    ) -> None:
        self.log = logging.getLogger("Wake_Word")
        self.wake_word = wake_word or ACTIVATION_PHRASE_WAKE_WORD
        self.listen_seconds = listen_seconds or LISTEN_SECONDS_STT
        self.sample_rate = sample_rate if sample_rate is not None else AUDIO_LISTENER_SAMPLE_RATE
        self.variants = list(variants) if variants else list(VARIANTS_WAKE_WORD)
        if self.wake_word not in self.variants:
            self.variants.insert(0, self.wake_word)

        self.on_say = (lambda s: print(f"[wake_word] {s}"))

        grammar = json.dumps(self.variants, ensure_ascii=False)
        self.model = vosk.Model(model_path)
        self.rec = vosk.KaldiRecognizer(self.model, self.sample_rate, grammar)

        self.listening_confirm = False
        self.listening = False

        self.partial_hits = 0
        self.required_hits = 10
        self.silence_frames_to_drain = MIN_SILENCE_MS_TO_DRAIN_STT

        self.vad = webrtcvad.Vad(3)
        self.frame_ms = 10
        self.frame_samples = int(self.sample_rate / 1000 * self.frame_ms)

        self.lock = threading.Lock()
        self.buffer = deque()
        self.size = 0
        self.max = int(self.listen_seconds * self.sample_rate * AUDIO_LISTENER_CHANNELS * 2)
        self.max_2 = int(1 * self.sample_rate * AUDIO_LISTENER_CHANNELS * 2)

    def force_listening(self) -> None:
        """Start confirmed capture without requiring the wake word."""
        if self.listening_confirm:
            return
        self.listening = True
        self.listening_confirm = True
        self.partial_hits = 0

    def wake_word_detector(self, frame: bytes) -> None | bytes:
        """Process one 10 ms PCM int16 mono frame for wake-word detection."""
        flag = True if self.vad.is_speech(frame, self.sample_rate) else False

        if (self.listening or self.listening_confirm) and flag:
            drained = self.buffer_add(frame)
            if drained is not None:
                return drained

        if not flag:
            if self.partial_hits > -self.silence_frames_to_drain:
                self.partial_hits -= 1
            if (self.listening or self.listening_confirm) and self.partial_hits <= -self.silence_frames_to_drain:
                self.partial_hits = 0
                if self.listening_confirm and self.size > 0:
                    return self.buffer_drain()
                if self.size > 0:
                    self.on_say("Hubo una detección pero no se confirmó, limpiando buffer")
                    self.buffer_clear()
                else:
                    self.buffer_clear(silent=True)
                return

        if self.rec.AcceptWaveform(frame):
            result = json.loads(self.rec.Result() or "{}")
            text = (result.get("text") or "").lower().strip()
            if text and self.matches_wake(text):
                self.log.info(f"[FULL] Wake word: {text!r}")
                if not self.listening_confirm:
                    self.listening_confirm = True
                    self.listening = True
                    self.on_say("Confirmo grabacion")
                self.partial_hits = 0
                return
            self.partial_hits = 0

        else:
            partial = json.loads(self.rec.PartialResult() or "{}").get("partial", "").lower().strip()
            if partial:
                if self.matches_wake(partial):
                    if not self.listening:
                        self.listening = True
                        self.on_say("Empiezo a grabar (primer partial)")
                        drained = self.buffer_add(frame) if flag else None
                        if drained is not None:
                            return drained
                    self.partial_hits += 1

                    if self.partial_hits >= self.required_hits:
                        self.log.info(f"[PARTIAL] Wake word: {partial!r}")
                        self.partial_hits = 0
                        return
                else:
                    self.partial_hits = 0

    def buffer_add(self, frame: bytes) -> None | bytes:
        with self.lock:
            self.buffer.append(frame)
            self.size += len(frame)
        if self.size > self.max and self.listening_confirm:
            return self.buffer_drain()
        if self.size > self.max_2 and self.listening and not self.listening_confirm:
            self.on_say("Límite de tiempo alcanzado sin confirmación, limpiando buffer")
            self.buffer_clear()
        return None

    def buffer_clear(self, silent: bool = False) -> None:
        """Clear the audio buffer and reset flags."""
        if not silent:
            self.on_say("Limpiando buffer")
        self.listening = False
        self.listening_confirm = False
        with self.lock:
            self.buffer.clear()
        self.size = 0

    def buffer_drain(self) -> bytes:
        """Return all buffered audio as one bytes object and clear the buffer."""
        self.on_say("Envío Información a STT")

        with self.lock:
            data = b"".join(self.buffer)
            self.buffer.clear()

        self.on_say("Limpio el buffer")
        self.size = 0
        self.listening = False
        self.listening_confirm = False
        return data

    def norm(self, s: str) -> str:
        """Normalize string: lowercase, remove accents."""
        s = s.lower()
        return (s.replace("á","a").replace("é","e").replace("í","i")
                .replace("ó","o").replace("ú","u").replace("ü","u"))

    def matches_wake(self, text: str) -> bool:
        """Return True if text matches any variant of the wake word."""
        t = self.norm(text)
        for v in self.variants:
            if self.norm(v) in t:
                return True
        return False


if "__main__" == __name__:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s %(asctime)s] [%(name)s] %(message)s")

    from utils.utils import LoadModel
    from stt.audio_listener import AudioListener

    model = LoadModel()
    audio_listener = AudioListener()
    ww = WakeWord(str(model.ensure_model("wake_word")[0]))
    audio_listener.start_stream()

    try: 
        print("Este es el nodo de prueba del Wake Word con Audio Listener.\n")
        while True:
            result = audio_listener.read_frame(ww.frame_samples)
            n_result = ww.wake_word_detector(result)
            if n_result is not None:
                print(f"Wake Word detectada, enviando {len(n_result)} bytes de audio para STT")

    except KeyboardInterrupt:
        audio_listener.delete()
        print(" Saliendo")
        exit(0)
