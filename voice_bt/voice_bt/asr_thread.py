"""Background thread running bilingual (ES/EN) Vosk ASR.

Writes recognized commands onto the py_trees blackboard so behavior tree
leaves can act on them. Modelled after the existing
`bilingual_speech_processor.py` script in /home/ggm/Documents/octavo/voice/.
"""

import json
import queue
import threading

import sounddevice as sd
from vosk import Model, KaldiRecognizer

from voice_bt.command_map import parse


class BilingualASR(threading.Thread):
    def __init__(self, logger, blackboard, es_model_path, en_model_path,
                 waypoint_names, sample_rate=16000, blocksize=8000):
        super().__init__(daemon=True)
        self._logger = logger
        self._bb = blackboard
        self._sample_rate = sample_rate
        self._blocksize = blocksize
        self._waypoint_names = list(waypoint_names)
        self._audio_q = queue.Queue()
        self._stop_evt = threading.Event()

        self._logger.info(f"Loading Spanish Vosk model from {es_model_path}")
        self._model_es = Model(es_model_path)
        self._logger.info(f"Loading English Vosk model from {en_model_path}")
        self._model_en = Model(en_model_path)

        self._rec_es = KaldiRecognizer(self._model_es, sample_rate)
        self._rec_en = KaldiRecognizer(self._model_en, sample_rate)

    def stop(self):
        self._stop_evt.set()

    def _audio_cb(self, indata, frames, time_info, status):
        if status:
            self._logger.warn(f"Audio status: {status}")
        self._audio_q.put(bytes(indata))

    def _dispatch(self, text, lang):
        cmd, wp = parse(text, self._waypoint_names)
        if cmd is None:
            return
        self._logger.info(f"[{lang}] '{text}' -> command={cmd} waypoint={wp}")
        # Set blackboard atomically: waypoint first, then command.
        if wp is not None:
            self._bb.set("voice_bt/target_waypoint", wp)
        self._bb.set("voice_bt/command", cmd)

    def run(self):
        self._logger.info("Starting bilingual ASR audio stream")
        try:
            with sd.RawInputStream(samplerate=self._sample_rate,
                                   blocksize=self._blocksize,
                                   dtype='int16', channels=1,
                                   callback=self._audio_cb):
                while not self._stop_evt.is_set():
                    try:
                        chunk = self._audio_q.get(timeout=0.5)
                    except queue.Empty:
                        continue

                    if self._rec_es.AcceptWaveform(chunk):
                        text = json.loads(self._rec_es.Result()).get("text", "")
                        if text:
                            self._dispatch(text, "ES")
                    if self._rec_en.AcceptWaveform(chunk):
                        text = json.loads(self._rec_en.Result()).get("text", "")
                        if text:
                            self._dispatch(text, "EN")
        except Exception as e:
            self._logger.error(f"ASR thread crashed: {e}")
