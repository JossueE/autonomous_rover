"""Non-blocking Piper TTS wrapper.

Spawns a thread that runs piper to synthesize speech to a temp WAV file and
plays it with aplay. Subsequent calls queue up so we don't talk over ourselves.
"""

import os
import queue
import subprocess
import tempfile
import threading


class PiperTTS:
    def __init__(self, logger, piper_bin, piper_model):
        self._logger = logger
        self._piper_bin = piper_bin
        self._piper_model = piper_model
        self._q = queue.Queue()
        self._stop_evt = threading.Event()
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def speak(self, text: str):
        if not text:
            return
        self._q.put(text)

    def stop(self):
        self._stop_evt.set()
        self._q.put(None)

    def _run(self):
        while not self._stop_evt.is_set():
            try:
                text = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            if text is None:
                break
            self._synthesize_and_play(text)

    def _synthesize_and_play(self, text: str):
        fd, tmp_path = tempfile.mkstemp(suffix=".wav")
        try:
            os.close(fd)
            self._logger.info(f"TTS: {text}")
            proc = subprocess.run(
                [self._piper_bin, "--model", self._piper_model,
                 "--output_file", tmp_path],
                input=text.encode("utf-8"),
                capture_output=True,
                timeout=15,
            )
            if proc.returncode != 0:
                self._logger.error(
                    f"piper failed: {proc.stderr.decode('utf-8', errors='ignore')}")
                return
            subprocess.run(["aplay", "-q", tmp_path], timeout=15)
        except FileNotFoundError as e:
            self._logger.error(f"TTS binary missing: {e}")
        except subprocess.TimeoutExpired:
            self._logger.error("TTS process timed out")
        except Exception as e:
            self._logger.error(f"TTS error: {e}")
        finally:
            if os.path.exists(tmp_path):
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
