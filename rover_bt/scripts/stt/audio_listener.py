import logging
from typing import Optional

import pyaudio

from config.settings import (
    AUDIO_LISTENER_CHANNELS,
    AUDIO_LISTENER_DEVICE_ID,
    AUDIO_LISTENER_FRAMES_PER_BUFFER,
    AUDIO_LISTENER_SAMPLE_RATE,
)


def define_device_id(
    pa: Optional[pyaudio.PyAudio] = None,
    prefered: Optional[int] = AUDIO_LISTENER_DEVICE_ID,
    log: Optional[logging.Logger] = None,
) -> Optional[int]:
    """Define the device id to use for audio input."""
    log = log or logging.getLogger("AudioListener")
    if prefered is not None and prefered != -1:
        log.info("Usando dispositivo de audio preferido (ID: %d)", prefered)
        return prefered

    if pa is None:
        log.warning("Pyaudio instance no iniciado, no se puede listar dispositivos.")
        return None

    devices = []
    fallback = None
    for i in range(pa.get_device_count()):
        try:
            info = pa.get_device_info_by_index(i)
        except Exception:
            continue
        if info.get("maxInputChannels", 0) <= 0:
            continue
        if fallback is None:
            fallback = i
        name = info.get("name", "")
        rate = int(info.get("defaultSampleRate", 0))
        log.info(
            "[%d] %s (in=%s, rate=%d)",
            i,
            name,
            info["maxInputChannels"],
            rate,
        )
        devices.append((i, name.lower(), rate))

    # Preference 1: Look for "pulse", "pipewire" or "default" (sound server routing)
    for i, name, rate in devices:
        if "pulse" in name or "pipewire" in name or name == "default":
            log.info("Usando dispositivo de servidor de sonido por defecto: %d (%s)", i, name)
            return i

    # Preference 2: Look for any device named "sysdefault"
    for i, name, rate in devices:
        if "sysdefault" in name:
            log.info("Usando dispositivo sysdefault: %d (%s)", i, name)
            return i

    # Preference 3: Look for any device with native 16000Hz support (since that's our target)
    for i, name, rate in devices:
        if rate == 16000:
            log.info("Usando dispositivo con frecuencia de muestreo nativa de 16kHz: %d (%s)", i, name)
            return i

    if fallback is not None:
        log.info("Cae en el fallback del primer dispositivo de entrada disponible: %d", fallback)
    return fallback


class AudioListener:
    def __init__(
        self,
        device_index: Optional[int] = None,
        sample_rate: Optional[int] = None,
        channels: Optional[int] = None,
        frames_per_buffer: Optional[int] = None,
    ):
        self.log = logging.getLogger("AudioListener")
        self.sample_rate = sample_rate if sample_rate is not None else AUDIO_LISTENER_SAMPLE_RATE
        self.audio_interface = pyaudio.PyAudio()
        
        pref_device = device_index if device_index is not None else AUDIO_LISTENER_DEVICE_ID
        self.device_index = define_device_id(self.audio_interface, pref_device, self.log)
        
        self.channels = channels if channels is not None else AUDIO_LISTENER_CHANNELS
        self.frames_per_buffer = frames_per_buffer if frames_per_buffer is not None else AUDIO_LISTENER_FRAMES_PER_BUFFER
        self.stream = None
        self.log.info(
            "AudioListener initialized with device_index=%s, sample_rate=%s, "
            "channels=%s, frames_per_buffer=%s",
            self.device_index,
            self.sample_rate,
            self.channels,
            self.frames_per_buffer,
        )

    def start_stream(self):
        """Start the audio stream if not already started."""
        if self.stream is None:
            try:
                self.stream = self.audio_interface.open(
                    format=pyaudio.paInt16,
                    channels=self.channels,
                    rate=self.sample_rate,
                    input=True,
                    input_device_index=self.device_index,
                    frames_per_buffer=self.frames_per_buffer,
                )
            except Exception as e:
                self.log.error(
                    f"Failed to open audio stream on device {self.device_index} at {self.sample_rate} Hz: {e}"
                )
                if self.device_index != -1 and self.device_index is not None:
                    self.log.info("Trying to fall back to default sound server routing...")
                    fallback_device = define_device_id(self.audio_interface, prefered=-1, log=self.log)
                    if fallback_device is not None and fallback_device != self.device_index:
                        self.device_index = fallback_device
                        self.log.info(f"Retrying audio stream on device {self.device_index}")
                        self.stream = self.audio_interface.open(
                            format=pyaudio.paInt16,
                            channels=self.channels,
                            rate=self.sample_rate,
                            input=True,
                            input_device_index=self.device_index,
                            frames_per_buffer=self.frames_per_buffer,
                        )
                        return
                raise e

    def read_frame(self, frame_samples: int) -> bytes:
        """Read a frame of audio data from the stream."""
        if self.stream is None:
            raise RuntimeError("El Audio stream no se ha comenzado o está fallando la lectura.")
        return self.stream.read(frame_samples, exception_on_overflow=False)

    def stop_stream(self):
        """Stop the audio stream if it is running."""
        if self.stream is not None:
            try:
                self.stream.stop_stream()
            except Exception:
                pass
            try:
                self.stream.close()
            except Exception:
                pass
            self.stream = None

    def delete(self):
        """Clean up the audio interface and stream."""
        if self.stream is not None:
            self.stop_stream()
        self.audio_interface.terminate()

    def deleate(self):
        """Backward-compatible alias for the original example typo."""
        self.delete()


if "__main__" == __name__:
    al = AudioListener()
    time_test = 3
    al.start_stream()
    import time
    time.sleep(time_test)
    data = al.read_frame(3200)
    print(f"Durante {time_test} segundos, leiste {len(data)} bytes.")
    al.stop_stream()
