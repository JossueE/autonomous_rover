import io
import logging
import wave
from pathlib import Path

import numpy as np
import pyaudio
import torch
from piper.voice import PiperVoice, SynthesisConfig
from config.settings import (
    NAME_OF_OUTS_TTS,
    PATH_TO_SAVE_TTS,
    SAMPLE_RATE_TTS,
    SAVE_WAV_TTS,
    SPEED_TTS,
    VOLUME_TTS,
)


class TTS:
    def __init__(self, model_path: str, model_path_conf: str):
        self.log = logging.getLogger("Text-to-Speech")
        self.log.info("Loading Piper TTS model")
        self.voice = PiperVoice.load(model_path=model_path, config_path=model_path_conf)
        self.sample_rate = SAMPLE_RATE_TTS
        self.count_of_audios = 0
        self.out_path = Path(PATH_TO_SAVE_TTS) / Path(NAME_OF_OUTS_TTS) / Path(f"{NAME_OF_OUTS_TTS}_{self.count_of_audios}.wav")

        self.syn_config = SynthesisConfig(
            volume=VOLUME_TTS,
            length_scale=SPEED_TTS,
            noise_scale=1.0,
            noise_w_scale=1.0,
            normalize_audio=False, # use raw audio from voice
        )

        self.pa = None
        self.stream = None
        self.log.info("Text-To-Speech inicializado")

    def synthesize(self, text: str):
        """Convert text to speech using Piper and return mono float32 audio."""
        if not text:
            return None

        if SAVE_WAV_TTS:
            self.out_path.parent.mkdir(parents=True, exist_ok=True)
            with wave.open(str(self.out_path), "wb") as wav_file:
                self.voice.synthesize_wav(text, wav_file, syn_config=self.syn_config)
                self.count_of_audios += 1
                self.out_path = Path(PATH_TO_SAVE_TTS) / Path(NAME_OF_OUTS_TTS) / Path(f"{NAME_OF_OUTS_TTS}_{self.count_of_audios}.wav")

        mem = io.BytesIO()
        with wave.open(mem, "wb") as w:
            self.voice.synthesize_wav(text,w, syn_config=self.syn_config)

        mem.seek(0)
        with wave.open(mem, "rb") as r:
            frames = r.readframes(r.getnframes())
            pcm_i16 = np.frombuffer(frames, dtype=np.int16)
        audio_f32 = (pcm_i16.astype(np.float32) / 32768.0)
        return audio_f32

    def play_audio_with_amplitude(self, audio_data, amplitude_callback=None):
        """
        Plays the given float32 numpy array (single-channel).
        If amplitude_callback is provided, pass the amplitude
            of each chunk to it for mouth animation, etc.
        """
        if audio_data is None or len(audio_data) == 0:
            return

        if torch.is_tensor(audio_data):
            audio_data = audio_data.cpu().numpy()

        self.start_stream()

        audio_int16 = np.clip(audio_data * 32767.0, -32767.0, 32767.0).astype(np.int16)

        chunk_size = 1024
        idx = 0
        total_frames = len(audio_int16)

        while idx < total_frames:
            chunk_end = min(idx + chunk_size, total_frames)
            chunk = audio_int16[idx:chunk_end]
            self.stream.write(chunk.tobytes())

            if amplitude_callback:
                # amplitude = mean absolute value
                amplitude = np.abs(chunk.astype(np.float32)).mean()
                amplitude_callback(amplitude)

            idx += chunk_size
        self.stop_tts()
        return True

    def start_stream(self):
        """Start the audio stream if not already started."""
        if self.stream is None:
            self.pa = pyaudio.PyAudio()
            self.stream = self.pa.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                output=True,
            )

    def stop_tts(self):
        """Stop the stream"""
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None
        if self.pa is not None:
            self.pa.terminate()
            self.pa = None


if "__main__" == __name__:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s %(asctime)s] [%(name)s] %(message)s")

    from utils.utils import LoadModel
    
    model = LoadModel()
    tts = TTS(str(model.ensure_model("tts")[0]), str(model.ensure_model("tts")[1]))

    try: 
        print("Este es el nodo de prueba del Text to Speech - Presione Ctrl+C para salir\n")
        while True:
            text = input("Escribe algo: ")
            get_audio = tts.synthesize(text)
            tts.play_audio_with_amplitude(get_audio)

    except KeyboardInterrupt:
        tts.stop_tts()
        print(" Saliendo")
        exit(0)
