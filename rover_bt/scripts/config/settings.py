# Global
LANGUAGE = "es"
MODELS_PATH = "config/models.yml"

# Audio Listener
AUDIO_LISTENER_DEVICE_ID: int | None = None
AUDIO_LISTENER_CHANNELS = 1
AUDIO_LISTENER_SAMPLE_RATE = 16000
AUDIO_LISTENER_FRAMES_PER_BUFFER = 1000

# Text-to-Speech
SAMPLE_RATE_TTS = 24000
VOLUME_TTS = 2.0
SPEED_TTS = 1.0
PATH_TO_SAVE_TTS = "tts/audios"
NAME_OF_OUTS_TTS = "test"
SAVE_WAV_TTS = False

# Speech-to-Text
SAMPLE_RATE_STT = 16000
LISTEN_SECONDS_STT = 5.0
MIN_SILENCE_MS_TO_DRAIN_STT = 50
SELF_VOCABULARY_STT = (
    "comando, emergencia, detente, alto, para, "
    "reanuda, avanza, adelante, retrocede, atras, gira a la izquierda, "
    "gira a la derecha, modo voz, teleop voz, modo autonomo, patrulla, "
    "inicia mapeo, modo slam, mejora el mapa, deten el mapeo, guarda el mapa, "
    "estado, donde estas, sigueme, sigue a la persona, ve a, inicio, fin, "
    "cocina, estacion, repisa, banda"
)

# Wake-Word
ACTIVATION_PHRASE_WAKE_WORD = "comando"
VARIANTS_WAKE_WORD = ["comando"]
