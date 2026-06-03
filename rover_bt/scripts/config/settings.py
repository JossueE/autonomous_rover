import os

# Global
LANGUAGE = "es"
MODELS_PATH = "config/models.yml"

# Audio Listener
AUDIO_LISTENER_DEVICE_ID: int | None = None
AUDIO_LISTENER_CHANNELS = 1
AUDIO_LISTENER_SAMPLE_RATE = 16000
AUDIO_LISTENER_FRAMES_PER_BUFFER = 1000

# LLM placeholders retained for compatibility with the local voice package.
USE_LLM = True
CONTEXT_LLM = 1024
THREADS_LLM = os.cpu_count() or 8
N_BACH_LLM = 512
GPU_LAYERS_LLM = 0
MAX_MOVE_DISTANCE_LLM = 5.0
CHAT_FORMAT_LLM = "chatml-function-calling"

# Information - data
FUZZY_LOGIC_ACCURACY_GENERAL_RAG = 0.70
FUZZY_LOGIC_ACCURACY_POSE = 0.70
PATH_GENERAL_RAG = "config/data/general_rag.json"
PATH_POSES = "config/data/poses.json"

# Audio Publisher
AUDIO_PUBLISHER_DEVICE_ID = -1
AUDIO_PUBLISHER_FRAMES_PER_BUFFER = 256
AUDIO_PUBLISHER_DEBUG = True

# Text-to-Speech
SAMPLE_RATE_TTS = 24000
DEVICE_SELECTOR_TTS = "cpu"
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
    "Octybot, ok robot, emergencia, ve a la enfermeria, "
    "avanza, detente, derecha, izquierda"
)

# Wake-Word
ACTIVATION_PHRASE_WAKE_WORD = "ok robot"
VARIANTS_WAKE_WORD = ["ok robot", "okay robot", "hey robot"]

# Avatar
AVATAR = False
