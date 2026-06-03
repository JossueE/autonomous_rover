#!/bin/bash
# Download voice model assets and install Python runtime deps for rover_bt.
# Run once after cloning the repo, before building with colcon.
#
# Usage: bash src/rover_bt/scripts/install_voice_assets.sh
#
# Assets downloaded/created inside src/rover_bt/voice_assets/:
#   whisper_cache/  — faster-whisper model cache (base, auto-downloaded)
#   piper/          — Piper TTS binary + shared libs
#   *.onnx          — Piper TTS voice model (es_MX-claude-high)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASSETS="$SCRIPT_DIR/../voice_assets"

# ── Piper TTS binary ──────────────────────────────────────────────────────────
PIPER_VERSION="2023.11.14-2"
case "$(uname -m)" in
    x86_64)          PIPER_ARCHIVE="piper_linux_x86_64.tar.gz" ;;
    aarch64|arm64)   PIPER_ARCHIVE="piper_linux_aarch64.tar.gz" ;;
    armv7l)          PIPER_ARCHIVE="piper_linux_armv7l.tar.gz" ;;
    *) echo "  WARNING: unknown arch $(uname -m); defaulting to x86_64 Piper build" >&2
       PIPER_ARCHIVE="piper_linux_x86_64.tar.gz" ;;
esac
PIPER_URL="https://github.com/rhasspy/piper/releases/download/${PIPER_VERSION}/${PIPER_ARCHIVE}"

download_piper() {
    local dest="$ASSETS/piper"

    if [ -d "$dest" ]; then
        echo "  piper/ already present, skipping."
        return
    fi

    local tar_file
    tar_file="$(mktemp /tmp/piper_XXXXXX.tar.gz)"
    echo "  Downloading Piper $PIPER_VERSION ..."
    curl -L --progress-bar -o "$tar_file" "$PIPER_URL"

    echo "  Extracting to $dest ..."
    mkdir -p "$dest"
    tar -xzf "$tar_file" -C "$dest" --strip-components=1
    rm "$tar_file"

    chmod +x "$dest/piper"
    echo "  piper/ ready."
}

# ── Piper voice model (es_MX-claude-high) ─────────────────────────────────────
VOICE_MODEL="es_MX-claude-high.onnx"
VOICE_CONFIG="es_MX-claude-high.onnx.json"
VOICE_BASE_URL="https://huggingface.co/rhasspy/piper-voices/resolve/main/es/es_MX/claude/high"

download_voice_model() {
    local model_path="$ASSETS/$VOICE_MODEL"

    if [ -f "$model_path" ]; then
        echo "  $VOICE_MODEL already present, skipping."
        return
    fi

    echo "  Downloading $VOICE_MODEL ..."
    curl -L --progress-bar -o "$model_path" "$VOICE_BASE_URL/$VOICE_MODEL"
    curl -L --progress-bar -o "$ASSETS/$VOICE_CONFIG" "$VOICE_BASE_URL/$VOICE_CONFIG"
    echo "  $VOICE_MODEL ready."
}

# ── Python runtime deps ───────────────────────────────────────────────────────
install_python_deps() {
    # sounddevice loads PortAudio (libportaudio.so.2) at import; pip does not
    # bundle the native lib on Linux/Jetson, so install it via apt first.
    if ! ldconfig -p 2>/dev/null | grep -q 'libportaudio\.so\.2'; then
        echo "  Installing PortAudio system library (libportaudio2) ..."
        sudo apt-get update && sudo apt-get install -y libportaudio2
    fi

    echo "  Installing Python packages (faster-whisper, webrtcvad, sounddevice) ..."
    # --break-system-packages only exists in pip >= 23.0.1 and is only needed on
    # PEP 668 "externally-managed" distros. Older pip (e.g. JetPack) rejects the flag.
    local break_flag=""
    if pip3 install --help 2>/dev/null | grep -q -- '--break-system-packages'; then
        break_flag="--break-system-packages"
    fi
    pip3 install $break_flag faster-whisper webrtcvad sounddevice
    echo "  Python deps ready."
}

# ── Pre-warm Whisper model cache ──────────────────────────────────────────────
prewarm_whisper_models() {
    local cache_dir="$ASSETS/whisper_cache"
    mkdir -p "$cache_dir"
    echo "  Pre-downloading Whisper base model to $cache_dir ..."
    python3 - <<PYEOF
from faster_whisper import WhisperModel
import os
cache = "$cache_dir"
for name in ("base",):
    print(f"  Fetching {name} ...")
    WhisperModel(name, device="cpu", compute_type="int8", download_root=cache)
    print(f"  {name} cached.")
PYEOF
    echo "  Whisper models ready."
}

# ── Main ──────────────────────────────────────────────────────────────────────
echo "=== rover_bt: setting up voice assets ==="
mkdir -p "$ASSETS"

echo "[1/4] Python deps"
install_python_deps

echo "[2/4] Whisper model cache (base)"
prewarm_whisper_models

echo "[3/4] Piper TTS binary"
download_piper

echo "[4/4] Piper voice model (es_MX-claude-high)"
download_voice_model

echo ""
echo "=== Done. Build with: colcon build --packages-select rover_bt ==="
