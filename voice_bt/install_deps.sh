#!/bin/bash
# Download voice model assets and install Python runtime deps for voice_bt.
# Run once after cloning the repo, before building with colcon.
#
# Usage: bash src/voice_bt/install_deps.sh
#
# Assets downloaded:
#   voice_assets/model_en/   — Vosk English ASR model
#   voice_assets/model_es/   — Vosk Spanish ASR model
#   voice_assets/piper/      — Piper TTS binary + shared libs
#   voice_assets/*.onnx      — Piper TTS voice model (es_MX-claude-high)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASSETS="$SCRIPT_DIR/voice_assets"

# ── Vosk models ───────────────────────────────────────────────────────────────
# Find model ZIPs at: https://alphacephei.com/vosk/models
VOSK_EN_URL="https://alphacephei.com/vosk/models/vosk-model-en-us-0.22-lgraph.zip"
VOSK_ES_URL="https://alphacephei.com/vosk/models/vosk-model-es-0.42.zip"

download_vosk_model() {
    local url="$1"
    local dest_name="$2"   # model_en or model_es
    local dest="$ASSETS/$dest_name"

    if [ -d "$dest" ]; then
        echo "  $dest_name already present, skipping."
        return
    fi

    local zip_file
    zip_file="$(mktemp /tmp/vosk_model_XXXXXX.zip)"
    echo "  Downloading $(basename "$url") ..."
    curl -L --progress-bar -o "$zip_file" "$url"

    echo "  Extracting to $dest ..."
    local tmp_dir
    tmp_dir="$(mktemp -d)"
    unzip -q "$zip_file" -d "$tmp_dir"
    rm "$zip_file"

    # The zip contains a single top-level directory — rename it to dest_name.
    local extracted
    extracted="$(find "$tmp_dir" -mindepth 1 -maxdepth 1 -type d | head -1)"
    mv "$extracted" "$dest"
    rm -rf "$tmp_dir"
    echo "  $dest_name ready."
}

# ── Piper TTS binary ──────────────────────────────────────────────────────────
# Releases: https://github.com/rhasspy/piper/releases
PIPER_VERSION="2023.11.14-2"
PIPER_ARCHIVE="piper_linux_x86_64.tar.gz"
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
# Browse voices at: https://huggingface.co/rhasspy/piper-voices
VOICE_MODEL="es_MX-claude-high.onnx"
VOICE_CONFIG="es_MX-claude-high.onnx.json"
VOICE_BASE_URL="https://huggingface.co/rhasspy/piper-voices/resolve/main/es/es_MX/claude/high"

download_voice_model() {
    local model_path="$ASSETS/$VOICE_MODEL"
    local config_path="$ASSETS/$VOICE_CONFIG"

    if [ -f "$model_path" ]; then
        echo "  $VOICE_MODEL already present, skipping."
        return
    fi

    echo "  Downloading $VOICE_MODEL ..."
    curl -L --progress-bar -o "$model_path" "$VOICE_BASE_URL/$VOICE_MODEL"
    curl -L --progress-bar -o "$config_path" "$VOICE_BASE_URL/$VOICE_CONFIG"
    echo "  $VOICE_MODEL ready."
}

# ── Python runtime deps ───────────────────────────────────────────────────────
install_python_deps() {
    echo "  Installing Python packages (vosk, sounddevice) ..."
    pip3 install --break-system-packages vosk sounddevice
    echo "  Python deps ready."
}

# ── Main ──────────────────────────────────────────────────────────────────────
echo "=== voice_bt: setting up assets ==="
mkdir -p "$ASSETS"

echo "[1/4] Vosk English model"
download_vosk_model "$VOSK_EN_URL" "model_en"

echo "[2/4] Vosk Spanish model"
download_vosk_model "$VOSK_ES_URL" "model_es"

echo "[3/4] Piper TTS binary"
download_piper

echo "[4/4] Piper voice model (es_MX-claude-high)"
download_voice_model

echo "[+] Python deps"
install_python_deps

echo ""
echo "=== Done. Build with: colcon build --packages-select voice_bt ==="
