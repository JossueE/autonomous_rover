<<<<<<< HEAD
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

=======
#!/usr/bin/env bash
>>>>>>> 95262d0 (new voice)
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
SOURCE_MODELS_FILE="$SCRIPT_DIR/../config/models.yml"
INSTALLED_MODELS_FILE="$SCRIPT_DIR/../../share/rover_bt/config/models.yml"

if [[ -f "$SOURCE_MODELS_FILE" ]]; then
  MODELS_FILE="$SOURCE_MODELS_FILE"
  CACHE_DIR="$SCRIPT_DIR/../voice_assets"
else
  MODELS_FILE="$INSTALLED_MODELS_FILE"
  WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/../../../.." &>/dev/null && pwd)"
  CACHE_DIR="${ROVER_BT_VOICE_ASSETS:-$WORKSPACE_ROOT/rover_bt/voice_assets}"
fi

have_cmd() { command -v "$1" >/dev/null 2>&1; }
die() { echo "ERROR: $*" >&2; exit 1; }

fetch() {
  local url="$1" out="$2"
  if have_cmd curl; then
    curl -L --fail --retry 3 -o "$out" "$url"
  else
    have_cmd wget || die "Necesitas curl o wget"
    wget -O "$out" "$url"
  fi
}

install_python_deps() {
<<<<<<< HEAD
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
=======
  echo "[deps] Instalando dependencias Python..."
  local break_flag=""
  if python3 -m pip install --help 2>/dev/null | grep -q -- '--break-system-packages'; then
    break_flag="--break-system-packages"
  fi
  python3 -m pip install $break_flag \
    pyaudio \
    vosk \
    webrtcvad \
    openai-whisper \
    piper-tts \
    numpy \
    pyyaml
}

download_file_or_zip() {
  local url="$1"
  local out_dir="$2"
  local name_hint="${3:-}"
  local fname ext tmp out dname

  [[ -n "$url" ]] || { echo "url vacia"; return 1; }
  [[ "$url" == -* ]] && { echo "URL invalida: empieza con '-'"; return 1; }
  mkdir -p "$out_dir"

  fname="$(basename "${url%%\?*}")"
  ext="${fname##*.}"

  if [[ "$ext" == "zip" && "$fname" != "$ext" ]]; then
    dname="${name_hint:-${fname%.zip}}"
    if [[ -d "$out_dir/$dname" ]]; then
      echo "  - ya existe: $out_dir/$dname"
      return 0
>>>>>>> 95262d0 (new voice)
    fi
    tmp="$out_dir/${name_hint:-pkg}.zip"
    echo "  - bajando ZIP: $url"
    fetch "$url" "$tmp"
    have_cmd unzip || die "Necesitas 'unzip' (sudo apt-get install -y unzip)"
    echo "  - descomprimiendo en $out_dir"
    unzip -q -o "$tmp" -d "$out_dir"
    rm -f "$tmp"
    return 0
  fi

  out="$out_dir/$fname"
  case "${name_hint##*.}" in
    pt|onnx|gguf|json|bin) out="$out_dir/$name_hint" ;;
  esac

  if [[ "$url" == *"drive.google.com"* || "$fname" == "uc" || "$fname" == "open" ]]; then
    [[ -n "$name_hint" ]] && out="$out_dir/$name_hint"
  fi

  if [[ "$out" == "$out_dir/$fname" && "$fname" != *.* && -n "$name_hint" ]]; then
    out="$out_dir/$name_hint"
  fi

  if [[ -f "$out" ]]; then
    echo "  - ya existe: $out"
  else
    echo "  - bajando: $url -> $out"
    fetch "$url" "$out"
  fi
}

<<<<<<< HEAD
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
=======
download_section() {
  local section="$1"
  local label="$2"
  local len

  len="$(yq -r "(.${section} // []) | length" "$MODELS_FILE" 2>/dev/null || echo 0)"
  if [[ -z "$len" || "$len" == "0" ]]; then
    return 0
  fi

  echo "[$label] Descargando modelos..."
  for i in $(seq 0 $((len - 1))); do
    local name url
    name="$(yq -r ".${section}[$i].name // \"\"" "$MODELS_FILE")"
    url="$(yq -r ".${section}[$i].url // \"\"" "$MODELS_FILE")"
    [[ -n "$url" && "$url" != "null" ]] || continue
    download_file_or_zip "$url" "$CACHE_DIR/$section" "$name"
  done
>>>>>>> 95262d0 (new voice)
}

[[ -f "$MODELS_FILE" ]] || die "No se encontro $MODELS_FILE."
have_cmd yq || die "Falta 'yq'. Instala con: sudo snap install yq"
mkdir -p "$CACHE_DIR"

echo "[*] Usando catalogo: $MODELS_FILE"
echo "[*] Cache: $CACHE_DIR"

install_python_deps
download_section "stt" "STT"
download_section "wake_word" "VOSK"
download_section "tts" "TTS"

<<<<<<< HEAD
echo "[2/4] Whisper model cache (base)"
prewarm_whisper_models

echo "[3/4] Piper TTS binary"
download_piper

echo "[4/4] Piper voice model (es_MX-claude-high)"
download_voice_model

echo ""
echo "=== Done. Build with: colcon build --packages-select rover_bt ==="
=======
echo "OK. Modelos listos en: $CACHE_DIR"
>>>>>>> 95262d0 (new voice)
