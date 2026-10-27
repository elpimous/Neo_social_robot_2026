#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════════
#  get_models.sh — télécharge les modèles du node listen (≈ 650 Mo)
#  Author : Vincent FOUCAULT — Octobre 2026
#  Usage  : ~/qbo_ws/src/listen_and_talk/scripts/get_models.sh
#           puis : colcon build --packages-select listen_and_talk
# ═══════════════════════════════════════════════════════════════════════════════
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/models/listen"
mkdir -p "$DIR" && cd "$DIR"
BASE=https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models

for f in silero_vad.onnx ten-vad.onnx; do
  [ -f "$f" ] || { echo "→ $f"; wget -q --show-progress "$BASE/$f"; }
done

P=sherpa-onnx-nemo-parakeet-tdt-0.6b-v3-int8
if [ ! -f "$P/encoder.int8.onnx" ]; then
  echo "→ $P (≈ 640 Mo)"
  wget -q --show-progress "$BASE/$P.tar.bz2"
  tar xjf "$P.tar.bz2" && rm -f "$P.tar.bz2"
  rm -rf "$P/test_wavs"
fi
echo "✅ Modèles prêts dans $DIR"
ls -la "$DIR"
