# qbo_stt

**Speech-To-Text Node — Robot Néo**
NVIDIA Parakeet TDT 0.6B v3 · TensorRT FP16 · Silero VAD · ROS2 Humble
Inférence 100% TensorRT — zéro ONNX Runtime

> Vincent Foucault — Avril 2026

---

## Description

Package ROS2 implémentant un pipeline complet de reconnaissance vocale temps réel pour le robot Néo (plateforme QBo, Jetson Orin NX 16Go, JetPack 6).

L'ensemble de l'inférence est réalisé **exclusivement via TensorRT** — aucune dépendance ONNX Runtime à l'exécution. Le preprocesseur mel est implémenté en pur numpy/librosa, compatible avec les paramètres NeMo de Parakeet.

### Pipeline complet

```
Micro PyAudio (16kHz mono)
        │
        ▼ /is_audio_recording == True
   Silero VAD
   (TRT engine ou fallback énergie RMS)
        │ segment parole détecté
        ▼
   Mel Spectrogram NeMo-compatible
   (librosa : n_fft=512, hop=160, win=320, n_mels=128)
   (normalisation per_feature : mean/std par bande)
        │
        ▼
   Encoder TRT FP16          (~37 ms pour 5s audio)
   /engine_model/encoder.engine
        │
        ▼
   Decoder TDT TRT FP16      (~51 ms)
   /engine_model/decoder_joint.engine
   (greedy decode : DURATIONS=[0,1,2,3,4])
        │
        ▼
   /listened  (std_msgs/String)
```

### Performances (Jetson Orin NX 16Go)

| Métrique | Valeur |
|---|---|
| RTF moyen (5s audio) | **0.022** |
| Latence encoder | ~37 ms |
| Latence decoder | ~51 ms |
| Latence inférence totale | ~90 ms |
| Latence perçue (VAD 500ms silence) | **~600 ms** |
| Langues supportées | 25 européennes (FR, EN, DE, ES, IT...) |
| Throughput decoder (trtexec) | **2190 qps** |

---

## Prérequis

### Matériel
- NVIDIA Jetson Orin NX 16Go (Ampere, Compute Capability 8.7)
- JetPack 6.x (CUDA 12.6, TensorRT 10.3+)
- Microphone USB mono 16kHz

### Logiciel système
- ROS2 Humble
- Python 3.10
- TensorRT 10.3 (`/usr/src/tensorrt/`)
- CUDA 12.6

### Dépendances Python

```bash
pip install pycuda librosa pyaudio numpy
```

> **Note** : `onnxruntime` n'est **pas** requis pour l'inférence.

---

## Préparation des engines TensorRT

Les engines `.engine` doivent être générés **sur la Jetson cible** (non portables entre GPU différents). Effectuer cette opération une seule fois.


### 1. Télécharger les modèles FP32 (HuggingFace)
mkdir -p /home/nvidia/qbo_ws/src/qbo_stt/models/whisper_small_onnx
cd /home/nvidia/qbo_ws/src/qbo_stt/models/whisper_small

# Encoder
     wget https://huggingface.co/csukuangfj/sherpa-onnx-whisper-small/resolve/main/small-encoder.onnx

     # Decoder
     wget https://huggingface.co/csukuangfj/sherpa-onnx-whisper-small/resolve/main/small-decoder.onnx

# Tokens
wget https://huggingface.co/csukuangfj/sherpa-onnx-whisper-small/resolve/main/small-tokens.txt


# Encoder
/usr/src/tensorrt/bin/trtexec \
  --onnx=small-encoder.onnx \
  --saveEngine=small-encoder.engine \
  --fp16 \
  --noTF32 \
  --builderOptimizationLevel=5 \
  --useCudaGraph

/usr/src/tensorrt/bin/trtexec \
  --onnx=small-encoder.onnx \
  --saveEngine=whisper_encoder_fp16.engine \
  --fp16 \
  --minShapes=mel:1x80x50 \
  --optShapes=mel:1x80x500 \
  --maxShapes=mel:1x80x1500 \
  --builderOptimizationLevel=5 \


# Decoder
/usr/src/tensorrt/bin/trtexec \
  --onnx=/home/nvidia/qbo_ws/src/qbo_stt/models/whisper_small/small-decoder.onnx \
  --saveEngine=/home/nvidia/qbo_ws/src/qbo_stt/models/whisper_small/whisper_decoder_fp16.engine \
  --fp16 \
  --noTF32 \
  --builderOptimizationLevel=5 \
  --useCudaGraph \
  --minShapes=tokens:1x1,\
in_n_layer_self_k_cache:12x1x448x768,\
in_n_layer_self_v_cache:12x1x448x768,\
n_layer_cross_k:12x1x448x768,\
n_layer_cross_v:12x1x448x768,\
offset:1 \
  --optShapes=tokens:1x1,\
in_n_layer_self_k_cache:12x1x448x768,\
in_n_layer_self_v_cache:12x1x448x768,\
n_layer_cross_k:12x1x448x768,\
n_layer_cross_v:12x1x448x768,\
offset:1 \
  --maxShapes=tokens:1x1,\
in_n_layer_self_k_cache:12x1x448x768,\
in_n_layer_self_v_cache:12x1x448x768,\
n_layer_cross_k:12x1x448x768,\
n_layer_cross_v:12x1x448x768,\
offset:1
---

# téléchargement de TEN VAD
wget https://huggingface.co/TEN-framework/ten-vad/resolve/main/src/onnx_model/ten-vad.onnx
# compiler TEN VAD
/usr/src/tensorrt/bin/trtexec     --onnx=ten-vad.onnx     --saveEngine=ten-vad.engine     --fp16

## Installation du package ROS2

cd ~/qbo_ws
colcon build --packages-select qbo_stt
source install/setup.bash

---

## Utilisation

### Lancer le node

```bash
ros2 launch qbo_stt stt.launch.py
```

### Activer / désactiver le micro

```bash
# Activer l'écoute
ros2 topic pub --once /is_listening std_msgs/Bool "data: true"

# Désactiver
ros2 topic pub --once /is_listening std_msgs/Bool "data: false"
```

### Écouter les transcriptions

```bash
ros2 topic echo /listened
```

## Topics

### Publiés

| Topic | Type | Description |
|---|---|---|
| `/listened` | `std_msgs/String` | Texte transcrit |

### Souscrits

| Topic | Type | Description |
|---|---|---|
| `/is_listening` | `std_msgs/Bool` | `true` = écoute active, `false` = micro off |

---

## Paramètres (config/params.yaml)

| Paramètre | Défaut | Description |
|---|---|---|
| `encoder_engine` | `/engine_model/encoder.engine` | Engine TRT encoder |
| `decoder_engine` | `/engine_model/decoder_joint.engine` | Engine TRT decoder |
| `vocab_file` | `/engine_model/vocab.txt` | Vocabulaire (8193 tokens) |
| `vad_threshold` | `0.5` | Seuil Silero VAD (0→1) |
| `vad_energy_thr` | `0.01` | Seuil RMS fallback |
| `vad_min_silence_ms` | `500` | Silence pour clore un segment |
| `vad_min_speech_ms` | `200` | Durée min parole valide |
| `vad_max_speech_s` | `10.0` | Durée max segment (s) |
| `audio_device_index` | `-1` | Device micro (-1=défaut) |
| `chunk_size` | `512` | Frames/chunk VAD (32ms) |

---

## Architecture

```
qbo_stt/
├── qbo_stt/
│   ├── __init__.py
│   └── stt_node.py          ← Node principal
│       ├── TRTEngine        — wrapper TRT pré-alloué
│       ├── SileroVAD        — VAD TRT ou énergie RMS
│       ├── extract_mel_nemo — mel NeMo-compatible (librosa)
│       ├── ParakeetASR      — encoder + greedy TDT decoder
│       └── STTNode          — ROS2 node (threads audio + inférence)
├── launch/
│   └── stt.launch.py
├── config/
│   └── params.yaml
├── resource/qbo_stt
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## Dépannage

### Engines non trouvés

```bash
ls -lh ~/qbo_ws/src/qbo_stt/models/engine
# Si absent → relancer la compilation trtexec
```

### Micro non détecté

```bash
# Lister les devices disponibles
python3 -c "
import pyaudio
pa = pyaudio.PyAudio()
for i in range(pa.get_device_count()):
    d = pa.get_device_info_by_index(i)
    if d['maxInputChannels'] > 0:
        print(f'[{i}] {d[\"name\"]}')
"
# Puis modifier audio_device_index dans params.yaml
```

### Transcription vide

- Baisser `vad_energy_thr` à `0.005` (micro peu sensible)
- Baisser `vad_min_speech_ms` à `100`
- Vérifier le niveau micro : `arecord -l`

### Performances GPU maximales

```bash
sudo nvpmodel -m 0    # mode MAXN
sudo jetson_clocks    # fréquences max
tegrastats            # monitoring temps réel
```

---

## Modèle

**NVIDIA Parakeet TDT 0.6B v3** — [HuggingFace](https://huggingface.co/nvidia/parakeet-tdt-0.6b-v3)
- Engine FP16 : ~1.2Go (encoder) + 35Mo (decoder)

Modèle de reconnaissance vocale multilingue 600M paramètres (architecture FastConformer-TDT),
entraîné sur 670 000h d'audio (Granary dataset + NeMo ASR Set 3.0, 128 GPU A100).
Supporte 25 langues européennes avec détection automatique — **WER français : 7.7%**, anglais : 6.1%.
Leader du HuggingFace Open ASR Leaderboard : **WER moyen 6.32%, RTFx 3330** (54× plus rapide que Whisper Large v3).
Robuste au bruit (WER stable 1.92→1.96% de 100dB à 25dB SNR). Licence CC-BY-4.0.

---

*Vincent Foucault — Robot Néo — Avril 2026*
