# qbo_stt — Speech-To-Text Node pour Robot Néo

> **Author** : Vincent Foucault — Avril 2026 — Licence MIT  
> **Plateforme** : Jetson Orin NX 16 GB · JetPack 6 · Ubuntu 22.04 · ROS2 Humble  
> **Version** : v11 — RNNoise + FastConformer CTC + Silero VAD

---

## Sommaire

1. [Vue d'ensemble](#1-vue-densemble)
2. [Architecture du pipeline](#2-architecture-du-pipeline)
3. [Modèles utilisés](#3-modèles-utilisés)
4. [Matériel requis](#4-matériel-requis)
5. [Installation des dépendances](#5-installation-des-dépendances)
   - 5.1 [Compilation RNNoise (aarch64)](#51-compilation-rnnoise-aarch64)
   - 5.2 [ONNX Runtime GPU](#52-onnx-runtime-gpu)
   - 5.3 [Dépendances Python](#53-dépendances-python)
6. [Téléchargement des modèles](#6-téléchargement-des-modèles)
7. [Structure du package ROS2](#7-structure-du-package-ros2)
8. [Configuration](#8-configuration)
   - 8.1 [Paramètres ROS2](#81-paramètres-ros2)
   - 8.2 [Topics](#82-topics)
9. [Lancement](#9-lancement)
10. [Réglages fins pour usage familial](#10-réglages-fins-pour-usage-familial)
11. [State machine VAD](#11-state-machine-vad)
12. [Détail technique du pipeline](#12-détail-technique-du-pipeline)
    - 12.1 [RNNoise](#121-rnnoise)
    - 12.2 [Silero VAD](#122-silero-vad)
    - 12.3 [FastConformer CTC](#123-fastconformer-ctc)
    - 12.4 [Décodage greedy CTC](#124-décodage-greedy-ctc)
    - 12.5 [Filtrage post-transcription](#125-filtrage-post-transcription)
13. [Performances mesurées](#13-performances-mesurées)
14. [Dépannage](#14-dépannage)
15. [Licence](#15-licence)

---

## 1. Vue d'ensemble

`qbo_stt` est le nœud de reconnaissance vocale du robot Néo (plateforme QBo).
Il convertit la parole française captée par le micro ReSpeaker Lite en texte
et le publie sur le topic ROS2 `/listened`.

Le pipeline est optimisé pour une **conversation naturelle en environnement familial**
(TV en fond, bruit de cuisine, plusieurs personnes dans la pièce) avec une latence
bout-en-bout de **150 à 350ms** après la fin de parole.

**Ce que fait ce nœud :**

- Capte l'audio en continu depuis le micro (16kHz, mono, int16)
- Débruite le signal en temps réel avec RNNoise (suppression TV, ventilateur)
- Détecte la parole frame par frame avec Silero VAD
- Segmente automatiquement les phrases via une state machine à 4 états
- Transcrit chaque segment en français avec FastConformer CTC (CUDA)
- Filtre les hallucinations et publie le résultat sur `/listened`

**Ce que ne fait PAS ce nœud :**

- Wake word (mot d'activation) — le contrôle se fait via `/is_listening`
- Identification du locuteur
- Traitement du langage naturel (NLU) — délégué aux nœuds aval

---

## 2. Architecture du pipeline

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    THREAD AUDIO (CPU, ~32ms/cycle)                      │
│                                                                         │
│  ReSpeaker Lite                                                         │
│  (card 0, 16kHz, mono, int16)                                           │
│       │                                                                 │
│       ▼  PyAudio read() — 512 samples (32ms)                            │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  RNNoise (ctypes, librnnoise.so, CPU)                           │    │
│  │  → débruitage neural temps réel                                 │    │
│  │  → 160 samples/frame à 16kHz (upsample×3 → process → down×3)    │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│       │                                                                 │
│       ▼  chunk float32 débruité                                         │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  Silero VAD (ONNX Runtime CPU, LSTM ~1MB)                       │    │
│  │  → probabilité parole [0,1] par chunk de 512 samples            │    │
│  │  → correction RMS (RMS_FLOOR=0.006, RMS_SPEECH=0.015)           │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│       │                                                                 │
│       ▼  décision booléenne is_speech                                   │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  State Machine VAD                                              │    │
│  │  IDLE → PRE_SPEECH → SPEECH → MAYBE_END                         │    │
│  │  → gère pré-buffer 600ms, confirmation 200ms, fin 600ms         │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│       │                                                                 │
└───────┼─────────────────────────────────────────────────────────────────┘
        │  numpy array float32 (segment complet, 0.15s–10s)
        ▼  Queue Python (maxsize=5, non bloquante)
┌─────────────────────────────────────────────────────────────────────────┐
│                 THREAD INFÉRENCE (GPU CUDA)                             │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  Extraction mel spectrogram (librosa)                           │    │
│  │  → 80 bandes mel, n_fft=512, hop=160, win=320                   │    │
│  │  → normalisation per_feature NeMo (mean=0, std=1 par bande)     │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│       │  (1, 80, T) float32                                             │
│       ▼                                                                 │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  FastConformer CTC (ONNX Runtime CUDA)                          │    │
│  │  → encoder convolutif + attention locale                        │    │
│  │  → subsampling×8 → logprobs (1, T/8, 2561)                      │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│       │  logprobs float32                                               │
│       ▼                                                                 │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  Décodage greedy CTC                                            │    │
│  │  → argmax → suppression blanks et répétitions → BPE → texte     │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│       │  texte brut                                                     │
│       ▼                                                                 │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  Filtrage post-transcription                                    │    │
│  │  → suppression garbage, charset FR, validation mots              │   │
│  └─────────────────────────────────────────────────────────────────┘    │
│       │  texte validé                                                   │
└───────┼─────────────────────────────────────────────────────────────────┘
        │
        ▼  std_msgs/String
   Topic /listened
```

---

## 3. Modèles utilisés

| Modèle | Fichier | Taille | Rôle |
|--------|---------|--------|------|
| FastConformer CTC | `model.onnx` | ~460 MB | Transcription ASR FR |
| Vocabulaire BPE | `tokens.txt` | ~50 KB | Décodage CTC |
| Silero VAD | `silero_vad.onnx` | ~1 MB | Détection activité vocale |
| RNNoise | `librnnoise.so.0` | ~100 KB | Débruitage temps réel |

**FastConformer** : `stt_multilingual_fastconformer_hybrid_large_pc`
- ~114M paramètres
- Entraîné sur ~20 000h de parole multilingue (français natif inclus)
- Ponctuation et casse incluses dans les prédictions
- Exporté depuis NVIDIA NeMo via sherpa-onnx (branche CTC)

---

## 4. Matériel requis

| Composant | Spécification |
|-----------|---------------|
| Carte principale | Jetson Orin NX 16 GB |
| OS / SDK | JetPack 6 (Ubuntu 22.04, CUDA 12.x) |
| Micro | ReSpeaker Lite (ALSA card 0, 16kHz mono) |
| ROS2 | Humble |
| RAM GPU | ~800 MB (FastConformer FP32 ONNX) |

---

## 5. Installation des dépendances

### 5.1 Compilation RNNoise (aarch64)

Les packages pip de `rnnoise-python` sont compilés uniquement pour x86_64.
Sur Orin NX (aarch64), il faut compiler la lib C depuis les sources :

```bash
# Outils de compilation
sudo apt install -y autoconf automake libtool gcc build-essential python3-dev

# Cloner le dépôt officiel
git clone https://github.com/xiph/rnnoise.git
cd rnnoise

# Générer les scripts autoconf
./autogen.sh

# Configuration (installation dans /usr/local/lib/)
./configure --prefix=/usr/local

# Compilation (utilise tous les cœurs disponibles)
make -j$(nproc)

# Installation système
sudo make install

# Mise à jour du cache des libs partagées
sudo ldconfig

# Vérification
ls -la /usr/local/lib/librnnoise*
# Attendu :
#   /usr/local/lib/librnnoise.a
#   /usr/local/lib/librnnoise.so -> librnnoise.so.0
#   /usr/local/lib/librnnoise.so.0

# Test rapide
python3 -c "import ctypes; lib = ctypes.CDLL('/usr/local/lib/librnnoise.so.0'); print('OK')"
```

> **Note :** Si la compilation échoue avec `aclocal: command not found`,
> installer `automake` : `sudo apt install -y automake`

### 5.2 ONNX Runtime GPU

ONNX Runtime avec support CUDA sur aarch64 — utiliser les wheels NVIDIA :

```bash
# Wheel officiel NVIDIA pour JetPack 6 / aarch64
# Étape 1 — Supprimer les deux versions conflictuelles
pip uninstall -y onnxruntime onnxruntime-gpu

# Étape 2 — Installer le wheel NVIDIA JetPack 6 / CUDA 12.6 / aarch64
pip install onnxruntime-gpu==1.23.0 \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
  --no-deps

# Étape 3 — Vérification
python3 -c "
import onnxruntime as ort
print('Version :', ort.__version__)
print('Providers :', ort.get_available_providers())
# Attendu : CUDAExecutionProvider dans la liste
"
```

### 5.3 Dépendances Python

```bash
pip install librosa pyaudio numpy

# Vérification PyAudio et ReSpeaker
python3 -c "
import pyaudio
pa = pyaudio.PyAudio()
for i in range(pa.get_device_count()):
    info = pa.get_device_info_by_index(i)
    if info['maxInputChannels'] > 0:
        print(f'Device {i}: {info[\"name\"]} ({info[\"defaultSampleRate\"]}Hz)')
pa.terminate()
"
```

---

## 6. Téléchargement des modèles

### FastConformer CTC (model.onnx + tokens.txt)

```bash
# Via sherpa-onnx — modèle multilingue avec ponctuation
# Choisir le modèle 'large' pour la meilleure précision FR

# Option A : depuis Hugging Face
pip install huggingface-hub
python3 -c "
from huggingface_hub import hf_hub_download
import shutil, os

# Téléchargement model.onnx
hf_hub_download(
    repo_id='csukuangfj/sherpa-onnx-nemo-ctc-en-conformer-large',
    filename='model.onnx',
    local_dir='/tmp/fc_model'
)
"
# Note : chercher le modèle multilingue avec 'fastconformer' et 'multilingual'
# sur https://huggingface.co/nvidia/

# Option B : depuis NVIDIA NGC (recommandé)
# Connexion NGC requise : https://ngc.nvidia.com
# Chercher : stt_multilingual_fastconformer_hybrid_large_pc
# Exporter via NeMo ou utiliser le script sherpa-onnx d'export

# Placement des fichiers
mkdir -p ~/qbo_ws/src/qbo_stt/models
cp /tmp/fc_model/model.onnx ~/qbo_ws/src/qbo_stt/models/
cp /tmp/fc_model/tokens.txt ~/qbo_ws/src/qbo_stt/models/
```

### Silero VAD

```bash
# Téléchargement direct depuis GitHub
cd ~/qbo_ws/src/qbo_stt/models

# Version ONNX recommandée (compatible avec le code)
wget https://github.com/snakers4/silero-vad/raw/master/src/silero_vad/data/silero_vad.onnx

# Vérification
python3 -c "
import onnxruntime as ort
sess = ort.InferenceSession('silero_vad.onnx', providers=['CPUExecutionProvider'])
print('VAD inputs:', [i.name for i in sess.get_inputs()])
# Attendu : ['x', 'h', 'c']
"
```

---

## 7. Structure du package ROS2

```
qbo_stt/
├── qbo_stt/
│   └── stt_node.py          ← nœud principal (ce fichier)
├── models/
│   ├── model.onnx            ← FastConformer CTC (~460 MB)
│   ├── tokens.txt            ← vocabulaire BPE (~50 KB)
│   └── silero_vad.onnx       ← Silero VAD (~1 MB)
├── launch/
│   └── stt.launch.py         ← fichier de lancement
├── package.xml
├── setup.py
└── README.md
```

**setup.py minimal :**

```python
from setuptools import setup

package_name = 'qbo_stt'
setup(
    name=package_name,
    version='11.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'stt_node = qbo_stt.stt_node:main',
        ],
    },
)
```

**launch/stt.launch.py :**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qbo_stt',
            executable='stt_node',
            name='stt_node',
            output='screen',
            parameters=[{
                'vad_threshold':        0.5,
                'vad_min_silence_ms':   400,
                'vad_min_speech_ms':    32,
                'vad_max_speech_s':     10.0,
                'vad_pre_buffer_ms':    500,
                'audio_device_index':   0,
                'chunk_size':           512,
                'rtf_warn_threshold':   0.20,
                'min_wav_duration_s':   0.15,
                'min_words_to_publish': 1,
                'rnnoise_enabled':      True,
            }]
        )
    ])
```

---

## 8. Configuration

### 8.1 Paramètres ROS2

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `fc_model` | `models/model.onnx` | Chemin FastConformer ONNX |
| `fc_tokens` | `models/tokens.txt` | Chemin vocabulaire BPE |
| `vad_model` | `models/silero_vad.onnx` | Chemin Silero VAD |
| `vad_threshold` | `0.5` | Seuil décision parole [0-1] |
| `vad_min_silence_ms` | `400` | Silence min avant fin segment (ms) |
| `vad_min_speech_ms` | `32` | Parole min pour confirmation (ms) |
| `vad_max_speech_s` | `10.0` | Durée max segment (secondes) |
| `vad_pre_buffer_ms` | `500` | Contexte audio avant détection (ms) |
| `audio_device_index` | `0` | Index device PyAudio (-1 = défaut système) |
| `chunk_size` | `512` | Taille chunk PyAudio (**ne pas modifier**) |
| `rtf_warn_threshold` | `0.20` | Seuil alerte RTF |
| `min_wav_duration_s` | `0.15` | Durée min segment valide (secondes) |
| `min_words_to_publish` | `1` | Mots min pour publication |
| `rnnoise_enabled` | `True` | Activer/désactiver RNNoise |

> ⚠️ **`chunk_size` doit rester à 512** — contrainte imposée par Silero VAD ONNX.
> Modifier cette valeur provoque des erreurs de dimension dans le modèle ONNX.

### 8.2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/listened` | `std_msgs/String` | Publié | Texte transcrit en français |
| `/is_listening` | `std_msgs/Bool` | Souscrit | `True` = activer écoute |

**Contrôle depuis un autre nœud :**

```python
# Activer l'écoute
ros2 topic pub /is_listening std_msgs/Bool "data: true" --once

# Désactiver (pendant que Néo parle)
ros2 topic pub /is_listening std_msgs/Bool "data: false" --once

# Écouter les transcriptions
ros2 topic echo /listened
```

---

## 9. Lancement

```bash
# Build du workspace
cd ~/qbo_ws
colcon build --packages-select qbo_stt
source install/setup.bash

# Lancement du nœud
ros2 launch qbo_stt stt.launch.py

# Ou directement
ros2 run qbo_stt stt_node

# Vérification au démarrage — logs attendus :
# ⚡ Chargement FastConformer CTC CUDA...
#   ✅ FastConformer : model.onnx
#   ✅ Vocab         : 2561 tokens
#   ✅ Blank token   : index 0
#   ✅ VAD           : silero_vad.onnx
#   ✅ RNNoise       : débruitage actif
# 🔥 Warmup FastConformer (allocation GPU)...
# ✅ STT FastConformer CTC FR prêt — en écoute
# 🎙️  Micro ouvert (device=0) | RNNoise ✅ actif
```

**Monitoring temps réel :**

```bash
# Voir les transcriptions
ros2 topic echo /listened

# Voir tous les logs du nœud (inclus RTF et filtrage)
ros2 run rqt_console rqt_console

# Statistiques topics
ros2 topic hz /listened
```

---

## 10. Réglages fins pour usage familial

### Environnement calme (bureau, chambre)

```python
# launch file parameters
'vad_threshold':      0.4,   # plus sensible
'vad_min_silence_ms': 500,   # peut se permettre d'attendre plus
'rnnoise_enabled':    False, # optionnel si pas de bruit de fond
```

### Environnement bruyant (salon avec TV, cuisine)

```python
'vad_threshold':        0.55,  # moins de faux positifs
'vad_min_silence_ms':   350,   # réactivité conservée
'rnnoise_enabled':      True,  # indispensable
'min_words_to_publish': 1,     # accepter les mots courts ("oui", "non")
```

### Conversation rapide (mots courts, réponses courtes)

```python
'min_wav_duration_s':   0.10,  # accepter les très courts segments
'vad_min_speech_ms':    32,    # confirmation quasi-immédiate
'min_words_to_publish': 1,     # publier même les mots isolés
```

### Robustesse maximale (enfants, accents, locuteurs multiples)

```python
'vad_threshold':        0.45,
'vad_pre_buffer_ms':    600,   # plus de contexte avant la parole
'min_wav_duration_s':   0.20,
'min_words_to_publish': 1,
```

---

## 11. State machine VAD

La détection des segments de parole est gérée par une machine à états finis
à 4 états. Cette approche évite les heuristiques ad-hoc et produit un
comportement déterministe et facilement débogable.

```
                  VAD=True
    ┌─────────────────────────────────────┐
    │                                     │
    ▼                                     │
  ┌──────┐   VAD=True   ┌────────────┐    │
  │      │ ─────────── ▶│            │    │
  │ IDLE │              │ PRE_SPEECH │    │
  │      │ ◀─────────── │            │    │
  └──────┘  VAD=False   └────────────┘    │
    ▲                        │            │
    │                        │ parole > MIN_SPEECH_MS (200ms)
    │                        ▼            │
    │                    ┌────────┐       │
    │     flush_final()  │        │ VAD=True
    └─────────────────── │ SPEECH │ ──────┘
                         │        │
                         └────────┘
                              │
                              │ VAD=False
                              ▼
                        ┌──────────┐
                        │          │ ─── VAD=True ──▶ SPEECH
                        │ MAYBE_END│
                        │          │ ─── silence > SLOW_END_MS ──▶ flush_final() ──▶ IDLE
                        └──────────┘
                              │
                              └── energy_ratio < 0.25 ET silence > FAST_END_MS
                                  ──▶ flush_fast() ──▶ SPEECH (phrase longue)
```

**Transitions critiques :**

| De            | Vers          | Condition                         | Action                                |
|---------------|---------------|-----------------------------------|---------------------------------------|
| IDLE          | PRE_SPEECH    | VAD=True                          | Init pending_speech                   |
| PRE_SPEECH    | IDLE          | VAD=False                         | Récupère chunks dans pré-buffer       |
| PRE_SPEECH    | SPEECH        | speech_ms > 200ms                 | Init speech_buffer avec pré-buffer    |
| SPEECH        | MAYBE_END     | VAD=False                         | Début comptage silence                |
| MAYBE_END     | SPEECH        | VAD=True                          | Reset silence_ms                      |
| MAYBE_END     | SPEECH        | energy < 25% ET silence > 350ms   | flush_fast() — phrase longue          |
| MAYBE_END     | IDLE          | silence > 600ms                   | flush_final() — fin de phrase         |
| MAYBE_END     | IDLE          | speech_ms > MAX                   | flush_final() — sécurité              |

---

## 12. Détail technique du pipeline

### 12.1 RNNoise

RNNoise est un réseau GRU (Gated Recurrent Unit) compact entraîné pour
séparer la parole du bruit de fond en domaine fréquentiel.

**Contrainte d'API :**
```
rnnoise_process_frame(state, float* out, const float* in)
                                          ↑ exactement 480 samples à 48kHz
```

**Adaptation à 16kHz :**
```
chunk 16kHz (160 samples)
  → upsample ×3 par répétition → 480 samples à 48kHz (approx.)
  → rnnoise_process_frame() → 480 samples débruités
  → downsample [::3] → 160 samples à 16kHz
```

**Gestion des résidus :**
Chaque chunk PyAudio fait 512 samples. `512 % 160 = 32` samples résiduels
sont stockés dans `self._residual` et préfixés au chunk suivant pour
garantir des frames complètes de 160 samples à chaque cycle.

### 12.2 Silero VAD

LSTM à 2 couches (h, c de shape `(2, 1, 64)`) qui maintient un contexte
temporel entre les chunks. La décision est binaire : `prob >= threshold`.

Deux corrections RMS complètent la décision VAD :
- `rms < RMS_FLOOR (0.006)` → forcer False (plancher numérique)
- `rms > RMS_SPEECH (0.015)` ET état actif → forcer True (phonème fort raté)

### 12.3 FastConformer CTC

Encoder acoustique hybride :
- **Convolutions locales** : capturent les patterns phonétiques courts (10-30ms)
- **Attention multi-têtes locale** : capturent les dépendances à plus long terme
- **Subsampling ×8** : réduction temporelle `T → T/8` (100fps → 12.5fps)
- **Tête CTC** : produit des log-probabilités sur 2561 tokens BPE par frame

**Normalisation per_feature (indispensable) :**
```
Pour chaque bande mel k de 0 à 79 :
  mean_k = mean(mel[k, :])
  std_k  = std(mel[k, :])
  mel_norm[k, :] = (mel[k, :] - mean_k) / max(std_k, 1e-5)
```

Sans cette normalisation, le modèle hallucine dans d'autres langues.

### 12.4 Décodage greedy CTC

```
logprobs (T/8, 2561)
    │
    ▼ argmax(axis=-1)
token_ids (T/8,)  ex: [145, 145, 0, 872, 872, 0, 0, 233]
    │                        └── blank=0
    ▼ suppression blanks + déduplications consécutives
decoded  (variable)  ex: [145, 872, 233]
    │
    ▼ vocab lookup + '▁' → ' '
texte    ex: "bonjour néo"
```

### 12.5 Filtrage post-transcription

5 filtres en cascade (du plus rapide au plus coûteux) :

1. **Longueur < 2** : rejet immédiat
2. **Token isolé ≤ 2 chars** : rejette `Wa`, `N`, `i`
3. **Aucune lettre** : rejette les pures ponctuations
4. **Charset FR** : rejette cyrillique, chinois, etc.
5. **Nombre de mots** : configurable via `min_words_to_publish`

---

## 13. Performances mesurées

Mesurées sur Jetson Orin NX 16GB · JetPack 6 · CUDA 12.x

| Métrique                  | Valeur        |
|-------------------------- |---------------|
| RTF moyen (segment 1-2s)  | 0.04 – 0.08   |
| RTF moyen (segment 2-5s)  | 0.06 – 0.10   |
| Latence bout-en-bout      | 150 – 350ms   |
| RAM GPU (FastConformer)   | ~780 MB       |
| CPU thread audio          | ~3% (Orin NX) |
| CPU RNNoise               | ~0.5%         |
| CPU Silero VAD            | ~1%           |
| Latence RNNoise / chunk   | ~0.3ms        |

**Signification RTF :**
- `RTF = 0.05` → 5% du temps réel → 1 seconde d'audio transcrite en 50ms
- `RTF < 0.20` → objectif pour une conversation naturelle sans accumulation

---

## 14. Dépannage

### RNNoise non chargé

```
[RNNoise] ⚠️  librnnoise.so introuvable → mode passthrough
```

**Cause :** lib non compilée ou non dans le PATH ldconfig.
**Solution :**
```bash
sudo ldconfig
ls /usr/local/lib/librnnoise*  # vérifier présence
sudo ldconfig -p | grep rnnoise  # vérifier cache
```

### RTF > 0.20 (inférence trop lente)

**Cause possible :** ONNX Runtime sur CPU au lieu de CUDA.
**Vérification :**
```python
import onnxruntime as ort
sess = ort.InferenceSession('model.onnx', providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
print(sess.get_providers())  # doit inclure CUDAExecutionProvider
```

**Solution :** Réinstaller onnxruntime-gpu depuis les wheels NVIDIA.

### Hallucinations en cyrillique ou anglais

**Cause :** normalisation per_feature absente ou paramètres mel incorrects.
**Vérification :** Inspecter `_extract_mel()` — les valeurs N_MELS=80, N_FFT=512,
HOP_LENGTH=160, WIN_LENGTH=320 doivent correspondre exactement au modèle.

### Trop de faux positifs VAD (robot s'active sur la TV)

```bash
# Augmenter le seuil VAD
ros2 param set /stt_node vad_threshold 0.65

# Ou augmenter le plancher RMS (dans le code)
RMS_FLOOR = 0.010  # au lieu de 0.006
```

### Mots courts non détectés ("oui", "non", "stop")

```bash
# Réduire le seuil de confirmation de parole
ros2 param set /stt_node vad_min_speech_ms 32

# Réduire la durée minimale du segment
ros2 param set /stt_node min_wav_duration_s 0.10
```

### Micro non trouvé (PyAudio error)

```bash
# Lister les devices audio disponibles
python3 -c "
import pyaudio
pa = pyaudio.PyAudio()
for i in range(pa.get_device_count()):
    d = pa.get_device_info_by_index(i)
    if d['maxInputChannels'] > 0:
        print(f'{i}: {d[\"name\"]} — {d[\"defaultSampleRate\"]}Hz')
pa.terminate()
"
# Adapter audio_device_index dans le launch file
```

### Timeout chargement modèles (> 60s)

**Cause :** modèle très gros ou GPU occupé par un autre processus.
**Solution :** Vérifier l'utilisation GPU avec `jtop` ou `nvidia-smi`,
augmenter le timeout dans `_models_ready.wait(timeout=120.0)`.

---

## 15. Licence

```
MIT License

Copyright (c) 2026 Vincent Foucault

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

*qbo_stt v11 — Vincent Foucault — Avril 2026*
