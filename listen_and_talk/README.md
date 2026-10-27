# listen_and_talk v2 — STT + TTS de Néo

**Vincent FOUCAULT — Octobre 2026 — MIT**

Deux nodes ROS2 **indépendants** (deux process, `respawn`) qui remplacent
l'ancien node monolithique `listen_and_talk_node`, ainsi que `qbo_stt` et
`qbo_tts` (ne pas les lancer en même temps : ils se disputeraient le micro,
la carte son et les topics).

```
 ReSpeaker Lite ─► listen_node ──/listened──────► qbo_discussion ──/to_speak──► talk_node ─► C-Media
                       ▲   └──────/interrupt_tts (barge-in)───────────────────────► │
                       └────────────────────────/is_speaking───────────────────────┘
```

| | Avant (v1 monolithique) | Maintenant (v2) |
|---|---|---|
| Architecture | STT + TTS dans un seul process | 2 process, relance auto (`respawn`) |
| ASR | Parakeet TRT + décodeur TDT maison (risque de boucle infinie, 1 appel TRT/trame) | Parakeet TDT 0.6B v3 via sherpa-onnx (décodeur éprouvé), ponctuation native |
| Débruitage | RNNoise (dégradait la transcription) | supprimé |
| VAD | seuil RMS fixe | Silero ou TEN-VAD |
| Capture micro | lecture bloquante + ASR dans la même boucle → pertes d'échantillons | callback PortAudio → ring lock-free, VAD et ASR dans d'autres threads |
| Écho de la voix du robot | Néo s'auto-interrompait et transcrivait ses propres phrases | rejet de l'écho + barge-in par niveau d'énergie |
| TTS | tout le texte généré avant le 1er son | streaming phrase par phrase (1er son ≈ 3–5× plus tôt) |
| Rééchantillonnage | répétition d'échantillons (images spectrales à −22 dB, son "métallique") | filtre polyphasé Kaiser (−51 dB) |
| Volume | `volume_gain` 2.0 + clamp → écrêtage | carte à 100 % (ALSA) + normalisation −1 dBFS + limiteur doux |
| Carte son | 48 kHz en dur (la C-Media veut 44,1 kHz) | négociation 44100/48000, reconnexion USB automatique |

La voix est **inchangée** : même modèle `axel.onnx` (paquet `qbo_tts`),
mêmes paramètres VITS (noise 0.667 / noise_w 0.8 / length 1.0).

## Installation (Jetson)

```bash
sudo apt install portaudio19-dev libasound2-dev
cd ~/qbo_ws/src/listen_and_talk && ./scripts/get_models.sh   # Parakeet v3 + VAD (~650 Mo)
cd ~/qbo_ws && colcon build --packages-select qbo_msgs qbo_tts listen_and_talk
source install/setup.bash
ros2 launch listen_and_talk listen_and_talk.launch.py
```

> ⚠️ **sherpa-onnx : rester en 1.x** (testé avec 1.13.8). La 2.0 a retiré
> le phonémiseur espeak intégré : la voix axel ne fonctionnerait plus. Le
> CMakeLists bloque la compilation si une 2.x est détectée.

## Topics

| Topic | Type | Node | Rôle |
|---|---|---|---|
| `/to_speak` | String | talk (sub) | texte à dire |
| `/interrupt_tts` | Empty | talk (sub) / listen (pub) | coupe la parole |
| `/stop_speaking` | String | talk (sub) | idem (compatibilité) |
| `/is_speaking` | Bool | talk (pub) / listen (sub) | état parole (changement + 1 Hz) |
| `/listened` | String | listen (pub) | texte reconnu |
| `/listen_result` | qbo_msgs/ListenResult | listen (pub) | texte + confiance |
| `/is_listening` | Bool | listen (sub) | active / coupe l'écoute |

## Réglages utiles

```bash
ros2 param set /talk_node volume_gain 1.3          # à chaud
ros2 param set /talk_node alsa_volume_percent 90
ros2 launch listen_and_talk listen_and_talk.launch.py listen:=false   # TTS seul
```

- **Fausses coupures de parole** (Néo s'interrompt seul) : monter
  `barge_in_ratio` (2.0 → 2.5/3.0) ou `barge_in_ms`.
- **Barge-in trop difficile** : baisser `barge_in_ratio` (1.6). Le haut-parleur
  étant proche du micro, il faut parler nettement plus fort que Néo.
- **Latence ASR** : chaque phrase logue `calcul … ms`. Essayer
  `asr_provider: cuda` et comparer.
- **VAD** : `vad_model: …/ten-vad.onnx` pour TEN-VAD (plus réactive).

## Banc de test hors ROS

```bash
colcon build --packages-select listen_and_talk --cmake-args -DLAT_BENCH=ON
B=install/listen_and_talk/lib/listen_and_talk/lat_bench
M=install/listen_and_talk/share/listen_and_talk/models/listen
$B tts      install/qbo_tts/share/qbo_tts/models "Bonjour, je suis Néo." /tmp/neo.wav
$B asr      $M/sherpa-onnx-nemo-parakeet-tdt-0.6b-v3-int8 $M/silero_vad.onnx enregistrement.wav
$B scenario install/qbo_tts/share/qbo_tts/models $M/sherpa-onnx-nemo-parakeet-tdt-0.6b-v3-int8 $M/silero_vad.onnx 0.15
```

`scenario` rejoue : question → 5 phrases du robot en écho → barge-in →
réponse rapide. Attendu : 3 textes, 1 barge-in (code retour 0).

## Étape suivante recommandée : annulation d'écho (AEC)

Le rejet d'écho actuel est temporel + énergétique : efficace, mais pour
couper la parole il faut parler plus fort que Néo. Une vraie AEC (WebRTC
AEC3) permettrait le barge-in à voix normale ; elle nécessite que la
référence (le son envoyé au haut-parleur) et le micro passent par le même
chemin : soit le haut-parleur branché sur la sortie jack du **ReSpeaker
Lite** (AEC matérielle XMOS intégrée), soit PulseAudio/PipeWire
`module-echo-cancel` entre la C-Media et le ReSpeaker.
