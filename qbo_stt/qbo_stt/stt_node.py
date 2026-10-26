#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║   qbo_stt — Speech-To-Text Node — Robot Néo                      ║
║   FastConformer CTC multilingue — ONNX Runtime CUDA              ║
║   VAD : Silero ONNX (CPU) — hystérésis 0.50/0.50 + N consécutifs ║
║   Sans débruitage (FastConformer préfère le signal brut)         ║
║   Author : Vincent Foucault — Avril 2026 — MIT                   ║
╚══════════════════════════════════════════════════════════════════╝

Version v19 — Suppression définitive de RNNoise + seuils renforcés

Analyse comparative A/B (v17) :
  Corpus de test identique, RNNoise ON vs OFF.

  ❌ Avec RNNoise :
     "Donc que tu vas bien"      → "Don que tu vas bien"   (-c)
     "Est-ce que tu vas bien"    → "Êt - ce que tu vas bien"
     "Comment vas-tu"            → "Comment vras - tu"
     "Répète-moi ton prénom"     → "Aaprès moi, ton canon"
     "Après moi ton prénom"      → "Après de moi toneon"

  ✅ Sans RNNoise :
     "Est-ce que tu en es certain ?"                  → parfait
     "De toute évidence, c'est le filtre noise ..."   → parfait

Causes :
  1. FastConformer (NeMo) est entraîné sur LibriSpeech, MCV, VoxPopuli
     avec bruit naturel modéré.
  2. RNNoise (RNN pour VoIP) applique un filtre agressif qui coupe :
       - Fricatives sourdes /s/ /f/ /ʃ/  → "Répète"  → "Après"
       - Consonnes nasales finales /n/ /m/ → "prénom" → "canon"
       - Attaques d'occlusives /p/ /t/ /k/ → "Donc"   → "Don"
  3. Les artefacts spectraux (gains variables par frame) créent
     une distorsion que FastConformer n'a jamais vue à l'entraînement.

Observation secondaire (logs v17) :
  Reprises confirmées suspectes avec prob = 0.486, 0.488, 0.460
  (3 chunks tout juste au-dessus de 0.45 sur bruit ambiant soutenu,
  pas une vraie reprise). Critère à durcir.

Fixes v19 :
  ✅ RNNoise supprimé du code (classe, imports, paramètres, appels).
     Le signal brut ReSpeaker Lite → FastConformer directement.
  ✅ vad_threshold_end : 0.45 → 0.50.
     Les vraies reprises montent à 0.7-0.9, aucune perte ; les
     queues LSTM à 0.40-0.48 sont maintenant filtrées.
  ✅ RMS_RESUME_MIN : un chunk ne compte comme candidat reprise
     que si son RMS > 0.008. Filtre les pseudo-reprises sur bruit
     ambiant de faible énergie (respiration, réverb).

Changements historiques :
  v14→v15 : suppression flush_fast, pre-buffer 800ms.
  v15→v16 : hystérésis Silero, suppression RMS_SPEECH, debug VAD.
  v16→v17 : threshold_end 0.45, N chunks consécutifs pour reprise.
  v17→v19 : RNNoise SUPPRIMÉ, threshold_end 0.50, RMS_RESUME_MIN.
"""

import re
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool

import numpy as np
import threading
import queue
import time
import os
from collections import deque

import onnxruntime as ort
import pyaudio
import librosa

# ── Chemins des modèles ───────────────────────────────────────────────────────
MODEL_BASE = '/home/nvidia/qbo_ws/src/qbo_stt/models'
FC_MODEL   = os.path.join(MODEL_BASE, 'model.onnx')
FC_TOKENS  = os.path.join(MODEL_BASE, 'tokens.txt')
VAD_MODEL  = os.path.join(MODEL_BASE, 'silero_vad.onnx')

# ── Constantes audio ──────────────────────────────────────────────────────────
SR = 16000

# ── Paramètres mel NeMo ───────────────────────────────────────────────────────
N_MELS     = 80
N_FFT      = 512
HOP_LENGTH = 160
WIN_LENGTH = 320

# ── Filtres post-transcription ────────────────────────────────────────────────
_VALID_CHARSET = re.compile(
    r"^[a-zA-Z0-9àâäéèêëîïôùûüœæçÀÂÄÉÈÊËÎÏÔÙÛÜŒÆÇ\s'\-\,\.\!\?\:\;\(\)%°]+$"
)
_GARBAGE_RE = re.compile(
    r'<\w+>'
    r'|^\s*$'
    r'|\bWait\b'
    r'|\bWa\b'
)


# ══════════════════════════════════════════════════════════════════════════════
#  Silero VAD — ONNX Runtime CPU, hystérésis paramétrable
# ══════════════════════════════════════════════════════════════════════════════
class SileroVAD:

    def __init__(
        self,
        model_path: str,
        threshold_start: float = 0.50,
        threshold_end:   float = 0.50,
    ):
        """
        Hystérésis :
          - threshold_start : seuil pour DÉMARRER la détection (IDLE → PRE_SPEECH).
          - threshold_end   : seuil pour CONFIRMER la parole en cours
            (SPEECH, MAYBE_END). Doit être ≤ threshold_start.
            À 0.50, les probs en silence (< 0.30 observées) passent sous
            le radar, les vraies reprises (> 0.7) sont détectées sans
            ambiguïté.
        """
        self.th_start = threshold_start
        self.th_end   = threshold_end
        self.session  = ort.InferenceSession(
            model_path, providers=['CPUExecutionProvider']
        )
        self._h = np.zeros((2, 1, 64), dtype=np.float32)
        self._c = np.zeros((2, 1, 64), dtype=np.float32)

    def reset(self):
        self._h = np.zeros((2, 1, 64), dtype=np.float32)
        self._c = np.zeros((2, 1, 64), dtype=np.float32)

    def probability(self, chunk: np.ndarray) -> float:
        out = self.session.run(None, {
            'x': chunk[np.newaxis].astype(np.float32),
            'h': self._h,
            'c': self._c,
        })
        prob, self._h, self._c = out[0], out[1], out[2]
        return float(prob[0][0])

    def is_speech(self, chunk: np.ndarray, currently_active: bool) -> tuple:
        """
        Retourne (is_speech_bool, raw_probability).
        Applique l'hystérésis : seuil haut pour démarrer, seuil bas pour
        continuer.
        """
        prob = self.probability(chunk)
        threshold = self.th_end if currently_active else self.th_start
        return (prob >= threshold), prob


# ══════════════════════════════════════════════════════════════════════════════
#  FastConformer ASR — ONNX Runtime CUDA (sans TensorRT EP implicite)
# ══════════════════════════════════════════════════════════════════════════════
class FastConformerASR:

    def __init__(self, model_path: str, tokens_path: str):
        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        available_providers = ort.get_available_providers()
        print(f'[FastConformer] Providers ORT disponibles : {available_providers}')

        cuda_provider_options = {
            'device_id':                 0,
            'arena_extend_strategy':     'kNextPowerOfTwo',
            'gpu_mem_limit':             4 * 1024 * 1024 * 1024,
            'cudnn_conv_algo_search':    'HEURISTIC',
            'do_copy_in_default_stream': True,
        }

        self.session = ort.InferenceSession(
            model_path,
            sess_options=opts,
            # TensorrtExecutionProvider intentionnellement ABSENT
            providers=[
                ('CUDAExecutionProvider', cuda_provider_options),
                'CPUExecutionProvider',
            ],
        )

        for i, inp in enumerate(self.session.get_inputs()):
            print(f'[FastConformer] Input[{i}] : {inp.name} {inp.shape} {inp.type}')

        self.vocab = self._load_vocab(tokens_path)
        self.blank = next(
            (idx for idx, tok in self.vocab.items() if tok == '<blk>'), 0
        )
        self._output_name = self.session.get_outputs()[0].name

    @staticmethod
    def _load_vocab(path: str) -> dict:
        vocab = {}
        with open(path, encoding='utf-8') as f:
            for line in f:
                parts = line.rstrip('\n').split(' ', 1)
                if len(parts) == 2:
                    vocab[int(parts[1])] = parts[0]
        return vocab

    def _extract_mel(self, wav: np.ndarray) -> np.ndarray:
        mel  = librosa.feature.melspectrogram(
            y=wav, sr=SR,
            n_fft=N_FFT, hop_length=HOP_LENGTH, win_length=WIN_LENGTH,
            window='hann', n_mels=N_MELS,
            fmin=0, fmax=8000, power=2.0, center=True,
        )
        mel  = np.log(mel + 1e-20)
        mean = mel.mean(axis=1, keepdims=True)
        std  = np.maximum(mel.std(axis=1, keepdims=True), 1e-5)
        mel  = (mel - mean) / std
        return mel[np.newaxis].astype(np.float32)

    def transcribe(self, wav: np.ndarray) -> tuple:
        t0           = time.perf_counter()
        mel_input    = self._extract_mel(wav)
        length_input = np.array([mel_input.shape[2]], dtype=np.int64)
        logprobs = self.session.run(
            [self._output_name],
            {'audio_signal': mel_input, 'length': length_input}
        )[0]
        text = self._greedy_ctc_decode(logprobs[0])
        dt   = time.perf_counter() - t0
        return text, (dt / (len(wav) / SR) if len(wav) > 0 else 0.0)

    def _greedy_ctc_decode(self, logprobs: np.ndarray) -> str:
        token_ids = np.argmax(logprobs, axis=-1)
        decoded, prev = [], None
        for tok in token_ids:
            tok = int(tok)
            if tok != self.blank and tok != prev:
                decoded.append(tok)
            prev = tok
        return ''.join(self.vocab.get(t, '') for t in decoded).replace('▁', ' ').strip()


# ══════════════════════════════════════════════════════════════════════════════
#  Nettoyage post-transcription
# ══════════════════════════════════════════════════════════════════════════════
def _clean_transcription(text: str) -> str:
    t = _GARBAGE_RE.sub('', text)
    t = re.sub(r'^[\u0400-\u04FF\u0500-\u052F\s\.\,\?\!\:\;\-]+', '', t)
    t = re.sub(r'\.{2,}', ' ', t)
    t = re.sub(r'[\s\.]+$', '', t)
    return t.strip()


def _is_valid_transcription(text: str, min_words: int = 1) -> bool:
    t = text.strip()
    if len(t) < 2:
        return False
    if len(t) <= 2 and len(t.split()) == 1:
        return False
    if not re.search(r'[a-zA-ZÀ-ÿœæŒÆ]', t):
        return False
    if not _VALID_CHARSET.match(t):
        return False
    if len(t.split()) < min_words:
        return False
    return True


# ══════════════════════════════════════════════════════════════════════════════
#  ROS2 STT Node
# ══════════════════════════════════════════════════════════════════════════════
class STTNode(Node):

    def __init__(self):
        super().__init__('stt_node')
        self.cb_group = ReentrantCallbackGroup()

        # ── Paramètres ROS2 ───────────────────────────────────────────────────
        self.declare_parameter('fc_model',                FC_MODEL)
        self.declare_parameter('fc_tokens',               FC_TOKENS)
        self.declare_parameter('vad_model',               VAD_MODEL)
        self.declare_parameter('vad_threshold_start',     0.50)
        self.declare_parameter('vad_threshold_end',       0.50)   # ↑ de 0.45
        self.declare_parameter('vad_resume_consecutive',  3)      # chunks (~96ms)
        self.declare_parameter('vad_min_silence_ms',      600)
        self.declare_parameter('vad_min_speech_ms',       200)
        self.declare_parameter('vad_max_speech_s',        10.0)
        self.declare_parameter('vad_pre_buffer_ms',       800)
        self.declare_parameter('audio_device_index',      0)
        self.declare_parameter('chunk_size',              512)
        self.declare_parameter('rtf_warn_threshold',      0.20)
        self.declare_parameter('min_wav_duration_s',      0.30)
        self.declare_parameter('min_words_to_publish',    1)
        self.declare_parameter('vad_debug',               False)

        fc_model  = os.path.expanduser(self.get_parameter('fc_model').value)
        fc_tokens = os.path.expanduser(self.get_parameter('fc_tokens').value)
        vad_path  = os.path.expanduser(self.get_parameter('vad_model').value)
        vad_th_s  = self.get_parameter('vad_threshold_start').value
        vad_th_e  = self.get_parameter('vad_threshold_end').value

        self.resume_consecutive = self.get_parameter('vad_resume_consecutive').value
        self.min_silence        = self.get_parameter('vad_min_silence_ms').value  / 1000.0
        self.min_speech         = self.get_parameter('vad_min_speech_ms').value   / 1000.0
        self.max_speech         = self.get_parameter('vad_max_speech_s').value
        self.pre_buffer_ms      = self.get_parameter('vad_pre_buffer_ms').value   / 1000.0
        self.device_index       = self.get_parameter('audio_device_index').value
        self.chunk_size         = self.get_parameter('chunk_size').value
        self.rtf_warn_thr       = self.get_parameter('rtf_warn_threshold').value
        self.min_wav_dur        = self.get_parameter('min_wav_duration_s').value
        self.min_words          = self.get_parameter('min_words_to_publish').value
        self.vad_debug          = self.get_parameter('vad_debug').value

        # ── Publishers / Subscriptions ────────────────────────────────────────
        self.pub = self.create_publisher(String, '/listened', 10)
        self.create_subscription(
            Bool, '/is_listening',
            self._audio_control_callback, 10,
            callback_group=self.cb_group
        )

        # ── État interne ──────────────────────────────────────────────────────
        self.audio_enabled   = True
        self.audio_lock      = threading.Lock()
        self.vad             = None
        self.asr             = None
        self._model_lock     = threading.Lock()

        self.audio_queue     = queue.Queue(maxsize=5)
        self._running        = True
        self._models_ready   = threading.Event()
        self._inference_done = threading.Event()

        self._fc_model  = fc_model
        self._fc_tokens = fc_tokens
        self._vad_path  = vad_path
        self._vad_th_s  = vad_th_s
        self._vad_th_e  = vad_th_e

        threading.Thread(target=self._inference_loop, daemon=True).start()
        if not self._models_ready.wait(timeout=60.0):
            self.get_logger().error('Timeout chargement modèles !')
            return
        threading.Thread(target=self._audio_capture_loop, daemon=True).start()

    # ──────────────────────────────────────────────────────────────────────────
    def _audio_control_callback(self, msg: Bool):
        with self.audio_lock:
            self.audio_enabled = msg.data
        self.get_logger().info('🎙️  Écoute activée' if msg.data else '🔇 Écoute désactivée')

    # ──────────────────────────────────────────────────────────────────────────
    def _audio_capture_loop(self):

        pa = pyaudio.PyAudio()
        kwargs = dict(
            format=pyaudio.paInt16,
            channels=1,
            rate=SR,
            input=True,
            frames_per_buffer=self.chunk_size,
        )
        if self.device_index >= 0:
            kwargs['input_device_index'] = self.device_index

        stream = pa.open(**kwargs)
        self.get_logger().info(
            f'🎙️  Micro ouvert (device={self.device_index}) | signal brut → FastConformer'
        )

        chunk_dur = self.chunk_size / SR   # ~0.032s pour chunk_size=512

        # ── Seuils énergie (sur signal BRUT) ──────────────────────────────────
        RMS_FLOOR      = 0.003   # forçage is_sp=False sur vrai silence
        RMS_RESUME_MIN = 0.008   # énergie mini pour compter un chunk comme
                                 # "candidat reprise" en MAYBE_END. Filtre
                                 # les pseudo-reprises sur bruit ambiant
                                 # de faible énergie (respiration, réverb).

        MIN_SPEECH_MS     = int(self.min_speech    * 1000)   # 200ms
        END_SILENCE_MS    = int(self.min_silence   * 1000)   # 600ms
        MAX_SPEECH_MS     = int(self.max_speech    * 1000)   # 10s
        MIN_FIRST_WORD_MS = 200
        N_RESUME          = max(1, int(self.resume_consecutive))

        # ── State machine ─────────────────────────────────────────────────────
        # États :
        #   IDLE         : pas de parole, on accumule le pre_buffer
        #   PRE_SPEECH   : parole détectée récemment, pas encore validée
        #   SPEECH       : parole en cours
        #   MAYBE_END    : silence accumulé, on attend confirmation de fin
        #                  OU N chunks consécutifs de parole pour reprendre
        state               = "IDLE"
        pre_buffer          = deque(maxlen=int(self.pre_buffer_ms / chunk_dur))
        pending_speech      = []
        speech_buffer       = []
        silence_ms          = 0.0
        speech_ms           = 0.0
        consecutive_resume  = 0    # compteur chunks is_sp=True en MAYBE_END
        debug_tick          = 0

        with self._model_lock:
            assert self.vad is not None
        self.vad.reset()

        # ── Helpers ───────────────────────────────────────────────────────────
        def reset():
            nonlocal state, silence_ms, speech_ms, speech_buffer
            nonlocal pending_speech, consecutive_resume
            state              = "IDLE"
            silence_ms         = 0.0
            speech_ms          = 0.0
            speech_buffer      = []
            pending_speech     = []
            consecutive_resume = 0
            self.vad.reset()

        def flush_and_send():
            """Envoie le speech_buffer à la queue puis remet la machine à zéro."""
            if speech_buffer:
                wav = np.concatenate(speech_buffer)
                try:
                    self.audio_queue.put_nowait(wav)
                except queue.Full:
                    self.get_logger().warn('Queue pleine — segment ignoré')
            reset()

        # ── Boucle principale ─────────────────────────────────────────────────
        while self._running:

            with self.audio_lock:
                enabled = self.audio_enabled

            if not enabled:
                try:
                    stream.read(self.chunk_size, exception_on_overflow=False)
                except Exception:
                    pass
                reset()
                time.sleep(0.05)
                continue

            try:
                raw   = stream.read(self.chunk_size, exception_on_overflow=False)
                chunk = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0

                rms = float(np.sqrt(np.mean(chunk ** 2)))

                # Hystérésis Silero
                currently_active = state in ("SPEECH", "MAYBE_END")
                with self._model_lock:
                    is_sp, prob = self.vad.is_speech(chunk, currently_active)

                # Sécurité énergie UNIQUEMENT dans le sens silence
                if is_sp and rms < RMS_FLOOR:
                    is_sp = False

                # ── State machine ─────────────────────────────────────────────
                if state == "IDLE":
                    pre_buffer.append(chunk)
                    if is_sp:
                        state          = "PRE_SPEECH"
                        pending_speech = [chunk]
                        speech_ms      = chunk_dur * 1000

                elif state == "PRE_SPEECH":
                    if is_sp:
                        pending_speech.append(chunk)
                        speech_ms += chunk_dur * 1000
                        if speech_ms > MIN_SPEECH_MS:
                            # Validation : on garde TOUT le pre_buffer (800ms)
                            state          = "SPEECH"
                            speech_buffer  = list(pre_buffer) + pending_speech
                            pre_buffer.clear()
                            pending_speech = []
                    else:
                        # Faux départ : recycler dans le pre_buffer
                        for c in pending_speech:
                            pre_buffer.append(c)
                        pending_speech = []
                        state          = "IDLE"
                        speech_ms      = 0.0

                elif state == "SPEECH":
                    speech_buffer.append(chunk)
                    speech_ms += chunk_dur * 1000
                    if is_sp:
                        silence_ms = 0.0
                    else:
                        state              = "MAYBE_END"
                        silence_ms         = chunk_dur * 1000
                        consecutive_resume = 0

                    if speech_ms > MAX_SPEECH_MS:
                        flush_and_send()

                elif state == "MAYBE_END":
                    # On continue à accumuler l'audio (on ne perd rien)
                    speech_buffer.append(chunk)

                    # Candidat reprise : prob ≥ th_end ET énergie suffisante.
                    # La condition RMS évite les pseudo-reprises sur bruit
                    # ambiant de faible énergie (respiration, réverb).
                    is_real_resume = is_sp and rms >= RMS_RESUME_MIN

                    if is_real_resume:
                        consecutive_resume += 1

                        if self.vad_debug:
                            self.get_logger().info(
                                f'[MAYBE_END] ↑ candidat reprise '
                                f'{consecutive_resume}/{N_RESUME} '
                                f'prob={prob:.3f} rms={rms:.4f}'
                            )

                        if consecutive_resume >= N_RESUME:
                            # Reprise confirmée → retour SPEECH
                            state              = "SPEECH"
                            silence_ms         = 0.0
                            consecutive_resume = 0
                            if self.vad_debug:
                                self.get_logger().info(
                                    '[MAYBE_END → SPEECH] reprise confirmée'
                                )
                    else:
                        # Chunk silencieux OU prob élevée mais faible énergie :
                        # on invalide le compteur reprise et on accumule silence.
                        consecutive_resume = 0
                        silence_ms += chunk_dur * 1000

                        if self.vad_debug:
                            debug_tick += 1
                            if debug_tick % 6 == 0:
                                self.get_logger().info(
                                    f'[MAYBE_END] silence={silence_ms:.0f}ms '
                                    f'speech={speech_ms:.0f}ms '
                                    f'prob={prob:.3f} rms={rms:.4f}'
                                )

                        if silence_ms > END_SILENCE_MS:
                            if speech_ms < MIN_FIRST_WORD_MS:
                                reset()
                            else:
                                flush_and_send()
                        elif speech_ms > MAX_SPEECH_MS:
                            flush_and_send()

            except Exception as e:
                self.get_logger().warn(f'Audio error : {e}')

        stream.stop_stream()
        stream.close()
        pa.terminate()

    # ──────────────────────────────────────────────────────────────────────────
    def _inference_loop(self):
        try:
            self.get_logger().info('⚡ Chargement FastConformer CTC CUDA...')
            asr = FastConformerASR(self._fc_model, self._fc_tokens)
            self.get_logger().info(f'  ✅ FastConformer : {os.path.basename(self._fc_model)}')
            self.get_logger().info(f'  ✅ Vocab         : {len(asr.vocab)} tokens')
            self.get_logger().info(f'  ✅ Blank token   : index {asr.blank}')

            vad = SileroVAD(self._vad_path, self._vad_th_s, self._vad_th_e)
            self.get_logger().info(
                f'  ✅ VAD           : silero_vad.onnx '
                f'(hystérésis {self._vad_th_s:.2f}/{self._vad_th_e:.2f}, '
                f'resume_consecutive={self.resume_consecutive})'
            )

            with self._model_lock:
                self.asr = asr
                self.vad = vad

            self.get_logger().info('🔥 Warmup FastConformer...')
            t_warmup = time.perf_counter()
            self.asr.transcribe(np.zeros(SR, dtype=np.float32))
            dt_warmup = time.perf_counter() - t_warmup
            self.get_logger().info(f'✅ STT FastConformer CTC FR prêt ({dt_warmup:.1f}s) — en écoute')
            self._models_ready.set()

            while self._running:
                try:
                    wav = self.audio_queue.get(timeout=0.5)
                except queue.Empty:
                    continue
                if wav is None:
                    break

                dur = len(wav) / SR

                # Rejet préventif avant inférence
                if dur < self.min_wav_dur:
                    self.get_logger().debug(f'Segment court ({dur:.2f}s) — ignoré')
                    continue
                if float(np.sqrt(np.mean(wav ** 2))) < 0.005:
                    self.get_logger().debug('Segment silencieux — ignoré')
                    continue

                # ── Inférence FastConformer directement sur le signal brut ────
                text, rtf = self.asr.transcribe(wav)
                text      = _clean_transcription(text)

                if rtf > self.rtf_warn_thr:
                    self.get_logger().warn(f'⚠️  RTF élevé : {rtf:.3f}')
                if not _is_valid_transcription(text, self.min_words):
                    self.get_logger().debug(f'Filtré : [{text!r}]')
                    continue

                self.get_logger().info(f'💬 [{dur:.2f}s | RTF={rtf:.3f}] → {text}')
                msg      = String()
                msg.data = text
                self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Inference thread fatal : {e}')
            self._models_ready.set()
        finally:
            self._inference_done.set()

    # ──────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        try:
            self.audio_queue.put_nowait(None)
        except queue.Full:
            pass
        self._inference_done.wait(timeout=5.0)
        super().destroy_node()


# ══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()