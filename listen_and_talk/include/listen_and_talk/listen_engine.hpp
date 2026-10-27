/**
 * @file    listen_engine.hpp
 * @brief   Moteur STT de Néo (indépendant de ROS).
 *
 *   AudioInput (callback PA → ring 16 kHz)
 *        │  thread "proc"  : VAD sherpa-onnx (Silero ou TEN) par blocs 32 ms
 *        │                   + gestion écho / barge-in + découpe segments
 *        ▼
 *   file bornée de segments
 *        │  thread "asr"   : Parakeet TDT 0.6B v3 (ou NeMo CTC) via sherpa-onnx
 *        ▼
 *   callback texte  →  (node ROS) /listened
 *
 * Trois threads découplés : la capture ne peut pas être bloquée par la VAD,
 * la VAD ne peut pas être bloquée par l'ASR. Files bornées partout →
 * mémoire constante, aucune dérive.
 *
 * ÉCHO (la voix de Néo captée par son propre micro) :
 * le haut-parleur (C-Media) et le micro (ReSpeaker) sont sur deux cartes
 * différentes : l'AEC matériel du ReSpeaker n'a pas la référence. Sans
 * précaution, Néo s'auto-interrompt et transcrit ses propres phrases.
 * Stratégie :
 *  - pendant /is_speaking (+ traîne `echo_tail_ms`), les segments VAD sont
 *    marqués "écho" et rejetés ;
 *  - barge-in : si l'énergie micro dépasse nettement le niveau d'écho
 *    appris (`barge_in_ratio`) avec VAD active pendant `barge_in_ms`, on
 *    déclare une interruption utilisateur → /interrupt_tts, et le segment
 *    est conservé (rogné au début de l'intervention) ;
 *  - un segment qui déborde après la fin de la parole robot est rogné et
 *    conservé (réponse rapide de l'utilisateur non perdue).
 *
 * @author  Vincent FOUCAULT — Octobre 2026
 * @license MIT
 */
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include "listen_and_talk/audio_dsp.hpp"
#include "listen_and_talk/audio_input.hpp"
#include "sherpa-onnx/c-api/c-api.h"

namespace lat {

// ═════════════════════════════════════════════════════════════════════════════
//  ASR (sherpa-onnx OfflineRecognizer)
// ═════════════════════════════════════════════════════════════════════════════
class AsrEngine {
 public:
  struct Config {
    std::string model_type = "nemo_transducer";  // nemo_transducer | nemo_ctc
    std::string encoder, decoder, joiner;        // transducer (Parakeet TDT)
    std::string ctc_model;                       // nemo_ctc (FastConformer CTC)
    std::string tokens;
    std::string provider = "cpu";
    int num_threads = 4;
  };

  struct Result {
    std::string text;
    float confidence = -1.0f;  // exp(moyenne log-prob tokens), -1 si inconnu
    double compute_ms = 0.0;
  };

  AsrEngine(Config cfg, LogFn log) : cfg_(std::move(cfg)), log_(std::move(log)) {}
  ~AsrEngine() {
    if (rec_) SherpaOnnxDestroyOfflineRecognizer(rec_);
  }
  AsrEngine(const AsrEngine&) = delete;
  AsrEngine& operator=(const AsrEngine&) = delete;

  bool init() {
    rec_ = create(cfg_.provider);
    if (!rec_ && cfg_.provider != "cpu") {
      log(1, "ASR : échec provider '" + cfg_.provider + "', repli sur CPU");
      rec_ = create("cpu");
    }
    if (!rec_) {
      log(2, "ASR : impossible de charger le modèle (" + cfg_.model_type + ")");
      return false;
    }
    log(0, "ASR : modèle " + cfg_.model_type + " chargé");
    return true;
  }

  Result decode(const float* x, size_t n, int sample_rate = 16000) {
    Result r;
    const auto t0 = std::chrono::steady_clock::now();
    const SherpaOnnxOfflineStream* s = SherpaOnnxCreateOfflineStream(rec_);
    SherpaOnnxAcceptWaveformOffline(s, sample_rate, x, static_cast<int32_t>(n));
    SherpaOnnxDecodeOfflineStream(rec_, s);
    const SherpaOnnxOfflineRecognizerResult* res = SherpaOnnxGetOfflineStreamResult(s);
    if (res) {
      if (res->text) r.text = res->text;
#ifdef LAT_HAS_YS_LOG_PROBS
      if (res->ys_log_probs && res->count > 0) {
        double sum = 0.0;
        for (int i = 0; i < res->count; ++i) sum += res->ys_log_probs[i];
        r.confidence = static_cast<float>(std::exp(sum / res->count));
      }
#endif
      SherpaOnnxDestroyOfflineRecognizerResult(res);
    }
    SherpaOnnxDestroyOfflineStream(s);
    r.compute_ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - t0).count();
    return r;
  }

 private:
  const SherpaOnnxOfflineRecognizer* create(const std::string& provider) {
    SherpaOnnxOfflineRecognizerConfig c;
    std::memset(&c, 0, sizeof(c));
    c.feat_config.sample_rate = 16000;
    c.feat_config.feature_dim = 80;  // ignoré par NeMo (lu dans les métadonnées)
    if (cfg_.model_type == "nemo_ctc") {
      c.model_config.nemo_ctc.model = cfg_.ctc_model.c_str();
    } else {
      c.model_config.transducer.encoder = cfg_.encoder.c_str();
      c.model_config.transducer.decoder = cfg_.decoder.c_str();
      c.model_config.transducer.joiner = cfg_.joiner.c_str();
    }
    c.model_config.model_type = cfg_.model_type.c_str();
    c.model_config.tokens = cfg_.tokens.c_str();
    c.model_config.num_threads = cfg_.num_threads;
    c.model_config.provider = provider.c_str();
    c.decoding_method = "greedy_search";
    return SherpaOnnxCreateOfflineRecognizer(&c);
  }

  void log(int lvl, const std::string& m) const {
    if (log_) log_(lvl, m);
  }

  Config cfg_;
  LogFn log_;
  const SherpaOnnxOfflineRecognizer* rec_ = nullptr;
};

// ═════════════════════════════════════════════════════════════════════════════
//  Pipeline VAD + écho + découpe (thread "proc", aucune dépendance audio)
// ═════════════════════════════════════════════════════════════════════════════
class ListenPipeline {
 public:
  static constexpr int kRate = 16000;
  static constexpr int kChunk = 512;  // 32 ms

  struct Config {
    std::string vad_model;           // silero_vad.onnx ou ten-vad.onnx
    std::string vad_type = "auto";   // auto | silero | ten  (auto = d'après le NOM du fichier)
    float vad_threshold = 0.5f;
    float min_silence_s = 0.6f;      // fin de phrase
    float min_speech_s = 0.2f;       // parole mini pour ouvrir un segment
    float max_speech_s = 12.0f;      // découpe forcée
    int preroll_ms = 250;            // audio ajouté avant le segment
    int postroll_ms = 150;           // audio ajouté après
    float min_segment_s = 0.35f;     // segment final mini
    float min_segment_rms = 0.004f;  // segment trop faible = bruit
    // Écho / barge-in
    bool barge_in = true;
    int echo_tail_ms = 450;          // traîne après la fin de parole robot
    int echo_latency_ms = 120;       // latence carte son → micro
    int barge_in_grace_ms = 350;     // apprentissage du niveau d'écho
    int barge_in_ms = 160;           // temps de parole forte requis…
    int barge_in_window_ms = 640;    // …dans cette fenêtre glissante
    float barge_in_ratio = 2.0f;     // ×niveau d'écho (≈ +6 dB)
    float barge_in_min_rms = 0.02f;  // plancher absolu
  };

  using SegmentFn = std::function<void(std::vector<float>&&, bool after_barge_in)>;
  using EventFn = std::function<void()>;

  ListenPipeline(Config cfg, LogFn log, SegmentFn on_segment, EventFn on_barge_in)
      : cfg_(std::move(cfg)), log_(std::move(log)),
        on_segment_(std::move(on_segment)), on_barge_in_(std::move(on_barge_in)) {
    hist_.assign(static_cast<size_t>(kRate) * 30, 0.0f);
    pending_.reserve(kChunk * 8);
  }
  ~ListenPipeline() {
    if (vad_) SherpaOnnxDestroyVoiceActivityDetector(vad_);
  }
  ListenPipeline(const ListenPipeline&) = delete;
  ListenPipeline& operator=(const ListenPipeline&) = delete;

  bool init() {
    SherpaOnnxVadModelConfig c;
    std::memset(&c, 0, sizeof(c));
    // Type de VAD : explicite, ou déduit du NOM DE FICHIER uniquement
    // (le chemin contient "listen_and_talk" → ne jamais chercher "ten" dedans).
    bool ten = false;
    if (cfg_.vad_type == "ten") {
      ten = true;
    } else if (cfg_.vad_type == "auto") {
      std::string base = cfg_.vad_model.substr(cfg_.vad_model.find_last_of('/') + 1);
      for (auto& ch : base) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
      ten = base.rfind("ten-vad", 0) == 0 || base.rfind("ten_vad", 0) == 0;
    }
#ifdef LAT_HAS_TEN_VAD
    if (ten) {
      c.ten_vad.model = cfg_.vad_model.c_str();
      c.ten_vad.threshold = cfg_.vad_threshold;
      c.ten_vad.min_silence_duration = cfg_.min_silence_s;
      c.ten_vad.min_speech_duration = cfg_.min_speech_s;
      c.ten_vad.max_speech_duration = cfg_.max_speech_s;
      c.ten_vad.window_size = 256;
    } else
#else
    if (ten) log(1, "VAD : TEN-VAD non supporté par cette version de sherpa-onnx → Silero attendu");
#endif
    {
      c.silero_vad.model = cfg_.vad_model.c_str();
      c.silero_vad.threshold = cfg_.vad_threshold;
      c.silero_vad.min_silence_duration = cfg_.min_silence_s;
      c.silero_vad.min_speech_duration = cfg_.min_speech_s;
      c.silero_vad.max_speech_duration = cfg_.max_speech_s;
      c.silero_vad.window_size = 512;
    }
    c.sample_rate = kRate;
    c.num_threads = 1;
    c.provider = "cpu";  // modèle minuscule : le GPU coûterait plus qu'il ne rapporte
    vad_ = SherpaOnnxCreateVoiceActivityDetector(&c, 30.0f);
    if (!vad_) {
      log(2, "VAD : impossible de charger " + cfg_.vad_model);
      return false;
    }
    log(0, std::string("VAD : ") + (ten ? "TEN-VAD" : "Silero") + " chargé (seuil " +
               std::to_string(cfg_.vad_threshold).substr(0, 4) + ")");
    return true;
  }

  /// Thread quelconque.
  void set_speaking(bool s) { speaking_.store(s); }
  void set_enabled(bool e) { enabled_.store(e); }
  bool enabled() const { return enabled_.load(); }

  /// Thread "proc" uniquement : nombre quelconque d'échantillons 16 kHz.
  void process(const float* x, size_t n) {
    pending_.insert(pending_.end(), x, x + n);
    size_t off = 0;
    while (pending_.size() - off >= kChunk) {
      step(pending_.data() + off);
      off += kChunk;
    }
    pending_.erase(pending_.begin(), pending_.begin() + static_cast<long>(off));
  }

  /// Position courante (échantillons depuis le démarrage).
  int64_t position() const { return n_; }

 private:
  struct EchoInterval {
    int64_t a;      // début parole robot
    int64_t b_raw;  // fin parole robot (+latence)
    int64_t b;      // fin + traîne
  };
  static constexpr int64_t kOpen = std::numeric_limits<int64_t>::max() / 4;

  void log(int lvl, const std::string& m) const {
    if (log_) log_(lvl, m);
  }

  static int64_t ms2s(int ms) { return static_cast<int64_t>(ms) * kRate / 1000; }

  void step(const float* c) {
    // Historique circulaire indexé en absolu.
    const size_t H = hist_.size();
    for (int i = 0; i < kChunk; ++i) hist_[static_cast<size_t>(n_ + i) % H] = c[i];

    // Fronts de /is_speaking → intervalles d'écho (timeline micro).
    const bool spk = speaking_.load();
    if (spk && !was_speaking_) {
      echo_.push_back({n_, kOpen, kOpen});
      spk_start_ = n_;
      echo_lvl_ = 0.0f;
      bi_acc_ms_ = 0;
      bi_hist_.clear();
      bi_fired_ = false;
      barge_at_ = -1;
    } else if (!spk && was_speaking_ && !echo_.empty()) {
      echo_.back().b_raw = n_ + ms2s(cfg_.echo_latency_ms);
      echo_.back().b = n_ + ms2s(cfg_.echo_tail_ms);
    }
    was_speaking_ = spk;

    if (!enabled_.load()) {
      if (vad_fed_) {  // écoute coupée : on repart d'une VAD propre
        SherpaOnnxVoiceActivityDetectorReset(vad_);
        vad_fed_ = false;
      }
      n_ += kChunk;
      return;
    }
    if (!vad_fed_) {
      vad_base_ = n_;
      vad_fed_ = true;
    }

    SherpaOnnxVoiceActivityDetectorAcceptWaveform(vad_, c, kChunk);
    const bool detected = SherpaOnnxVoiceActivityDetectorDetected(vad_) != 0;
    const float r = rms(c, kChunk);

    // ── Barge-in ──────────────────────────────────────────────────────────
    if (cfg_.barge_in && spk && !bi_fired_) {
      // Niveau d'écho = enveloppe crête du RMS micro pendant la parole robot.
      //  - phase de grâce : apprentissage rapide (max) ;
      //  - ensuite : montée LENTE (2 %/bloc) et descente lente (~6 s) →
      //    l'irruption soudaine d'une voix humaine dépasse le seuil avant
      //    que l'enveloppe ne la rattrape (sinon le seuil "suit" l'utilisateur).
      const int elapsed_ms = static_cast<int>((n_ - spk_start_) * 1000 / kRate);
      const float thr = std::max(cfg_.barge_in_min_rms, echo_lvl_ * cfg_.barge_in_ratio);
      if (elapsed_ms < cfg_.barge_in_grace_ms) {
        echo_lvl_ = std::max(r, echo_lvl_);
      } else {
        // Fenêtre glissante : la parole humaine arrive par bouffées
        // syllabiques (~100 ms) → on compte le temps "fort" sur la fenêtre.
        const bool loud = detected && r > thr;
        bi_acc_ms_ += loud ? 32 : 0;
        bi_hist_.push_back(loud);
        const size_t win = static_cast<size_t>(std::max(1, cfg_.barge_in_window_ms / 32));
        while (bi_hist_.size() > win) {
          if (bi_hist_.front()) bi_acc_ms_ -= 32;
          bi_hist_.pop_front();
        }
        if (bi_acc_ms_ >= cfg_.barge_in_ms) {
          bi_fired_ = true;
          barge_at_ = n_;
          log(0, "✋ Barge-in (rms " + fmt(r) + " > seuil " + fmt(thr) + ")");
          if (on_barge_in_) on_barge_in_();
        }
        if (r > echo_lvl_) echo_lvl_ += (r - echo_lvl_) * 0.02f;
        else echo_lvl_ = std::max(r, echo_lvl_ * 0.995f);
      }
    }

    n_ += kChunk;

    // ── Segments terminés ─────────────────────────────────────────────────
    while (!SherpaOnnxVoiceActivityDetectorEmpty(vad_)) {
      const SherpaOnnxSpeechSegment* seg = SherpaOnnxVoiceActivityDetectorFront(vad_);
      if (seg && seg->n > 0)
        handle_segment(vad_base_ + static_cast<int64_t>(seg->start), seg->n);
      if (seg) SherpaOnnxDestroySpeechSegment(seg);
      SherpaOnnxVoiceActivityDetectorPop(vad_);
    }

    // Purge des vieux intervalles.
    while (!echo_.empty() && echo_.front().b != kOpen && echo_.front().b < n_ - ms2s(60000))
      echo_.pop_front();
  }

  void handle_segment(int64_t s, int32_t len) {
    const int64_t e = s + len;
    int64_t start = s;
    bool barge = false;

    for (const auto& iv : echo_) {
      if (!(s < iv.b && e > iv.a)) continue;  // pas de chevauchement
      if (barge_at_ >= 0 && barge_at_ >= iv.a && barge_at_ <= iv.b && e > barge_at_) {
        start = std::max(start, barge_at_ - ms2s(cfg_.barge_in_window_ms));
        barge = true;
      } else if (iv.b != kOpen && e > iv.b) {
        start = std::max(start, iv.b_raw);  // réponse rapide : on garde la suite
      } else {
        log(0, "🔁 Segment d'écho ignoré (" + fmt(len / float(kRate)) + " s)");
        return;
      }
    }

    // Pré/post-roll pris dans l'historique (bornés à ce qui existe).
    const int64_t H = static_cast<int64_t>(hist_.size());
    int64_t a = std::max<int64_t>({start - ms2s(cfg_.preroll_ms), n_ - H + 1, 0});
    if (start > s) a = std::max(a, start);  // ne pas ré-inclure l'écho rogné
    const int64_t b = std::min<int64_t>(e + ms2s(cfg_.postroll_ms), n_);
    if (b - a < static_cast<int64_t>(cfg_.min_segment_s * kRate)) return;

    std::vector<float> out(static_cast<size_t>(b - a));
    for (int64_t i = a; i < b; ++i) out[static_cast<size_t>(i - a)] = hist_[static_cast<size_t>(i % H)];
    if (rms(out.data(), out.size()) < cfg_.min_segment_rms) return;
    if (on_segment_) on_segment_(std::move(out), barge);
  }

  static std::string fmt(float v) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.3f", v);
    return b;
  }

  Config cfg_;
  LogFn log_;
  SegmentFn on_segment_;
  EventFn on_barge_in_;
  const SherpaOnnxVoiceActivityDetector* vad_ = nullptr;

  std::vector<float> hist_;
  std::vector<float> pending_;
  int64_t n_ = 0;
  int64_t vad_base_ = 0;
  bool vad_fed_ = false;

  std::atomic<bool> speaking_{false};
  std::atomic<bool> enabled_{true};
  bool was_speaking_ = false;
  std::deque<EchoInterval> echo_;
  int64_t spk_start_ = 0;
  float echo_lvl_ = 0.0f;
  int bi_acc_ms_ = 0;
  std::deque<bool> bi_hist_;
  bool bi_fired_ = false;
  int64_t barge_at_ = -1;
};

// ═════════════════════════════════════════════════════════════════════════════
//  Filtres de texte post-ASR
// ═════════════════════════════════════════════════════════════════════════════
struct TextFilter {
  size_t min_chars = 2;
  std::vector<std::string> ignore_words{"euh", "heu", "hum", "hmm", "mmh", "bah", "ah", "oh"};

  /// Retourne le texte nettoyé, ou "" s'il doit être rejeté.
  std::string apply(const std::string& in) const {
    static const std::regex ws(R"(\s+)");
    static const std::regex tags(R"(<[^>]*>)");
    std::string t = std::regex_replace(in, tags, " ");
    t = std::regex_replace(t, ws, " ");
    t = trim(t);
    if (t.size() < min_chars) return {};

    // Au moins une lettre (ASCII ou UTF-8 multi-octets, ex. "é").
    bool letter = false;
    for (unsigned char ch : t)
      if (std::isalpha(ch) || ch >= 0xC3) { letter = true; break; }
    if (!letter) return {};

    // Rejet si le texte n'est composé que de mots "parasites".
    std::string low;
    for (unsigned char ch : t) low += static_cast<char>(std::tolower(ch));
    std::string word;
    bool only_fillers = true;
    auto flush = [&]() {
      if (word.empty()) return;
      if (std::find(ignore_words.begin(), ignore_words.end(), word) == ignore_words.end())
        only_fillers = false;
      word.clear();
    };
    for (char ch : low) {
      if (std::isalnum(static_cast<unsigned char>(ch)) || static_cast<unsigned char>(ch) >= 0x80)
        word += ch;
      else
        flush();
    }
    flush();
    if (only_fillers) return {};
    return t;
  }

  static std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\n");
    if (b == std::string::npos) return {};
    const auto e = s.find_last_not_of(" \t\n");
    return s.substr(b, e - b + 1);
  }
};

// ═════════════════════════════════════════════════════════════════════════════
//  ListenEngine : capture + pipeline + ASR, 2 threads de travail
// ═════════════════════════════════════════════════════════════════════════════
class ListenEngine {
 public:
  struct Output {
    std::string text;
    float confidence;
    double duration_s;
    double compute_ms;
    bool after_barge_in;
  };
  using TextFn = std::function<void(const Output&)>;

  ListenEngine(AudioInput* in, ListenPipeline::Config pcfg, AsrEngine::Config acfg,
               TextFilter filter, LogFn log, TextFn on_text, std::function<void()> on_barge_in)
      : in_(in), log_(log), on_text_(std::move(on_text)), filter_(std::move(filter)),
        asr_(std::move(acfg), log),
        pipe_(std::move(pcfg), log,
              [this](std::vector<float>&& s, bool b) { enqueue(std::move(s), b); },
              std::move(on_barge_in)) {}

  ~ListenEngine() { stop(); }

  bool init() {
    if (!pipe_.init()) return false;
    if (!asr_.init()) return false;
    // Chauffe (1 s de silence) : charge les noyaux, évite la latence du 1er tour.
    std::vector<float> z(16000, 0.0f);
    const auto r = asr_.decode(z.data(), z.size());
    log(0, "ASR : warmup " + std::to_string(static_cast<int>(r.compute_ms)) + " ms");
    return true;
  }

  void start() {
    if (running_.exchange(true)) return;
    if (in_) proc_ = std::thread([this] { proc_loop(); });
    asr_thr_ = std::thread([this] { asr_loop(); });
  }

  void stop() {
    if (!running_.exchange(false)) return;
    q_cv_.notify_all();
    if (proc_.joinable()) proc_.join();
    if (asr_thr_.joinable()) asr_thr_.join();
  }

  ListenPipeline& pipeline() { return pipe_; }
  AsrEngine& asr() { return asr_; }

  /// Tests hors-ligne : injecte de l'audio 16 kHz comme s'il venait du micro.
  void feed(const float* x, size_t n) { pipe_.process(x, n); }

  /// Attente de vidage de la file ASR (tests).
  void drain() {
    for (;;) {
      {
        std::lock_guard<std::mutex> lk(q_mtx_);
        if (queue_.empty() && !decoding_) return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

 private:
  struct Seg {
    std::vector<float> wav;
    bool barge;
  };

  void log(int lvl, const std::string& m) const {
    if (log_) log_(lvl, m);
  }

  void enqueue(std::vector<float>&& w, bool barge) {
    {
      std::lock_guard<std::mutex> lk(q_mtx_);
      if (queue_.size() >= 3) {
        queue_.pop_front();
        log(1, "ASR saturé : plus ancien segment abandonné");
      }
      queue_.push_back({std::move(w), barge});
    }
    q_cv_.notify_one();
  }

  void proc_loop() {
    std::vector<float> buf(2048);
    uint64_t last_ovf = 0;
    while (running_.load()) {
      const size_t n = in_->read(buf.data(), buf.size());
      if (n == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        continue;
      }
      pipe_.process(buf.data(), n);
      const uint64_t ovf = in_->overflows();
      if (ovf != last_ovf) {
        log(1, "Micro : " + std::to_string(ovf - last_ovf) + " échantillons perdus");
        last_ovf = ovf;
      }
    }
  }

  void asr_loop() {
    while (running_.load()) {
      Seg seg;
      {
        std::unique_lock<std::mutex> lk(q_mtx_);
        q_cv_.wait(lk, [this] { return !queue_.empty() || !running_.load(); });
        if (!running_.load()) break;
        seg = std::move(queue_.front());
        queue_.pop_front();
        decoding_ = true;
      }
      const double dur = seg.wav.size() / 16000.0;
      const auto r = asr_.decode(seg.wav.data(), seg.wav.size());
      const std::string t = filter_.apply(r.text);
      if (t.empty()) {
        if (!r.text.empty()) log(0, "Filtré : [" + r.text + "]");
      } else if (on_text_) {
        on_text_({t, r.confidence, dur, r.compute_ms, seg.barge});
      }
      std::lock_guard<std::mutex> lk(q_mtx_);
      decoding_ = false;
    }
  }

  AudioInput* in_;
  LogFn log_;
  TextFn on_text_;
  TextFilter filter_;
  AsrEngine asr_;
  ListenPipeline pipe_;

  std::atomic<bool> running_{false};
  std::thread proc_, asr_thr_;
  std::mutex q_mtx_;
  std::condition_variable q_cv_;
  std::deque<Seg> queue_;
  bool decoding_ = false;
};

}  // namespace lat
