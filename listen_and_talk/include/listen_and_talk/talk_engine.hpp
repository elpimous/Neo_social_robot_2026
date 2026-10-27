/**
 * @file    talk_engine.hpp
 * @brief   Moteur TTS de Néo (indépendant de ROS) : voix VITS "axel"
 *          via sherpa-onnx, synthèse PHRASE PAR PHRASE en streaming.
 *
 * Pipeline :
 *   texte → nettoyage → sherpa-onnx VITS (max_num_sentences=1, callback)
 *         → normalisation crête + gain + limiteur doux
 *         → rééchantillonnage polyphasé 22050 → taux carte (44100/48000)
 *         → ring buffer → AudioOutput (callback PortAudio)
 *
 * Gain de latence : la 1re phrase est jouée pendant que la suivante est
 * synthétisée (avant : tout le texte devait être généré avant le 1er son).
 *
 * Interruption : compteur d'époque. interrupt() incrémente l'époque,
 * vide la file, coupe la génération en cours (callback → 0) et demande
 * un fondu au périphérique. Aucune donnée d'une époque périmée ne peut
 * être écrite après la demande de vidage (verrou write_mtx_).
 *
 * @author  Vincent FOUCAULT — Octobre 2026
 * @license MIT
 */
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include "listen_and_talk/audio_dsp.hpp"
#include "listen_and_talk/audio_output.hpp"
#include "sherpa-onnx/c-api/c-api.h"

namespace lat {

class TalkEngine {
 public:
  struct Config {
    std::string model, tokens, data_dir;   // fichiers VITS (axel.onnx …)
    std::string provider = "cuda";
    int num_threads = 2;
    int sid = 0;
    float speed = 1.0f;
    float sentence_pause_ms = 140.0f;      // silence inséré entre 2 phrases
    bool normalize = true;                 // normalisation crête par phrase
    float target_peak = 0.89f;             // -1 dBFS
    float max_norm_gain = 4.0f;            // gain de normalisation max
    float volume_gain = 1.0f;              // gain utilisateur (modifiable à chaud)
    size_t queue_max = 16;                 // textes en attente max
    size_t max_text_chars = 1200;          // garde-fou
  };

  struct Stats {
    double first_audio_ms = 0;  // texte reçu → 1er échantillon dans le ring
    double synth_ms = 0;        // temps total de synthèse
    double audio_ms = 0;        // durée audio produite
  };

  TalkEngine(Config cfg, AudioOutput* out, LogFn log)
      : cfg_(std::move(cfg)), out_(out), log_(std::move(log)) {
    set_volume_gain(cfg_.volume_gain);
    set_speed(cfg_.speed);
    set_sid(cfg_.sid);
  }

  ~TalkEngine() {
    stop();
    if (tts_) SherpaOnnxDestroyOfflineTts(tts_);
  }

  TalkEngine(const TalkEngine&) = delete;
  TalkEngine& operator=(const TalkEngine&) = delete;

  /// Charge le modèle. Retourne false en cas d'échec.
  bool init() {
    SherpaOnnxOfflineTtsConfig c;
    std::memset(&c, 0, sizeof(c));
    c.model.vits.model = cfg_.model.c_str();
    c.model.vits.tokens = cfg_.tokens.c_str();
    c.model.vits.data_dir = cfg_.data_dir.c_str();
    c.model.vits.noise_scale = 0.667f;
    c.model.vits.noise_scale_w = 0.8f;
    c.model.vits.length_scale = 1.0f;
    c.model.num_threads = cfg_.num_threads;
    c.model.provider = cfg_.provider.c_str();
    c.max_num_sentences = 1;  // ← streaming phrase par phrase
    tts_ = SherpaOnnxCreateOfflineTts(&c);
    if (!tts_ && cfg_.provider != "cpu") {
      log(1, "TTS : échec provider '" + cfg_.provider + "', repli sur CPU");
      c.model.provider = "cpu";
      tts_ = SherpaOnnxCreateOfflineTts(&c);
    }
    if (!tts_) {
      log(2, "TTS : impossible de charger " + cfg_.model);
      return false;
    }
    model_rate_ = SherpaOnnxOfflineTtsSampleRate(tts_);
    log(0, "TTS : voix chargée (" + std::to_string(model_rate_) + " Hz, " + c.model.provider + ")");
    return true;
  }

  int model_sample_rate() const { return model_rate_; }

  /// Chauffe le GPU (inférence muette) : évite les ~500 ms du 1er appel.
  void warmup(const std::string& text = "Bonjour.") {
    std::vector<float> dummy;
    synthesize_to_buffer(text, dummy);
  }

  void start() {
    if (running_.exchange(true)) return;
    worker_ = std::thread([this] { loop(); });
  }

  void stop() {
    if (!running_.exchange(false)) return;
    {
      std::lock_guard<std::mutex> lk(q_mtx_);
      ++epoch_;
    }
    q_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

  /// Ajoute un texte à dire (thread quelconque, non bloquant).
  void say(const std::string& raw) {
    std::string text = clean_text(raw);
    if (text.empty()) return;
    {
      std::lock_guard<std::mutex> lk(q_mtx_);
      if (queue_.size() >= cfg_.queue_max) {
        queue_.pop_front();
        log(1, "TTS : file pleine, plus ancien texte abandonné");
      }
      queue_.push_back({std::move(text), epoch_.load(), std::chrono::steady_clock::now()});
    }
    q_cv_.notify_one();
  }

  /// Coupe immédiatement la parole en cours et vide la file.
  void interrupt() {
    bool was_busy = busy();
    {
      std::lock_guard<std::mutex> wl(write_mtx_);
      std::lock_guard<std::mutex> lk(q_mtx_);
      ++epoch_;
      queue_.clear();
      if (out_) out_->request_flush();
    }
    q_cv_.notify_all();
    if (was_busy) log(0, "🛑 TTS interrompu");
  }

  /// Vrai tant que quelque chose est en file, en synthèse ou en lecture.
  bool busy() const {
    if (synthesizing_.load()) return true;
    {
      std::lock_guard<std::mutex> lk(q_mtx_);
      if (!queue_.empty()) return true;
    }
    return out_ && (out_->queued() > 0 || out_->playing());
  }

  void set_volume_gain(float g) { gain_.store(std::max(0.0f, g)); }
  void set_speed(float s) { speed_.store(std::clamp(s, 0.5f, 2.0f)); }
  void set_sid(int sid) { sid_.store(sid); }

  /// Synthèse complète dans un buffer (tests / warmup) au taux du modèle.
  bool synthesize_to_buffer(const std::string& text, std::vector<float>& out) {
    SherpaOnnxGenerationConfig g;
    std::memset(&g, 0, sizeof(g));
    g.sid = sid_.load();
    g.speed = speed_.load();
    g.silence_scale = 0.2f;
    const SherpaOnnxGeneratedAudio* a =
        SherpaOnnxOfflineTtsGenerateWithConfig(tts_, text.c_str(), &g, nullptr, nullptr);
    if (!a) return false;
    out.assign(a->samples, a->samples + a->n);
    SherpaOnnxDestroyOfflineTtsGeneratedAudio(a);
    return true;
  }

  /// Mode test (out == nullptr) : reçoit l'audio final au taux `test_rate`.
  void set_test_sink(std::function<void(const float*, size_t)> sink, int test_rate) {
    sink_ = std::move(sink);
    test_rate_ = test_rate;
  }

  Stats last_stats() const {
    std::lock_guard<std::mutex> lk(stats_mtx_);
    return stats_;
  }

  /// Nettoyage du texte : balises <...>, markdown, espaces multiples.
  static std::string clean_text(const std::string& in) {
    static const std::regex tags(R"(<[^>]*>)");
    static const std::regex md(R"([*_#`~|\[\]{}])");
    static const std::regex ws(R"(\s+)");
    std::string t = std::regex_replace(in, tags, " ");
    t = std::regex_replace(t, md, " ");
    t = std::regex_replace(t, ws, " ");
    const auto b = t.find_first_not_of(' ');
    if (b == std::string::npos) return {};
    const auto e = t.find_last_not_of(' ');
    return t.substr(b, e - b + 1);
  }

 private:
  struct Request {
    std::string text;
    uint64_t epoch;
    std::chrono::steady_clock::time_point received;
  };

  struct GenCtx {
    TalkEngine* self;
    uint64_t epoch;
    PolyphaseResampler* rs;
    std::vector<float> tmp;
    std::vector<float> res;
    bool first = true;
    std::chrono::steady_clock::time_point received;
    double audio_ms = 0;
  };

  void log(int lvl, const std::string& m) const {
    if (log_) log_(lvl, m);
  }

  void loop() {
    std::unique_ptr<PolyphaseResampler> rs;
    while (running_.load()) {
      Request req;
      {
        std::unique_lock<std::mutex> lk(q_mtx_);
        q_cv_.wait(lk, [this] { return !queue_.empty() || !running_.load(); });
        if (!running_.load()) break;
        req = std::move(queue_.front());
        queue_.pop_front();
        if (req.epoch != epoch_.load()) continue;  // périmé (interrompu)
        synthesizing_.store(true);
      }

      if (req.text.size() > cfg_.max_text_chars) {
        size_t cut = req.text.find_last_of(".!?", cfg_.max_text_chars);
        req.text.resize(cut == std::string::npos ? cfg_.max_text_chars : cut + 1);
        log(1, "TTS : texte tronqué à " + std::to_string(req.text.size()) + " caractères");
      }

      // Taux de la carte (peut changer après un reopen USB).
      const int dev_rate = out_ ? out_->sample_rate() : (test_rate_ > 0 ? test_rate_ : model_rate_);
      if (!rs || rs->out_rate() != dev_rate || rs->in_rate() != model_rate_)
        rs = std::make_unique<PolyphaseResampler>(model_rate_, dev_rate);
      rs->reset();

      // Attendre la fin d'un éventuel fondu d'interruption précédent.
      for (int i = 0; i < 40 && out_ && out_->flushing(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

      GenCtx ctx{this, req.epoch, rs.get(), {}, {}, true, req.received, 0.0};
      SherpaOnnxGenerationConfig g;
      std::memset(&g, 0, sizeof(g));
      g.sid = sid_.load();
      g.speed = speed_.load();
      g.silence_scale = 0.2f;

      log(0, "🗣️  TTS : \"" + req.text + "\"");
      const auto t0 = std::chrono::steady_clock::now();
      const SherpaOnnxGeneratedAudio* a = SherpaOnnxOfflineTtsGenerateWithConfig(
          tts_, req.text.c_str(), &g, &TalkEngine::on_audio, &ctx);
      const auto t1 = std::chrono::steady_clock::now();
      if (a) SherpaOnnxDestroyOfflineTtsGeneratedAudio(a);

      // Queue du filtre de rééchantillonnage.
      if (ctx.epoch == epoch_.load()) {
        ctx.res.clear();
        rs->flush(ctx.res);
        push_audio(ctx, ctx.res.data(), ctx.res.size());
      }

      double synth_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      double first_ms = 0;
      {
        std::lock_guard<std::mutex> lk(stats_mtx_);
        stats_.synth_ms = synth_ms;
        stats_.audio_ms = ctx.audio_ms;
        first_ms = stats_.first_audio_ms;
      }
      if (ctx.audio_ms > 0)
        log(0, "TTS : synthèse " + std::to_string(static_cast<int>(synth_ms)) + " ms pour " +
                   std::to_string(static_cast<int>(ctx.audio_ms)) + " ms d'audio | 1er son après " +
                   std::to_string(static_cast<int>(first_ms)) + " ms");
      synthesizing_.store(false);
    }
  }

  // Callback sherpa-onnx : appelé une fois par phrase.
  static int32_t on_audio(const float* samples, int32_t n, float /*progress*/, void* arg) {
    auto* ctx = static_cast<GenCtx*>(arg);
    TalkEngine* self = ctx->self;
    if (ctx->epoch != self->epoch_.load() || !self->running_.load()) return 0;
    if (n <= 0) return 1;

    ctx->tmp.assign(samples, samples + n);
    normalize_and_limit(ctx->tmp.data(), ctx->tmp.size(), self->cfg_.normalize,
                        self->cfg_.target_peak, self->cfg_.max_norm_gain,
                        self->gain_.load());
    // Pause inter-phrases (VITS ne la produit pas en mode streaming).
    const size_t pause = static_cast<size_t>(self->cfg_.sentence_pause_ms * 1e-3f *
                                             self->model_rate_);
    ctx->tmp.insert(ctx->tmp.end(), pause, 0.0f);
    ctx->audio_ms += 1000.0 * ctx->tmp.size() / self->model_rate_;

    ctx->res.clear();
    ctx->rs->process(ctx->tmp.data(), ctx->tmp.size(), ctx->res);
    return self->push_audio(*ctx, ctx->res.data(), ctx->res.size()) ? 1 : 0;
  }

  /// Écrit dans le ring avec contre-pression. false = interrompu.
  bool push_audio(GenCtx& ctx, const float* p, size_t n) {
    constexpr size_t kBlock = 2048;
    size_t off = 0;
    while (off < n) {
      if (ctx.epoch != epoch_.load() || !running_.load()) return false;
      size_t w = 0;
      if (!out_) {  // mode test : pas de carte son, sortie vers un puits
        if (sink_) sink_(p + off, n - off);
        w = n - off;
      } else {
        std::lock_guard<std::mutex> wl(write_mtx_);
        if (ctx.epoch != epoch_.load()) return false;
        w = out_->write(p + off, std::min(kBlock, n - off));
      }
      if (w == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // ring plein
        continue;
      }
      if (ctx.first) {
        ctx.first = false;
        std::lock_guard<std::mutex> lk(stats_mtx_);
        stats_.first_audio_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - ctx.received)
                                    .count();
      }
      off += w;
    }
    return true;
  }

  Config cfg_;
  AudioOutput* out_;
  LogFn log_;
  const SherpaOnnxOfflineTts* tts_ = nullptr;
  int model_rate_ = 22050;

  std::atomic<bool> running_{false};
  std::atomic<bool> synthesizing_{false};
  std::atomic<uint64_t> epoch_{0};
  std::atomic<float> gain_{1.0f};
  std::atomic<float> speed_{1.0f};
  std::atomic<int> sid_{0};

  mutable std::mutex q_mtx_;
  std::condition_variable q_cv_;
  std::deque<Request> queue_;
  std::mutex write_mtx_;
  std::thread worker_;

  mutable std::mutex stats_mtx_;
  Stats stats_;
  std::function<void(const float*, size_t)> sink_;
  int test_rate_ = 0;
};

}  // namespace lat
