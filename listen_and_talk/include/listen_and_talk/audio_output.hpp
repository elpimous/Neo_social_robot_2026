/**
 * @file    audio_output.hpp
 * @brief   Sortie audio PortAudio en mode callback, flux toujours ouvert.
 *
 * Différences clés avec l'ancien code (Pa_WriteStream bloquant +
 * Start/Stop à chaque phrase) :
 *  - Le flux est ouvert et démarré UNE fois : plus de négociation USB,
 *    plus de "clic" de démarrage, latence de départ ≈ 0.
 *  - Le callback temps réel lit un ring buffer lock-free : aucun mutex,
 *    aucune allocation, il ne peut jamais bloquer le reste du node.
 *  - Interruption = fondu de sortie (~12 ms) puis vidage : coupure nette
 *    mais sans "pop".
 *  - Détection de perte du périphérique (USB débranché/rebranché) :
 *    reopen() complet (Pa_Terminate/Pa_Initialize = re-scan des cartes).
 *  - Sample rate négocié : essai du taux demandé puis 44100 / 48000.
 *
 * Le producteur écrit du MONO au taux du périphérique ; le callback
 * duplique sur tous les canaux.
 *
 * @author  Vincent FOUCAULT — Octobre 2026
 * @license MIT
 */
#pragma once

#include <portaudio.h>

#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "listen_and_talk/alsa_mixer.hpp"  // LogFn
#include "listen_and_talk/spsc_ring.hpp"

namespace lat {

class AudioOutput {
 public:
  struct Config {
    std::string device_hint = "USB Audio Device";  // sous-chaîne du nom PortAudio
    int sample_rate = 44100;                        // taux souhaité
    int channels = 2;                               // canaux souhaités
    double latency_s = 0.05;                        // latence suggérée
    float fade_ms = 12.0f;                          // fondu d'interruption
    double ring_seconds = 30.0;                     // tampon max
  };

  AudioOutput(Config cfg, LogFn log) : cfg_(std::move(cfg)), log_(std::move(log)) {
    // Ring alloué une seule fois, dimensionné pour le taux max possible :
    // jamais ré-alloué ensuite (le producteur peut écrire pendant un reopen).
    const int max_rate = std::max(48000, cfg_.sample_rate);
    ring_ = std::make_unique<SpscRing<float>>(static_cast<size_t>(cfg_.ring_seconds * max_rate));
    scratch_.assign(16384, 0.0f);
  }
  ~AudioOutput() { close(true); }

  AudioOutput(const AudioOutput&) = delete;
  AudioOutput& operator=(const AudioOutput&) = delete;

  /// Ouvre et démarre le flux. Retourne false si aucun format accepté.
  bool open() {
    if (!pa_initialized_) {
      if (Pa_Initialize() != paNoError) {
        log(2, "Pa_Initialize a échoué");
        return false;
      }
      pa_initialized_ = true;
    }

    const PaDeviceIndex dev = find_device(cfg_.device_hint);
    if (dev == paNoDevice) {
      log(2, "Aucun périphérique de sortie trouvé (hint='" + cfg_.device_hint + "')");
      return false;
    }
    const PaDeviceInfo* info = Pa_GetDeviceInfo(dev);

    std::vector<int> rates{cfg_.sample_rate, 44100, 48000};
    std::vector<int> chans{cfg_.channels, 2, 1};
    for (int ch : chans) {
      if (ch < 1 || ch > info->maxOutputChannels) continue;
      for (int sr : rates) {
        PaStreamParameters p{};
        p.device = dev;
        p.channelCount = ch;
        p.sampleFormat = paFloat32;
        p.suggestedLatency = std::max(cfg_.latency_s, info->defaultLowOutputLatency);
        if (Pa_IsFormatSupported(nullptr, &p, sr) != paFormatIsSupported) continue;

        PaStream* s = nullptr;
        if (Pa_OpenStream(&s, nullptr, &p, sr, paFramesPerBufferUnspecified,
                          paClipOff | paDitherOff, &AudioOutput::pa_callback,
                          this) != paNoError)
          continue;

        ring_->drop_all();  // flux arrêté : ce thread fait office de consommateur

        stream_ = s;
        rate_.store(sr);
        channels_ = ch;
        fade_frames_ = std::max(1, static_cast<int>(cfg_.fade_ms * 1e-3f * sr));
        flush_req_.store(false);
        fading_ = false;
        error_.store(false);

        if (Pa_StartStream(stream_) != paNoError) {
          Pa_CloseStream(stream_);
          stream_ = nullptr;
          continue;
        }
        const PaStreamInfo* si = Pa_GetStreamInfo(stream_);
        log(0, std::string("Sortie audio : [") + std::to_string(dev) + "] " + info->name +
                   " | " + std::to_string(sr) + " Hz | " + std::to_string(ch) +
                   " canaux | latence " +
                   std::to_string(static_cast<int>((si ? si->outputLatency : 0) * 1000)) + " ms");
        return true;
      }
    }
    log(2, std::string("Aucun format accepté par ") + info->name);
    return false;
  }

  /// Ferme le flux. `terminate` = libère aussi PortAudio.
  void close(bool terminate) {
    if (stream_) {
      Pa_AbortStream(stream_);
      Pa_CloseStream(stream_);
      stream_ = nullptr;
    }
    if (terminate && pa_initialized_) {
      Pa_Terminate();
      pa_initialized_ = false;
    }
  }

  /// Ferme tout et ré-ouvre (re-scan des périphériques USB).
  bool reopen() {
    close(true);
    return open();
  }

  /// Flux actif et sans erreur signalée.
  bool healthy() const {
    return stream_ && !error_.load() && Pa_IsStreamActive(stream_) == 1;
  }

  int sample_rate() const { return rate_.load(); }

  /// Producteur : écrit du mono (non bloquant). Retourne le nombre écrit.
  size_t write(const float* mono, size_t n) { return ring_ ? ring_->write(mono, n) : 0; }
  size_t free_space() const { return ring_ ? ring_->free_space() : 0; }
  size_t queued() const { return ring_ ? ring_->size() : 0; }

  /// Demande un fondu + vidage (thread quelconque).
  void request_flush() { flush_req_.store(true, std::memory_order_release); }
  bool flushing() const { return flush_req_.load(std::memory_order_acquire); }

  /// Vrai si le dernier callback a émis de l'audio non vide.
  bool playing() const { return playing_.load(std::memory_order_relaxed); }

 private:
  void log(int lvl, const std::string& m) const {
    if (log_) log_(lvl, m);
  }

  static PaDeviceIndex find_device(const std::string& hint) {
    const int n = Pa_GetDeviceCount();
    if (!hint.empty()) {
      for (int i = 0; i < n; ++i) {
        const PaDeviceInfo* d = Pa_GetDeviceInfo(i);
        if (d && d->maxOutputChannels > 0 && std::string(d->name).find(hint) != std::string::npos)
          return i;
      }
    }
    return Pa_GetDefaultOutputDevice();
  }

  static int pa_callback(const void*, void* output, unsigned long frames,
                         const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags flags,
                         void* user) {
    auto* self = static_cast<AudioOutput*>(user);
    self->render(static_cast<float*>(output), frames);
    (void)flags;
    return paContinue;
  }

  // Temps réel : pas d'allocation, pas de verrou.
  void render(float* out, unsigned long frames) {
    unsigned long done = 0;
    bool any = false;
    while (done < frames) {
      const size_t want = std::min<size_t>(frames - done, scratch_.size());
      size_t got = 0;

      if (flush_req_.load(std::memory_order_acquire)) {
        if (!fading_) {
          fading_ = true;
          fade_left_ = fade_frames_;
        }
        const size_t fw = std::min<size_t>(want, static_cast<size_t>(fade_left_));
        got = ring_->read(scratch_.data(), fw);
        for (size_t i = 0; i < got; ++i) {
          scratch_[i] *= static_cast<float>(fade_left_ - static_cast<int>(i)) / fade_frames_;
        }
        fade_left_ -= static_cast<int>(got);
        if (fade_left_ <= 0 || got < fw) {
          ring_->drop_all();
          fading_ = false;
          flush_req_.store(false, std::memory_order_release);
        }
      } else {
        got = ring_->read(scratch_.data(), want);
      }

      for (size_t i = got; i < want; ++i) scratch_[i] = 0.0f;
      if (got) any = true;

      float* o = out + done * channels_;
      for (size_t i = 0; i < want; ++i)
        for (int c = 0; c < channels_; ++c) *o++ = scratch_[i];
      done += want;
      if (got < want && !flush_req_.load(std::memory_order_relaxed)) {
        // Plus rien à lire : silence pour le reste du buffer.
        float* z = out + done * channels_;
        std::fill(z, z + (frames - done) * channels_, 0.0f);
        break;
      }
    }
    playing_.store(any, std::memory_order_relaxed);
  }

  Config cfg_;
  LogFn log_;
  bool pa_initialized_ = false;
  PaStream* stream_ = nullptr;
  std::atomic<int> rate_{0};
  int channels_ = 2;
  std::unique_ptr<SpscRing<float>> ring_;
  std::vector<float> scratch_;
  std::atomic<bool> flush_req_{false};
  std::atomic<bool> error_{false};
  std::atomic<bool> playing_{false};
  bool fading_ = false;
  int fade_left_ = 0;
  int fade_frames_ = 1;
};

}  // namespace lat
