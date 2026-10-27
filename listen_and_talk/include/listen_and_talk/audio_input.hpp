/**
 * @file    audio_input.hpp
 * @brief   Capture micro PortAudio en mode callback → ring lock-free 16 kHz.
 *
 * Le callback temps réel ne fait que : conversion int16→float, sélection
 * du canal, (ré)échantillonnage si la carte n'accepte pas 16 kHz, écriture
 * dans le ring. L'inférence (VAD/ASR) ne peut donc JAMAIS provoquer de
 * perte d'échantillons micro (défaut de l'ancien code : Pa_ReadStream puis
 * transcription dans la même boucle → overflow pendant l'ASR).
 *
 * @author  Vincent FOUCAULT — Octobre 2026
 * @license MIT
 */
#pragma once

#include <portaudio.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "listen_and_talk/alsa_mixer.hpp"  // LogFn
#include "listen_and_talk/audio_dsp.hpp"
#include "listen_and_talk/spsc_ring.hpp"

namespace lat {

class AudioInput {
 public:
  struct Config {
    std::string device_hint = "Lite";  // sous-chaîne du nom PortAudio (ReSpeaker Lite)
    int channels = 1;                  // canaux demandés (repli auto 2 → canal choisi)
    int channel_index = 0;             // canal utilisé si multi-canaux
    int target_rate = 16000;           // taux livré au pipeline
    double latency_s = 0.03;
    double ring_seconds = 20.0;
  };

  AudioInput(Config cfg, LogFn log) : cfg_(std::move(cfg)), log_(std::move(log)) {
    ring_ = std::make_unique<SpscRing<float>>(
        static_cast<size_t>(cfg_.ring_seconds * cfg_.target_rate));
    mono_.reserve(8192);
    res_.reserve(8192);
  }
  ~AudioInput() { close(true); }

  AudioInput(const AudioInput&) = delete;
  AudioInput& operator=(const AudioInput&) = delete;

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
      log(2, "Micro introuvable (hint='" + cfg_.device_hint + "')");
      return false;
    }
    const PaDeviceInfo* info = Pa_GetDeviceInfo(dev);

    std::vector<int> rates{cfg_.target_rate, 48000, 44100, 32000};
    std::vector<int> chans{cfg_.channels, 1, 2};
    for (int ch : chans) {
      if (ch < 1 || ch > info->maxInputChannels) continue;
      for (int sr : rates) {
        PaStreamParameters p{};
        p.device = dev;
        p.channelCount = ch;
        p.sampleFormat = paInt16;
        p.suggestedLatency = std::max(cfg_.latency_s, info->defaultLowInputLatency);
        if (Pa_IsFormatSupported(&p, nullptr, sr) != paFormatIsSupported) continue;
        PaStream* s = nullptr;
        if (Pa_OpenStream(&s, &p, nullptr, sr, paFramesPerBufferUnspecified, paClipOff,
                          &AudioInput::pa_callback, this) != paNoError)
          continue;

        channels_ = ch;
        chan_sel_ = std::min(std::max(0, cfg_.channel_index), ch - 1);
        rs_ = std::make_unique<PolyphaseResampler>(sr, cfg_.target_rate);
        stream_ = s;
        overflows_.store(0);
        if (Pa_StartStream(stream_) != paNoError) {
          Pa_CloseStream(stream_);
          stream_ = nullptr;
          continue;
        }
        log(0, std::string("Micro : [") + std::to_string(dev) + "] " + info->name + " | " +
                   std::to_string(sr) + " Hz | " + std::to_string(ch) + " canal(aux), canal " +
                   std::to_string(chan_sel_) + (sr != cfg_.target_rate ? " | rééchantillonné → 16 kHz" : ""));
        return true;
      }
    }
    log(2, std::string("Aucun format micro accepté par ") + info->name);
    return false;
  }

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

  bool reopen() {
    close(true);
    return open();
  }

  bool healthy() const { return stream_ && Pa_IsStreamActive(stream_) == 1; }

  /// Consommateur : lit jusqu'à n échantillons 16 kHz.
  size_t read(float* dst, size_t n) { return ring_->read(dst, n); }
  size_t available() const { return ring_->size(); }

  /// Nombre d'échantillons perdus (ring plein ou overflow matériel).
  uint64_t overflows() const { return overflows_.load(); }

 private:
  void log(int lvl, const std::string& m) const {
    if (log_) log_(lvl, m);
  }

  static PaDeviceIndex find_device(const std::string& hint) {
    const int n = Pa_GetDeviceCount();
    if (!hint.empty()) {
      for (int i = 0; i < n; ++i) {
        const PaDeviceInfo* d = Pa_GetDeviceInfo(i);
        if (d && d->maxInputChannels > 0 && std::string(d->name).find(hint) != std::string::npos)
          return i;
      }
    }
    return Pa_GetDefaultInputDevice();
  }

  static int pa_callback(const void* input, void*, unsigned long frames,
                         const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags flags,
                         void* user) {
    auto* self = static_cast<AudioInput*>(user);
    if (flags & paInputOverflow) self->overflows_.fetch_add(frames);
    if (input) self->capture(static_cast<const int16_t*>(input), frames);
    return paContinue;
  }

  // Thread audio : vecteurs pré-réservés (pas de réallocation en régime normal).
  void capture(const int16_t* in, unsigned long frames) {
    mono_.resize(frames);
    for (unsigned long i = 0; i < frames; ++i)
      mono_[i] = in[i * channels_ + chan_sel_] * (1.0f / 32768.0f);
    res_.clear();
    rs_->process(mono_.data(), mono_.size(), res_);
    const size_t w = ring_->write(res_.data(), res_.size());
    if (w < res_.size()) overflows_.fetch_add(res_.size() - w);
  }

  Config cfg_;
  LogFn log_;
  bool pa_initialized_ = false;
  PaStream* stream_ = nullptr;
  int channels_ = 1;
  int chan_sel_ = 0;
  std::unique_ptr<PolyphaseResampler> rs_;
  std::unique_ptr<SpscRing<float>> ring_;
  std::vector<float> mono_, res_;
  std::atomic<uint64_t> overflows_{0};
};

}  // namespace lat
