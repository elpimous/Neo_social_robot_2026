/**
 * @file    audio_dsp.hpp
 * @brief   Outils DSP légers : rééchantillonneur polyphasé (rapport
 *          rationnel quelconque), normalisation crête, limiteur doux, RMS.
 *
 * Le rééchantillonneur remplace l'ancien "upsample ×2 par répétition"
 * (bloqueur d'ordre 0) qui générait des images spectrales audibles
 * (son métallique/sifflant au-dessus de 11 kHz). Ici : filtre sinc
 * fenêtré Kaiser, phase linéaire, ~16 coefficients par phase,
 * état conservé entre les appels (pas de clic entre deux phrases).
 *
 * @author  Vincent FOUCAULT — Octobre 2026
 * @license MIT
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lat {

// ─────────────────────────────────────────────────────────────────────────────
//  Rééchantillonneur polyphasé L/M, flux continu, mono float.
// ─────────────────────────────────────────────────────────────────────────────
class PolyphaseResampler {
 public:
  PolyphaseResampler(int in_rate, int out_rate, int taps_per_phase = 16)
      : in_rate_(in_rate), out_rate_(out_rate), taps_(taps_per_phase) {
    const int g = std::gcd(in_rate, out_rate);
    L_ = out_rate / g;
    M_ = in_rate / g;
    passthrough_ = (L_ == 1 && M_ == 1);
    if (passthrough_) return;

    // Filtre prototype à la fréquence L*in_rate.
    const int N = L_ * taps_;
    const double fc = 0.5 / std::max(L_, M_) * 0.92;  // marge de transition
    const double beta = 8.0;                            // Kaiser ~ -80 dB
    const double i0b = bessel_i0(beta);
    std::vector<double> proto(N);
    const double c = (N - 1) / 2.0;
    for (int n = 0; n < N; ++n) {
      const double x = n - c;
      const double sinc =
          (x == 0.0) ? 2.0 * fc : std::sin(2.0 * M_PI * fc * x) / (M_PI * x);
      const double r = 2.0 * n / (N - 1) - 1.0;
      const double w = bessel_i0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / i0b;
      proto[n] = sinc * w * L_;
    }
    // Réorganisation polyphasée : h_[p][j] = proto[p + j*L]
    h_.assign(static_cast<size_t>(L_) * taps_, 0.0f);
    for (int p = 0; p < L_; ++p) {
      double sum = 0.0;
      for (int j = 0; j < taps_; ++j) sum += proto[p + j * L_];
      for (int j = 0; j < taps_; ++j)
        h_[p * taps_ + j] = static_cast<float>(proto[p + j * L_] / (sum != 0.0 ? sum : 1.0));
    }
    reset();
  }

  int in_rate() const { return in_rate_; }
  int out_rate() const { return out_rate_; }

  void reset() {
    x_.assign(static_cast<size_t>(taps_ - 1), 0.0f);
    pos_ = taps_ - 1;
    phase_ = 0;
  }

  /// Ajoute n échantillons d'entrée et AJOUTE les sorties produites à `out`.
  void process(const float* in, size_t n, std::vector<float>& out) {
    if (passthrough_) {
      out.insert(out.end(), in, in + n);
      return;
    }
    x_.insert(x_.end(), in, in + n);
    out.reserve(out.size() + (n * L_) / M_ + 2);
    while (pos_ < static_cast<int64_t>(x_.size())) {
      const float* h = &h_[static_cast<size_t>(phase_) * taps_];
      const float* xp = &x_[static_cast<size_t>(pos_)];
      float acc = 0.0f;
      for (int j = 0; j < taps_; ++j) acc += h[j] * xp[-j];
      out.push_back(acc);
      phase_ += M_;
      pos_ += phase_ / L_;
      phase_ %= L_;
    }
    // On ne garde que l'historique nécessaire (taps-1 échantillons).
    const int64_t keep_from = pos_ - (taps_ - 1);
    if (keep_from > 0) {
      const int64_t drop = std::min<int64_t>(keep_from, static_cast<int64_t>(x_.size()));
      x_.erase(x_.begin(), x_.begin() + drop);
      pos_ -= drop;
    }
  }

  /// Vide la queue du filtre (fin d'énoncé).
  void flush(std::vector<float>& out) {
    if (passthrough_) return;
    std::vector<float> zeros(static_cast<size_t>(taps_ / 2 + 1), 0.0f);
    process(zeros.data(), zeros.size(), out);
  }

 private:
  static double bessel_i0(double x) {
    double sum = 1.0, term = 1.0;
    const double q = x * x / 4.0;
    for (int k = 1; k < 50; ++k) {
      term *= q / (static_cast<double>(k) * k);
      sum += term;
      if (term < 1e-12 * sum) break;
    }
    return sum;
  }

  int in_rate_, out_rate_, taps_;
  int L_ = 1, M_ = 1;
  bool passthrough_ = true;
  std::vector<float> h_;
  std::vector<float> x_;
  int64_t pos_ = 0;
  int phase_ = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Utilitaires niveau
// ─────────────────────────────────────────────────────────────────────────────
inline float rms(const float* x, size_t n) {
  if (n == 0) return 0.0f;
  double s = 0.0;
  for (size_t i = 0; i < n; ++i) s += static_cast<double>(x[i]) * x[i];
  return static_cast<float>(std::sqrt(s / n));
}

inline float peak(const float* x, size_t n) {
  float p = 0.0f;
  for (size_t i = 0; i < n; ++i) p = std::max(p, std::fabs(x[i]));
  return p;
}

/// Limiteur doux : linéaire sous `knee`, compression tanh au-dessus,
/// jamais au-delà de 1.0. Remplace le clamp brutal (écrêtage = grésillement).
inline float soft_limit(float x, float knee = 0.85f) {
  const float a = std::fabs(x);
  if (a <= knee) return x;
  const float room = 1.0f - knee;
  const float y = knee + room * std::tanh((a - knee) / room);
  return std::copysign(y, x);
}

/// Normalise un bloc vers `target_peak` (gain plafonné), applique un gain
/// utilisateur puis le limiteur doux.
inline void normalize_and_limit(float* x, size_t n, bool normalize,
                                float target_peak, float max_norm_gain,
                                float user_gain) {
  float g = user_gain;
  if (normalize) {
    const float p = peak(x, n);
    if (p > 1e-4f) g *= std::min(max_norm_gain, target_peak / p);
  }
  for (size_t i = 0; i < n; ++i) x[i] = soft_limit(x[i] * g);
}

}  // namespace lat
