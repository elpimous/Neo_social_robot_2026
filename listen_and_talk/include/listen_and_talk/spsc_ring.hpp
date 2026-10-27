/**
 * @file    spsc_ring.hpp
 * @brief   Ring buffer lock-free mono-producteur / mono-consommateur.
 *
 * Utilisé entre un thread "normal" et un callback audio temps réel
 * (PortAudio) : aucune allocation, aucun mutex, aucune attente côté
 * callback. Capacité arrondie à la puissance de 2 supérieure.
 *
 * @author  Vincent FOUCAULT — Octobre 2026
 * @license MIT
 */
#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <vector>

namespace lat {

template <typename T>
class SpscRing {
 public:
  explicit SpscRing(size_t min_capacity) {
    size_t cap = 1;
    while (cap < min_capacity) cap <<= 1;
    buf_.resize(cap);
    mask_ = cap - 1;
  }

  size_t capacity() const { return buf_.size(); }

  /// Nombre d'éléments lisibles (approximation sûre côté consommateur).
  size_t size() const {
    return head_.load(std::memory_order_acquire) -
           tail_.load(std::memory_order_acquire);
  }

  size_t free_space() const { return capacity() - size(); }

  /// Producteur : écrit jusqu'à n éléments, retourne le nombre écrit.
  size_t write(const T* src, size_t n) {
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t tail = tail_.load(std::memory_order_acquire);
    n = std::min(n, capacity() - (head - tail));
    for (size_t i = 0; i < n; ++i) buf_[(head + i) & mask_] = src[i];
    head_.store(head + n, std::memory_order_release);
    return n;
  }

  /// Consommateur : lit jusqu'à n éléments, retourne le nombre lu.
  size_t read(T* dst, size_t n) {
    const size_t tail = tail_.load(std::memory_order_relaxed);
    const size_t head = head_.load(std::memory_order_acquire);
    n = std::min(n, head - tail);
    for (size_t i = 0; i < n; ++i) dst[i] = buf_[(tail + i) & mask_];
    tail_.store(tail + n, std::memory_order_release);
    return n;
  }

  /// Consommateur : jette tout le contenu actuellement lisible.
  void drop_all() {
    tail_.store(head_.load(std::memory_order_acquire),
                std::memory_order_release);
  }

 private:
  std::vector<T> buf_;
  size_t mask_ = 0;
  alignas(64) std::atomic<size_t> head_{0};  // écrit par le producteur
  alignas(64) std::atomic<size_t> tail_{0};  // écrit par le consommateur
};

}  // namespace lat
