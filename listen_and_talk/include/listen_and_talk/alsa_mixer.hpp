/**
 * @file    alsa_mixer.hpp
 * @brief   Réglage du mixer ALSA d'une carte son (volume + unmute).
 *
 * Équivalent programmatique de :  amixer -c <carte> set <ctl> 100% unmute
 * appliqué à TOUS les contrôles de lecture (ou de capture) de la carte,
 * à chaque démarrage du node → plus besoin d'alsamixer/alsactl manuel.
 *
 * La carte est cherchée par sous-chaîne de son nom ALSA (ex. "USB Audio
 * Device", "C-Media", "Lite") ou par numéro ("1").
 *
 * @author  Vincent FOUCAULT — Octobre 2026
 * @license MIT
 */
#pragma once

#include <alsa/asoundlib.h>

#include <cstdlib>
#include <functional>
#include <string>

namespace lat {

using LogFn = std::function<void(int level, const std::string&)>;  // 0=info 1=warn 2=err

/// Retourne l'index ALSA de la carte (ou -1).
inline int find_alsa_card(const std::string& hint) {
  if (hint.empty()) return -1;
  char* end = nullptr;
  const long num = std::strtol(hint.c_str(), &end, 10);
  if (end && *end == '\0') return static_cast<int>(num);

  int card = -1;
  while (snd_card_next(&card) == 0 && card >= 0) {
    char* name = nullptr;
    char* lname = nullptr;
    bool match = false;
    if (snd_card_get_name(card, &name) == 0 && name) {
      match = std::string(name).find(hint) != std::string::npos;
      free(name);
    }
    if (!match && snd_card_get_longname(card, &lname) == 0 && lname) {
      match = std::string(lname).find(hint) != std::string::npos;
    }
    if (lname) free(lname);
    if (match) return card;
  }
  return -1;
}

/**
 * Règle tous les contrôles de la carte à `percent` % et les active.
 * @param playback  true = contrôles de lecture, false = capture.
 * @return nombre de contrôles réglés (0 si échec).
 */
inline int set_alsa_card_volume(const std::string& card_hint, int percent,
                                bool playback, const LogFn& log) {
  const int card = find_alsa_card(card_hint);
  if (card < 0) {
    if (log) log(1, "ALSA : carte '" + card_hint + "' introuvable — volume non réglé");
    return 0;
  }
  percent = percent < 0 ? 0 : (percent > 100 ? 100 : percent);

  snd_mixer_t* h = nullptr;
  const std::string dev = "hw:" + std::to_string(card);
  if (snd_mixer_open(&h, 0) < 0) return 0;
  if (snd_mixer_attach(h, dev.c_str()) < 0 ||
      snd_mixer_selem_register(h, nullptr, nullptr) < 0 ||
      snd_mixer_load(h) < 0) {
    snd_mixer_close(h);
    if (log) log(1, "ALSA : impossible d'ouvrir le mixer " + dev);
    return 0;
  }

  int count = 0;
  std::string names;
  for (snd_mixer_elem_t* e = snd_mixer_first_elem(h); e; e = snd_mixer_elem_next(e)) {
    if (!snd_mixer_selem_is_active(e)) continue;
    long mn = 0, mx = 0;
    bool done = false;
    if (playback && snd_mixer_selem_has_playback_volume(e)) {
      snd_mixer_selem_get_playback_volume_range(e, &mn, &mx);
      snd_mixer_selem_set_playback_volume_all(e, mn + (mx - mn) * percent / 100);
      done = true;
    }
    if (playback && snd_mixer_selem_has_playback_switch(e)) {
      snd_mixer_selem_set_playback_switch_all(e, 1);
      done = true;
    }
    if (!playback && snd_mixer_selem_has_capture_volume(e)) {
      snd_mixer_selem_get_capture_volume_range(e, &mn, &mx);
      snd_mixer_selem_set_capture_volume_all(e, mn + (mx - mn) * percent / 100);
      done = true;
    }
    if (!playback && snd_mixer_selem_has_capture_switch(e)) {
      snd_mixer_selem_set_capture_switch_all(e, 1);
      done = true;
    }
    if (done) {
      ++count;
      names += std::string(names.empty() ? "" : ", ") + snd_mixer_selem_get_name(e);
    }
  }
  snd_mixer_close(h);

  if (log) {
    if (count)
      log(0, "ALSA " + dev + " : " + std::to_string(percent) + "% " +
                 (playback ? "lecture" : "capture") + " [" + names + "]");
    else
      log(1, "ALSA " + dev + " : aucun contrôle de " +
                 (playback ? std::string("lecture") : std::string("capture")) + " trouvé");
  }
  return count;
}

}  // namespace lat
