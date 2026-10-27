/**
 * @file    ros_common.hpp
 * @brief   Petits utilitaires communs aux nodes listen / talk.
 * @author  Vincent FOUCAULT — Octobre 2026
 * @license MIT
 */
#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <fstream>
#include <string>

#include "listen_and_talk/alsa_mixer.hpp"

namespace lat {

/// Résout "pkg://<paquet>/<chemin>" (dossier share du paquet) et "~/".
inline std::string resolve_path(const std::string& p) {
  if (p.rfind("pkg://", 0) == 0) {
    const std::string rest = p.substr(6);
    const auto slash = rest.find('/');
    const std::string pkg = rest.substr(0, slash);
    const std::string rel = slash == std::string::npos ? "" : rest.substr(slash + 1);
    try {
      return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
    } catch (const std::exception&) {
      return p;  // paquet introuvable : on laisse tel quel, l'erreur sera explicite
    }
  }
  if (p.rfind("~/", 0) == 0) {
    const char* home = std::getenv("HOME");
    if (home) return std::string(home) + p.substr(1);
  }
  return p;
}

inline bool file_exists(const std::string& p) {
  std::ifstream f(p);
  return f.good();
}

/// Adaptateur LogFn → logger ROS du node.
inline LogFn ros_logger(const rclcpp::Logger& lg) {
  return [lg](int lvl, const std::string& m) {
    if (lvl >= 2)
      RCLCPP_ERROR(lg, "%s", m.c_str());
    else if (lvl == 1)
      RCLCPP_WARN(lg, "%s", m.c_str());
    else
      RCLCPP_INFO(lg, "%s", m.c_str());
  };
}

}  // namespace lat
