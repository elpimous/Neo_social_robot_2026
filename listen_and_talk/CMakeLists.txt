# ═══════════════════════════════════════════════════════════════════════════════
#  listen_and_talk — STT + TTS de Néo, deux nodes indépendants
#  Author : Vincent FOUCAULT — Octobre 2026
# ═══════════════════════════════════════════════════════════════════════════════
cmake_minimum_required(VERSION 3.16)
project(listen_and_talk CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# ── Dépendances ROS2 ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(qbo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── Dépendances système ───────────────────────────────────────────────────────
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
pkg_check_modules(ALSA REQUIRED alsa)
pkg_check_modules(SHERPA_ONNX REQUIRED sherpa-onnx)

# ── Vérifications sherpa-onnx ─────────────────────────────────────────────────
# >= 2.0 : espeak-ng intégré SUPPRIMÉ → la voix "axel" (VITS/Piper) ne
# fonctionne plus sans phonémiseur externe. On bloque la compilation plutôt
# que de produire un robot muet.
if(SHERPA_ONNX_VERSION VERSION_GREATER_EQUAL "2.0.0")
  message(FATAL_ERROR
    "sherpa-onnx ${SHERPA_ONNX_VERSION} détecté : à partir de 2.0 le phonémiseur "
    "espeak intégré a été retiré, la voix axel ne fonctionnerait plus. "
    "Rester sur la branche 1.x (ex. 1.13.8).")
endif()

find_file(SHERPA_C_API_H sherpa-onnx/c-api/c-api.h
          HINTS ${SHERPA_ONNX_INCLUDE_DIRS} /usr/local/include /usr/include)
if(NOT SHERPA_C_API_H)
  message(FATAL_ERROR "En-tête sherpa-onnx/c-api/c-api.h introuvable")
endif()
file(READ "${SHERPA_C_API_H}" _SHERPA_API)
string(FIND "${_SHERPA_API}" "SherpaOnnxOfflineTtsGenerateWithConfig" _has_gen)
if(_has_gen EQUAL -1)
  message(FATAL_ERROR "sherpa-onnx trop ancien (GenerateWithConfig absent) : mettre à jour en 1.12+/1.13.x")
endif()
string(FIND "${_SHERPA_API}" "SherpaOnnxTenVadModelConfig ten_vad" _has_ten)
string(FIND "${_SHERPA_API}" "ys_log_probs" _has_lp)
set(LAT_DEFS "")
if(NOT _has_ten EQUAL -1)
  list(APPEND LAT_DEFS LAT_HAS_TEN_VAD)
endif()
if(NOT _has_lp EQUAL -1)
  list(APPEND LAT_DEFS LAT_HAS_YS_LOG_PROBS)
endif()
message(STATUS "sherpa-onnx ${SHERPA_ONNX_VERSION} — options : ${LAT_DEFS}")

set(LAT_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SHERPA_ONNX_INCLUDE_DIRS}
  ${PORTAUDIO_INCLUDE_DIRS}
  ${ALSA_INCLUDE_DIRS})
set(LAT_LIBS
  ${SHERPA_ONNX_LINK_LIBRARIES}
  ${PORTAUDIO_LINK_LIBRARIES}
  ${ALSA_LINK_LIBRARIES}
  Threads::Threads)

# ── talk_node ─────────────────────────────────────────────────────────────────
add_executable(talk_node src/talk_node.cpp)
target_include_directories(talk_node PRIVATE ${LAT_INCLUDES})
target_compile_definitions(talk_node PRIVATE ${LAT_DEFS})
target_link_libraries(talk_node ${LAT_LIBS})
ament_target_dependencies(talk_node rclcpp std_msgs ament_index_cpp)

# ── listen_node ───────────────────────────────────────────────────────────────
add_executable(listen_node src/listen_node.cpp)
target_include_directories(listen_node PRIVATE ${LAT_INCLUDES})
target_compile_definitions(listen_node PRIVATE ${LAT_DEFS})
target_link_libraries(listen_node ${LAT_LIBS})
ament_target_dependencies(listen_node rclcpp std_msgs qbo_msgs ament_index_cpp)

# ── Banc de test hors ROS (optionnel) : colcon build --cmake-args -DLAT_BENCH=ON
option(LAT_BENCH "Construire lat_bench (test TTS→STT sans ROS)" OFF)
if(LAT_BENCH)
  add_executable(lat_bench test/lat_bench.cpp)
  target_include_directories(lat_bench PRIVATE ${LAT_INCLUDES})
  target_compile_definitions(lat_bench PRIVATE ${LAT_DEFS})
  target_link_libraries(lat_bench ${LAT_LIBS})
  install(TARGETS lat_bench DESTINATION lib/${PROJECT_NAME})
endif()

# ── Installation ──────────────────────────────────────────────────────────────
install(TARGETS talk_node listen_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# models/ n'est pas versionné (cf. .gitignore racine) : créé par get_models.sh
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/models)
  install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models
          PATTERN "README.md" EXCLUDE)
else()
  message(WARNING "listen_and_talk/models absent : lancer scripts/get_models.sh puis recompiler")
endif()
install(PROGRAMS scripts/get_models.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
