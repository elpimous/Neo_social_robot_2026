cmake_minimum_required(VERSION 3.5)
project(listen_and_talk)

# ------------------ Packages ROS2 ------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ------------------ TensorRT / RNNoise ------------------
find_library(TENSORRT_LIB nvinfer REQUIRED)
find_library(TENSORRT_PLUGIN_LIB nvinfer_plugin REQUIRED)
find_library(RNNOISE_LIB rnnoise REQUIRED)

# ------------------ Sherpa / PortAudio ------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(SHERPA_ONNX REQUIRED sherpa-onnx)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

# ------------------ CUDA ------------------
find_package(CUDAToolkit REQUIRED)

# ------------------ Includes ------------------
include_directories(
  include
  ${SHERPA_ONNX_INCLUDE_DIRS}
  ${PORTAUDIO_INCLUDE_DIRS}
)

# ------------------ Executable ------------------
add_executable(listen_and_talk_node src/listen_and_talk.cpp)

ament_target_dependencies(listen_and_talk_node rclcpp std_msgs)

# ------------------ LINK (IMPORTANT) ------------------
target_link_libraries(listen_and_talk_node
  ${SHERPA_ONNX_LIBRARIES}
  ${PORTAUDIO_LIBRARIES}

  CUDA::cudart              # ✅ FIX CRUCIAL

  ${TENSORRT_LIB}
  ${TENSORRT_PLUGIN_LIB}
  ${RNNOISE_LIB}

  yaml-cpp
  pthread
)

# ------------------ Install ------------------
install(TARGETS listen_and_talk_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

ament_package()