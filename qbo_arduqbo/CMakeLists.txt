cmake_minimum_required(VERSION 3.8)
project(qbo_arduqbo)

# --------------------
# Options de compilation
# --------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------
# Trouver les dépendances nécessaires
# --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_aggregator REQUIRED)
# find_package(rqt_robot_monitor REQUIRED)
find_package(qbo_msgs REQUIRED)

include_directories(include)

# ====================================
# LIBRAIRIES
# ====================================

# --- Librairie série
add_library(qbo_serial
  src/cereal_port/CerealPort.cpp
  src/drivers/arduqbo_instructions.cpp
  src/drivers/qboduino_driver.cpp
)
target_link_libraries(qbo_serial pthread)
ament_target_dependencies(qbo_serial rclcpp)

# --- Contrôleur Dynamixel
add_library(dynamixel_controller_lib src/controllers/dynamixel_controller.cpp)
ament_target_dependencies(dynamixel_controller_lib
  rclcpp sensor_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
  dynamixel_workbench_toolbox diagnostic_updater qbo_msgs
)

# --- Contrôleurs Qbo
add_library(base_controller src/controllers/base_controller.cpp)
target_link_libraries(base_controller qbo_serial)
ament_target_dependencies(base_controller
  rclcpp tf2 tf2_ros tf2_geometry_msgs
  geometry_msgs nav_msgs diagnostic_updater
  qbo_msgs std_srvs
)

add_library(imu_controller src/controllers/imu_controller.cpp)
target_link_libraries(imu_controller qbo_serial)
ament_target_dependencies(imu_controller
  rclcpp sensor_msgs std_srvs
  diagnostic_updater qbo_msgs
)

add_library(lcd_controller src/controllers/lcd_controller.cpp)
target_link_libraries(lcd_controller qbo_serial)
ament_target_dependencies(lcd_controller
  rclcpp sensor_msgs std_srvs
  diagnostic_updater qbo_msgs
)

add_library(nose_controller src/controllers/nose_controller.cpp)
target_link_libraries(nose_controller qbo_serial)
ament_target_dependencies(nose_controller rclcpp diagnostic_updater qbo_msgs)

add_library(mouth_controller src/controllers/mouth_controller.cpp)
target_link_libraries(mouth_controller qbo_serial)
ament_target_dependencies(mouth_controller rclcpp qbo_msgs diagnostic_updater)

add_library(audio_controller src/controllers/audio_controller.cpp)
target_link_libraries(audio_controller qbo_serial)
ament_target_dependencies(audio_controller rclcpp qbo_msgs)

add_library(sensor_controller src/controllers/sensor_controller.cpp)
target_link_libraries(sensor_controller qbo_serial)
ament_target_dependencies(sensor_controller
  rclcpp sensor_msgs
  diagnostic_updater
)


# ====================================
# EXECUTABLES
# ====================================

# --- Noeud Dynamixel
add_executable(qbo_dynamixel src/qbo_dynamixel.cpp)
target_link_libraries(qbo_dynamixel dynamixel_controller_lib)
ament_target_dependencies(qbo_dynamixel
  rclcpp sensor_msgs std_msgs
  tf2 tf2_ros
  dynamixel_workbench_toolbox diagnostic_updater qbo_msgs
)

# --- Noeud Qbo Arduqbo
add_executable(qbo_arduqbo
  src/qbo_arduqbo.cpp
  src/drivers/i2c_bus_driver.cpp
  src/controllers/battery_controller.cpp
  # src/controllers/imu_controller.cpp
)
target_include_directories(qbo_arduqbo PUBLIC include)
target_link_libraries(qbo_arduqbo
  qbo_serial
  base_controller
  imu_controller
  lcd_controller
  nose_controller
  mouth_controller
  audio_controller
  sensor_controller
)
ament_target_dependencies(qbo_arduqbo
  rclcpp std_msgs diagnostic_msgs diagnostic_updater qbo_msgs
)

# ====================================
# INSTALLATION
# ====================================

install(TARGETS
  qbo_serial
  qbo_dynamixel
  qbo_arduqbo
  dynamixel_controller_lib
  base_controller
  imu_controller
  lcd_controller
  nose_controller
  mouth_controller
  audio_controller
  sensor_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ====================================
# TESTS & FORMAT
# ====================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
