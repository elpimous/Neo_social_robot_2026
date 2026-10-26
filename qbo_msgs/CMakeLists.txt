cmake_minimum_required(VERSION 3.8)
project(qbo_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ListenResult.msg"
  "msg/BatteryLevel.msg"
  "msg/LCD.msg"
  "msg/Nose.msg"
  "msg/Mouth.msg"
  "msg/MicReport.msg"
  "msg/FacePosAndDist.msg"
  "srv/Text2Speach.srv"
  "srv/TorqueEnable.srv"
  "srv/SetOdometry.srv"
  "srv/CalibrateIMU.srv"
  "srv/TestLeds.srv"
  "srv/SetMouthAnimation.srv"
  "srv/RecognizeFace.srv"
  "srv/GetName.srv"
  "srv/Train.srv"
  "srv/LearnFaces.srv"
  DEPENDENCIES std_msgs builtin_interfaces sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
