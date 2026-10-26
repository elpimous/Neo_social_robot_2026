cmake_minimum_required(VERSION 3.8)
project(qbo_faces_tracking)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(PROGRAMS
  qbo_faces_tracking/qbo_faces_tracking_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME qbo_faces_tracking_node
)

install(DIRECTORY qbo_faces_tracking
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  FILES_MATCHING PATTERN "*.engine"
)

install(DIRECTORY face_embeddings/
  DESTINATION share/${PROJECT_NAME}/face_embeddings
  FILES_MATCHING PATTERN "*.npz"
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
