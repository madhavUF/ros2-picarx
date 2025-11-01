cmake_minimum_required(VERSION 3.8)
project(my_first_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/TargetPerson.msg"
  "msg/RobotCommand.msg"
  "msg/CameraCommand.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Install Python package (entry points defined in setup.py)
if(NOT TARGET ament_cmake_python_build_${PROJECT_NAME}_egg)
  ament_python_install_package(${PROJECT_NAME})
endif()

# Install Python scripts
install(PROGRAMS
  my_first_pkg/yolo_detector_node.py
  my_first_pkg/person_tracker_node.py
  my_first_pkg/behavior_controller_node.py
  my_first_pkg/robot_controller_node.py
  my_first_pkg/camera_servo_node.py
  my_first_pkg/multi_sensor_publisher.py
  my_first_pkg/room_scanner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
