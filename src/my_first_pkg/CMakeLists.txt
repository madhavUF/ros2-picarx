cmake_minimum_required(VERSION 3.8)
project(my_first_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  my_first_pkg/drive_node.py
  my_first_pkg/wheel_command_publisher.py
  my_first_pkg/wheel_command_listener.py
  my_first_pkg/wheel_command_listener_test.py
  my_first_pkg/sensor_reader.py
  my_first_pkg/multi_sensor_publisher.py
  my_first_pkg/camera_publisher.py
  my_first_pkg/camera_subscriber.py
  my_first_pkg/autonomous_obstacle_avoidance.py
  my_first_pkg/multi_sensor_subscriber.py
  my_first_pkg/yolo_detector_node.py
  my_first_pkg/person_tracker_node.py
  my_first_pkg/behavior_controller_node.py
  my_first_pkg/robot_controller_node.py
  my_first_pkg/camera_servo_node.py
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
