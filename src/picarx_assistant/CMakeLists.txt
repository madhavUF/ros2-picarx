cmake_minimum_required(VERSION 3.8)
project(picarx_assistant)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioChunk.msg"
  "msg/AssistantState.msg"
  "msg/VoiceCommand.msg"
  "srv/DescribeScene.srv"
  "srv/VLMQuery.srv"
  "srv/ExecuteCommand.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  picarx_assistant/voice_input_node.py
  picarx_assistant/stt_node.py
  picarx_assistant/tts_node.py
  picarx_assistant/conversation_node.py
  picarx_assistant/ros2_mcp_server.py
  picarx_assistant/vlm_node.py
  picarx_assistant/assistant_bridge.py
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
