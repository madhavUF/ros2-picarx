cmake_minimum_required(VERSION 3.0.2)
project(yolo_detector_ros1)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Detection.msg
  DetectionArray.msg
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    cv_bridge
    message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/yolo_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install model files (optional)
install(FILES
  yolov8n.pt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
