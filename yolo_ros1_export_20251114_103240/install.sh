#!/bin/bash
# Installation script for ROS1 YOLO Detector Package

echo "========================================="
echo "Installing YOLO ROS1 Package"
echo "========================================="

# Check if ROS1 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS1 is not sourced!"
    echo "Please source your ROS setup first:"
    echo "  source /opt/ros/noetic/setup.bash  (for ROS Noetic)"
    echo "  source /opt/ros/melodic/setup.bash (for ROS Melodic)"
    exit 1
fi

echo "Detected ROS distribution: $ROS_DISTRO"

# 1. Install Python dependencies
echo ""
echo "Step 1: Installing Python dependencies..."
pip3 install -r requirements.txt

# 2. Check/create catkin workspace
if [ ! -d ~/catkin_ws/src ]; then
    echo ""
    echo "Creating catkin workspace at ~/catkin_ws..."
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws
    catkin_make
else
    echo ""
    echo "Using existing catkin workspace at ~/catkin_ws"
fi

# 3. Copy package to workspace
echo ""
echo "Step 2: Installing ROS1 package..."
cp -r yolo_detector_ros1 ~/catkin_ws/src/

# 4. Build package
echo ""
echo "Step 3: Building package..."
cd ~/catkin_ws
catkin_make --pkg yolo_detector_ros1

# 5. Check if build succeeded
if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "Installation complete!"
    echo "========================================="
    echo ""
    echo "To use the package:"
    echo "  1. Source the workspace:"
    echo "     source ~/catkin_ws/devel/setup.bash"
    echo ""
    echo "  2. Launch the detector:"
    echo "     roslaunch yolo_detector_ros1 yolo_detector.launch"
    echo ""
    echo "  3. View detections:"
    echo "     rostopic echo /vision/detections"
    echo ""
    echo "  4. View annotated image (optional):"
    echo "     rosrun image_view image_view image:=/vision/annotated_image"
    echo ""
else
    echo ""
    echo "========================================="
    echo "Build failed!"
    echo "========================================="
    echo "Please check the error messages above"
    exit 1
fi
