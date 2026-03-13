#!/bin/bash
# Installation script for target robot

echo "Installing YOLO Model Package..."

# 1. Install Python dependencies
echo "Step 1: Installing Python dependencies..."
pip3 install -r requirements.txt

# 2. Copy ROS2 package
echo "Step 2: Installing ROS2 package..."
if [ ! -d ~/ros2_ws/src ]; then
    echo "Error: ROS2 workspace not found at ~/ros2_ws"
    echo "Please create it first: mkdir -p ~/ros2_ws/src"
    exit 1
fi

cp -r my_first_pkg ~/ros2_ws/src/

# 3. Copy YOLO model
echo "Step 3: Installing YOLO model..."
cp yolov8n.pt ~/ros2_ws/src/my_first_pkg/

# 4. Build ROS2 package
echo "Step 4: Building ROS2 package..."
cd ~/ros2_ws
colcon build --packages-select my_first_pkg
source install/setup.bash

echo "========================================="
echo "Installation complete!"
echo "========================================="
echo ""
echo "To test:"
echo "  source ~/ros2_ws/install/setup.bash"
echo "  ros2 launch my_first_pkg vision_only.launch.py"
echo ""
