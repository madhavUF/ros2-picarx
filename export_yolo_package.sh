#!/bin/bash
#
# Export YOLO Model and ROS2 Package for Transfer
#

echo "========================================="
echo "Exporting YOLO Model Package"
echo "========================================="

# Create export directory
EXPORT_DIR="yolo_model_export_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$EXPORT_DIR"

echo "Export directory: $EXPORT_DIR"

# Copy YOLO model
echo "Copying YOLO model file..."
cp yolov8n.pt "$EXPORT_DIR/"

# Copy ROS2 package
echo "Copying ROS2 package..."
cp -r src/my_first_pkg "$EXPORT_DIR/"

# Copy dependencies list
echo "Creating requirements.txt..."
cat > "$EXPORT_DIR/requirements.txt" << EOF
# YOLO and Vision Dependencies
ultralytics>=8.0.0
opencv-python>=4.8.0
torch>=2.0.0
torchvision>=0.15.0
numpy>=1.24.0

# ROS2 Python Dependencies
cv-bridge
sensor-msgs
EOF

# Create installation script
echo "Creating install script..."
cat > "$EXPORT_DIR/install.sh" << 'EOF'
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
EOF

chmod +x "$EXPORT_DIR/install.sh"

# Create README
cat > "$EXPORT_DIR/README.md" << EOF
# YOLO Model Transfer Package

## Contents

- \`yolov8n.pt\`: YOLOv8 nano model (6.2MB)
- \`my_first_pkg/\`: Complete ROS2 package with YOLO detector node
- \`requirements.txt\`: Python dependencies
- \`install.sh\`: Automated installation script

## Installation on Target Robot

### Option 1: Automated Installation
\`\`\`bash
./install.sh
\`\`\`

### Option 2: Manual Installation

1. Install dependencies:
\`\`\`bash
pip3 install -r requirements.txt
\`\`\`

2. Copy package to ROS2 workspace:
\`\`\`bash
cp -r my_first_pkg ~/ros2_ws/src/
cp yolov8n.pt ~/ros2_ws/src/my_first_pkg/
\`\`\`

3. Build package:
\`\`\`bash
cd ~/ros2_ws
colcon build --packages-select my_first_pkg
source install/setup.bash
\`\`\`

## Testing

Test YOLO detection:
\`\`\`bash
ros2 launch my_first_pkg vision_only.launch.py
\`\`\`

View detections:
\`\`\`bash
ros2 topic echo /vision/detections
\`\`\`

## Configuration

Model path can be changed via parameter:
\`\`\`bash
ros2 param set /yolo_detector model_path /path/to/your/model.pt
\`\`\`

Camera device:
\`\`\`bash
ros2 param set /yolo_detector camera_device /dev/video0
\`\`\`

## Requirements

- ROS2 (Humble/Iron/Jazzy)
- Python 3.8+
- USB camera or compatible video device
- CUDA (optional, for GPU acceleration)

EOF

# Create archive
echo "Creating compressed archive..."
tar -czf "${EXPORT_DIR}.tar.gz" "$EXPORT_DIR"

echo ""
echo "========================================="
echo "Export Complete!"
echo "========================================="
echo ""
echo "Package created:"
echo "  Directory: $EXPORT_DIR/"
echo "  Archive:   ${EXPORT_DIR}.tar.gz"
echo ""
echo "Transfer to target robot:"
echo "  scp ${EXPORT_DIR}.tar.gz user@robot-ip:~/"
echo ""
echo "Or copy to USB drive:"
echo "  cp ${EXPORT_DIR}.tar.gz /media/usb/"
echo ""
