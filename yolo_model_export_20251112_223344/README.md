# YOLO Model Transfer Package

## Contents

- `yolov8n.pt`: YOLOv8 nano model (6.2MB)
- `my_first_pkg/`: Complete ROS2 package with YOLO detector node
- `requirements.txt`: Python dependencies
- `install.sh`: Automated installation script

## Installation on Target Robot

### Option 1: Automated Installation
```bash
./install.sh
```

### Option 2: Manual Installation

1. Install dependencies:
```bash
pip3 install -r requirements.txt
```

2. Copy package to ROS2 workspace:
```bash
cp -r my_first_pkg ~/ros2_ws/src/
cp yolov8n.pt ~/ros2_ws/src/my_first_pkg/
```

3. Build package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_first_pkg
source install/setup.bash
```

## Testing

Test YOLO detection:
```bash
ros2 launch my_first_pkg vision_only.launch.py
```

View detections:
```bash
ros2 topic echo /vision/detections
```

## Configuration

Model path can be changed via parameter:
```bash
ros2 param set /yolo_detector model_path /path/to/your/model.pt
```

Camera device:
```bash
ros2 param set /yolo_detector camera_device /dev/video0
```

## Requirements

- ROS2 (Humble/Iron/Jazzy)
- Python 3.8+
- USB camera or compatible video device
- CUDA (optional, for GPU acceleration)

