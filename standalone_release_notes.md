# Standalone YOLO Object Detector (No ROS Required)

Simple, pure Python YOLO object detection with **ZERO ROS dependencies**. Perfect for robots and systems using standalone Python scripts.

## 📦 What's Included

- **Standalone Python script** - Single file, easy to use
- **YOLOv8 nano model** (6.2MB, pre-trained on COCO)
- **No ROS required** - Just Python + OpenCV + YOLO
- **Command-line interface** - Many options and configurations
- **Automated installation** - One-command setup

## 🚀 Quick Start

### Download and Install

```bash
# Download
wget https://github.com/madhavUF/ros2-picarx/releases/download/v1.0.0-standalone/yolo_standalone_export_20251114_131556.tar.gz

# Extract
tar -xzf yolo_standalone_export_20251114_131556.tar.gz
cd yolo_standalone_export_20251114_131556/

# Install
./install.sh
```

### Run Detection

```bash
# Basic detection (console output)
python3 yolo_detector_standalone.py

# Show live video with detections
python3 yolo_detector_standalone.py --display

# Save annotated video
python3 yolo_detector_standalone.py --save-video output.mp4

# Run for 60 seconds
python3 yolo_detector_standalone.py --duration 60
```

## ✨ Features

- ✅ **No ROS dependency** - Pure Python implementation
- ✅ **Easy to use** - Single command to run
- ✅ **Real-time detection** - 10 FPS default (configurable)
- ✅ **80 object classes** - COCO dataset (person, car, etc.)
- ✅ **Flexible camera support** - Any USB camera
- ✅ **Optional video display** - See detections live
- ✅ **Video recording** - Save annotated videos
- ✅ **Detection statistics** - Summary of all detected objects
- ✅ **Python module** - Can be imported and used in your code

## 📋 Requirements

- **Python**: 3.6+ (3.8+ recommended)
- **Camera**: USB camera or video device
- **Optional**: CUDA for GPU acceleration
- **NO ROS REQUIRED!**

## 🎯 Command-Line Options

```
--camera DEVICE       Camera device (0, 1, or /dev/video0)
--model PATH          YOLO model path (default: yolov8n.pt)
--confidence FLOAT    Detection threshold 0-1 (default: 0.5)
--display             Show video window
--save-video PATH     Save annotated video to file
--fps INT             Target FPS (default: 10)
--duration SECONDS    Run for specified duration
--max-frames INT      Process max number of frames
--help                Show all options
```

## 💡 Example Usage

```bash
# Use specific camera
python3 yolo_detector_standalone.py --camera /dev/video0

# Higher confidence, lower FPS
python3 yolo_detector_standalone.py --confidence 0.7 --fps 5

# Display + save video
python3 yolo_detector_standalone.py --display --save-video detections.mp4

# Run for 2 minutes with live display
python3 yolo_detector_standalone.py --display --duration 120
```

## 🔌 Use as Python Module

```python
from yolo_detector_standalone import YOLODetector

# Create detector
detector = YOLODetector(camera_device=0, confidence=0.5)

# Process frames
success, detections = detector.detect_frame()
for det in detections:
    print(f"{det['class']}: {det['confidence']:.2f}")

detector.cleanup()
```

## 📊 Example Output

```
Loading YOLO model: yolov8n.pt
YOLO model loaded successfully
Opening camera: 0
Camera opened: 640x480

Starting continuous detection...
Press Ctrl+C to stop

Frame 10: 2 detections - person (0.92), laptop (0.78)
Frame 30: 1 detections - person (0.88)
Frame 50: 3 detections - person (0.91), cell phone (0.65), cup (0.54)

============================================================
DETECTION SUMMARY
============================================================
Duration: 45.2 seconds
Total Frames: 452
Average FPS: 10.0

Object Detections:
  person: 340
  laptop: 120
  cell phone: 45
  cup: 23
============================================================
```

## 🔧 Camera Configuration

```bash
# Find available cameras
ls -l /dev/video*

# Try different cameras
python3 yolo_detector_standalone.py --camera 0
python3 yolo_detector_standalone.py --camera 1
python3 yolo_detector_standalone.py --camera /dev/video10
```

## 🤖 Perfect For

- **Robots without ROS** - Simple Python scripts
- **Quick prototyping** - No complex setup
- **Educational projects** - Easy to understand
- **Integration** - Use as module in existing code
- **Remote systems** - No display required (console output)
- **Video analysis** - Save and review later

## 📖 Documentation

Complete documentation included in package:
- Detailed installation guide
- All command-line options explained
- Python module usage examples
- Troubleshooting tips
- Camera configuration help
- Performance optimization

## 🆚 Comparison with ROS Version

| Feature | Standalone | ROS1 Version |
|---------|-----------|--------------|
| Dependencies | Python only | ROS1 + catkin |
| Setup time | 1 minute | 5-10 minutes |
| Package size | 5.7MB | 11MB |
| Workspace needed | No | Yes (catkin_ws) |
| Use case | Simple scripts | ROS ecosystem |

## 🔗 Source Code

Browse source in this repository:
- `yolo_detector_standalone.py` - Main script
- `export_yolo_standalone.sh` - Export script

## 💪 Why Choose Standalone?

**Choose this if:**
- ✅ You don't use ROS
- ✅ You want simple Python scripts
- ✅ You need quick deployment
- ✅ You're building prototypes
- ✅ You want to integrate YOLO into existing Python code

**Choose ROS1 version if:**
- You have an existing ROS1 workspace
- You need ROS topics and messages
- You're building a ROS-based system

## 🎁 Bonus: Works Everywhere

- ✅ Raspberry Pi
- ✅ Desktop Linux
- ✅ Ubuntu Server
- ✅ Robots (Husky, TurtleBot, custom)
- ✅ MacOS (with webcam)
- ✅ Any system with Python + USB camera

---

**Package Size**: 5.7MB
**Build Date**: November 14, 2025
**YOLO Model**: YOLOv8n (nano)
**License**: MIT
**No ROS Required**: ✅
