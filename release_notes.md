# YOLOv8 Object Detector for ROS1 (Melodic/Noetic)

Complete ROS1 package for YOLOv8 object detection, ready to deploy on robots running ROS1.

## 📦 What's Included

- **Complete ROS1 catkin package** with YOLO detector node
- **YOLOv8 nano model** (6.2MB, pre-trained on COCO dataset)
- **Custom ROS1 messages** for detection data
- **Launch files** for easy startup
- **Automated installation script**
- **Comprehensive documentation** with examples

## 🚀 Quick Start

### Download and Install

```bash
# Download the package
wget https://github.com/madhavUF/ros2-picarx/releases/download/v1.0.0-ros1/yolo_ros1_export_20251114_103240.tar.gz

# Extract
tar -xzf yolo_ros1_export_20251114_103240.tar.gz
cd yolo_ros1_export_20251114_103240/

# Run automated installer
./install.sh
```

### Launch the Detector

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch yolo_detector_ros1 yolo_detector.launch
```

### View Detections

```bash
# Terminal 1: View detection messages
rostopic echo /vision/detections

# Terminal 2: View annotated images (optional)
rosrun image_view image_view image:=/vision/annotated_image
```

## ✨ Features

- ✅ **ROS1 Compatible**: Works with Melodic and Noetic
- ✅ **Real-time Detection**: 10 Hz default (configurable)
- ✅ **80 Object Classes**: Pre-trained COCO dataset (person, car, etc.)
- ✅ **Plug & Play**: Just needs USB camera
- ✅ **Configurable**: Camera device, confidence threshold, publish rate
- ✅ **GPU Support**: Automatic GPU acceleration if CUDA available

## 📋 Requirements

- **ROS1**: Melodic or Noetic
- **Python**: 3.6+ (3.8+ recommended)
- **Camera**: USB camera or compatible video device
- **Optional**: CUDA for GPU acceleration

## 🔧 Configuration

All parameters configurable via launch file or command line:

```bash
roslaunch yolo_detector_ros1 yolo_detector.launch \
  camera_device:=/dev/video1 \
  confidence_threshold:=0.7 \
  publish_rate:=5.0
```

## 📡 Published Topics

- `/vision/detections` - Detection array with bounding boxes and class names
- `/vision/annotated_image` - Camera image with bounding boxes drawn

## 📖 Documentation

Complete documentation included in the package:
- Installation guide (automated and manual)
- Configuration options
- Integration examples (Python and C++)
- Troubleshooting tips
- Advanced usage scenarios

## 🔗 Source Code

Browse the source code in this repository:
- `yolo_detector_ros1/` - ROS1 catkin package
- `export_yolo_ros1.sh` - Export script to create new packages

## 💡 Use Cases

Perfect for:
- Mobile robot object detection
- Warehouse automation
- Security and surveillance robots
- Educational robotics projects
- ROS1 migration from standalone systems

## 🤖 Related

This is a ROS1 port of the YOLO detection system originally developed for the PiCar-X robot. See the repository for the complete ROS2 and standalone versions.

---

**Package Size**: 11MB
**Build Date**: November 14, 2025
**YOLO Model**: YOLOv8n (nano)
**License**: MIT
