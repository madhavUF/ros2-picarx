# YOLO Detector for ROS1

YOLOv8 object detection package for ROS1 (Melodic/Noetic).

## Contents

- `yolo_detector_ros1/`: Complete ROS1 catkin package
  - `scripts/yolo_detector_node.py`: Main detector node
  - `msg/`: Custom message definitions (Detection, DetectionArray)
  - `launch/yolo_detector.launch`: Launch file
  - `yolov8n.pt`: YOLOv8 nano model (6.2MB)
- `requirements.txt`: Python dependencies
- `install.sh`: Automated installation script

## Requirements

- **ROS1**: Melodic or Noetic
- **Python**: 3.6+ (3.8+ recommended)
- **Hardware**: USB camera or compatible video device
- **Optional**: CUDA for GPU acceleration

## Quick Installation

### Option 1: Automated (Recommended)

```bash
# Extract the package
tar -xzf yolo_ros1_export_*.tar.gz
cd yolo_ros1_export_*/

# Run installation script
./install.sh
```

### Option 2: Manual Installation

1. **Install Python dependencies:**
```bash
pip3 install -r requirements.txt
```

2. **Install ROS dependencies:**
```bash
sudo apt-get install ros-$ROS_DISTRO-cv-bridge \
                     ros-$ROS_DISTRO-sensor-msgs \
                     ros-$ROS_DISTRO-image-transport
```

3. **Copy to catkin workspace:**
```bash
cp -r yolo_detector_ros1 ~/catkin_ws/src/
```

4. **Build the package:**
```bash
cd ~/catkin_ws
catkin_make --pkg yolo_detector_ros1
source devel/setup.bash
```

## Usage

### Basic Launch

```bash
# Terminal 1: Start roscore (if not running)
roscore

# Terminal 2: Launch YOLO detector
source ~/catkin_ws/devel/setup.bash
roslaunch yolo_detector_ros1 yolo_detector.launch
```

### View Detections

```bash
# View detection messages
rostopic echo /vision/detections

# View annotated images (requires image_view)
rosrun image_view image_view image:=/vision/annotated_image
```

### Monitor Topics

```bash
# List all topics
rostopic list

# View detection rate
rostopic hz /vision/detections

# View detection details
rostopic echo /vision/detections
```

## Configuration

Edit the launch file to customize parameters:

```bash
roscd yolo_detector_ros1/launch
nano yolo_detector.launch
```

### Available Parameters

- `model_path`: Path to YOLO model file (default: package's yolov8n.pt)
- `confidence_threshold`: Minimum confidence for detections (0.0-1.0, default: 0.5)
- `camera_device`: Camera device path (default: /dev/video0)
- `publish_rate`: Detection frequency in Hz (default: 10.0)
- `publish_annotated_image`: Publish images with bounding boxes (default: true)

### Example Custom Launch

```bash
roslaunch yolo_detector_ros1 yolo_detector.launch \
  camera_device:=/dev/video2 \
  confidence_threshold:=0.7 \
  publish_rate:=5.0
```

## Published Topics

### `/vision/detections` (yolo_detector_ros1/DetectionArray)

Array of detected objects with:
- `header`: Standard ROS header with timestamp
- `detections[]`: Array of Detection messages
- `frame_width`, `frame_height`: Image dimensions

### `/vision/annotated_image` (sensor_msgs/Image)

Camera image with bounding boxes and labels drawn on it.

## Message Definitions

### Detection.msg

```
string class_name      # Object class (e.g., "person", "car")
float32 confidence     # Detection confidence (0.0-1.0)
float32 x1, y1         # Top-left corner
float32 x2, y2         # Bottom-right corner
float32 center_x       # Center X coordinate
float32 center_y       # Center Y coordinate
float32 area           # Bounding box area (pixels)
```

### DetectionArray.msg

```
std_msgs/Header header
Detection[] detections
int32 frame_width
int32 frame_height
```

## Troubleshooting

### Camera Not Opening

```bash
# List available cameras
ls -l /dev/video*

# Test camera
v4l2-ctl --list-devices

# If using /dev/video10 or other device:
roslaunch yolo_detector_ros1 yolo_detector.launch camera_device:=/dev/video10
```

### Python Module Not Found

```bash
# Install missing dependencies
pip3 install ultralytics opencv-python torch torchvision

# Or use requirements.txt
pip3 install -r requirements.txt
```

### Message Import Errors

```bash
# Rebuild messages
cd ~/catkin_ws
catkin_make --pkg yolo_detector_ros1
source devel/setup.bash
```

### Performance Issues

1. **Lower detection rate:**
```bash
roslaunch yolo_detector_ros1 yolo_detector.launch publish_rate:=5.0
```

2. **Increase confidence threshold:**
```bash
roslaunch yolo_detector_ros1 yolo_detector.launch confidence_threshold:=0.7
```

3. **Use GPU (if available):**
   - Ensure PyTorch with CUDA is installed
   - YOLO will automatically use GPU if available

## Integration Examples

### Subscribe to Detections (Python)

```python
#!/usr/bin/env python
import rospy
from yolo_detector_ros1.msg import DetectionArray

def detection_callback(msg):
    rospy.loginfo(f"Received {len(msg.detections)} detections")
    for det in msg.detections:
        rospy.loginfo(f"  {det.class_name}: {det.confidence:.2f}")

rospy.init_node('detection_listener')
rospy.Subscriber('/vision/detections', DetectionArray, detection_callback)
rospy.spin()
```

### Subscribe to Detections (C++)

```cpp
#include <ros/ros.h>
#include <yolo_detector_ros1/DetectionArray.h>

void detectionCallback(const yolo_detector_ros1::DetectionArray::ConstPtr& msg) {
    ROS_INFO("Received %zu detections", msg->detections.size());
    for (const auto& det : msg->detections) {
        ROS_INFO("  %s: %.2f", det.class_name.c_str(), det.confidence);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "detection_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/vision/detections", 10, detectionCallback);
    ros::spin();
    return 0;
}
```

## Advanced Usage

### Custom YOLO Model

Replace yolov8n.pt with your own trained model:

```bash
# Copy your model to package
roscd yolo_detector_ros1
cp /path/to/your/model.pt ./custom_model.pt

# Update launch file or use parameter
roslaunch yolo_detector_ros1 yolo_detector.launch \
  model_path:=$(rospack find yolo_detector_ros1)/custom_model.pt
```

### Multiple Cameras

Launch multiple detector instances:

```bash
# Camera 1
roslaunch yolo_detector_ros1 yolo_detector.launch \
  camera_device:=/dev/video0 \
  __name:=yolo_cam1 \
  __ns:=camera1

# Camera 2
roslaunch yolo_detector_ros1 yolo_detector.launch \
  camera_device:=/dev/video1 \
  __name:=yolo_cam2 \
  __ns:=camera2
```

## License

MIT License

## Support

For issues or questions, refer to:
- YOLOv8 Documentation: https://docs.ultralytics.com
- ROS1 Documentation: http://wiki.ros.org
