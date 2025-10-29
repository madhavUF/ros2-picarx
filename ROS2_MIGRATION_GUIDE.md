# ROS2 Migration Guide: Person Following System

This guide explains the migration from standalone Python scripts to a full ROS2 implementation for the person-following system.

## 🎯 Overview

The new ROS2 architecture provides:
- **Modular Design**: Each component is a separate node
- **Reusability**: Nodes can be reused in different configurations
- **Testability**: Can record and replay data with rosbag
- **Configurability**: Parameters can be tuned at runtime
- **Scalability**: Easy to add new behaviors and sensors

## 🏗️ Architecture

### System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                 PERCEPTION LAYER                         │
├─────────────────────────────────────────────────────────┤
│  yolo_detector_node                                      │
│  ↓ /vision/detections (DetectionArray)                  │
│  ↓ /vision/annotated_image (Image)                      │
│                                                          │
│  multi_sensor_publisher                                  │
│  ↓ /sensor/distance (Float32)                           │
│  ↓ /sensor/grayscale (String)                           │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│                 PROCESSING LAYER                         │
├─────────────────────────────────────────────────────────┤
│  person_tracker_node                                     │
│  ← /vision/detections                                    │
│  ↓ /tracking/target_person (TargetPerson)               │
│  ↓ /tracking/person_state (String)                      │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│                  DECISION LAYER                          │
├─────────────────────────────────────────────────────────┤
│  behavior_controller_node                                │
│  ← /tracking/target_person                               │
│  ← /sensor/distance                                      │
│  ↓ /control/robot_command (RobotCommand)                │
│  ↓ /control/camera_command (CameraCommand)              │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│                  ACTUATION LAYER                         │
├─────────────────────────────────────────────────────────┤
│  robot_controller_node                                   │
│  ← /control/robot_command                                │
│  → Hardware: Motors, Steering                           │
│                                                          │
│  camera_servo_node                                       │
│  ← /control/camera_command                               │
│  → Hardware: Pan/Tilt Servos                            │
└─────────────────────────────────────────────────────────┘
```

## 📦 Custom Message Types

### Detection.msg
Single object detection from YOLO:
```
string class_name
float32 confidence
float32 x1, y1, x2, y2  # Bounding box
float32 center_x, center_y
float32 area
```

### DetectionArray.msg
Array of detections:
```
std_msgs/Header header
Detection[] detections
int32 frame_width, frame_height
```

### TargetPerson.msg
Tracked person information:
```
std_msgs/Header header
bool person_detected
float32 confidence
float32 center_x, center_y
float32 area
float32 distance_estimate
float32 horizontal_offset
```

### RobotCommand.msg
Robot movement command:
```
std_msgs/Header header
string action  # forward, backward, stop, turn_left, turn_right
float32 speed
float32 steering_angle
```

### CameraCommand.msg
Camera servo command:
```
std_msgs/Header header
float32 pan_angle   # -90 to 90
float32 tilt_angle  # -30 to 90
bool reset_to_center
```

## 🚀 Building the System

### 1. Install Dependencies

```bash
# ROS2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Python dependencies
pip3 install ultralytics opencv-python cv_bridge
```

### 2. Build the Package

```bash
cd ~/ros2_ws  # or your workspace
colcon build --packages-select my_first_pkg
source install/setup.bash
```

### 3. Verify Build

```bash
# Check if messages are available
ros2 interface list | grep my_first_pkg

# Check if nodes are available
ros2 pkg executables my_first_pkg
```

## 🎮 Running the System

### Full Person Following System

```bash
# Launch with default parameters
ros2 launch my_first_pkg person_following.launch.py

# Launch in simulation mode (no hardware)
ros2 launch my_first_pkg person_following.launch.py simulation:=true

# Launch with custom model
ros2 launch my_first_pkg person_following.launch.py model_path:=/path/to/yolov8n.pt
```

### Vision Testing Only

```bash
# Test vision without moving robot
ros2 launch my_first_pkg vision_only.launch.py

# With image visualization
ros2 launch my_first_pkg vision_only.launch.py publish_annotated_image:=true
```

### Individual Nodes

```bash
# YOLO Detector
ros2 run my_first_pkg yolo_detector_node

# Person Tracker
ros2 run my_first_pkg person_tracker_node

# Behavior Controller
ros2 run my_first_pkg behavior_controller_node

# Robot Controller
ros2 run my_first_pkg robot_controller_node

# Camera Servo
ros2 run my_first_pkg camera_servo_node
```

## 🔧 Configuration

### Using Parameter Files

```bash
# Launch with custom parameters
ros2 launch my_first_pkg person_following.launch.py \
  --params-file src/my_first_pkg/config/person_following_params.yaml

# Launch in simulation mode
ros2 launch my_first_pkg person_following.launch.py \
  --params-file src/my_first_pkg/config/simulation_params.yaml
```

### Runtime Parameter Tuning

```bash
# List all parameters
ros2 param list

# Get a parameter value
ros2 param get /behavior_controller base_speed

# Set a parameter value
ros2 param set /behavior_controller base_speed 30.0

# Dump all parameters
ros2 param dump /behavior_controller
```

## 📊 Monitoring and Debugging

### Topic Monitoring

```bash
# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /vision/detections
ros2 topic echo /tracking/target_person
ros2 topic echo /control/robot_command

# Check topic frequency
ros2 topic hz /vision/detections

# Show topic info
ros2 topic info /vision/detections
```

### Visualization

```bash
# View annotated images
ros2 run rqt_image_view rqt_image_view /vision/annotated_image

# RQT graph (node connections)
rqt_graph

# Plot numeric data
rqt_plot /tracking/target_person/horizontal_offset
```

### Recording and Playback

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /vision/detections /tracking/target_person

# Playback recorded data
ros2 bag play <bag_file>
```

## 🔄 Migration from Standalone Scripts

### Before (Standalone Script)

```python
# yolo_robot_headless.py - Everything in one file
model = YOLO('yolov8n.pt')
cap = cv2.VideoCapture('/dev/video10')
px = Picarx()

while True:
    ret, frame = cap.read()
    results = model(frame)
    # Detection logic
    # Tracking logic
    # Decision logic
    # Hardware control
    px.forward(speed)
```

### After (ROS2 Nodes)

**YOLO Detector Node** (Perception):
```python
# yolo_detector_node.py - Only detection
model = YOLO('yolov8n.pt')
results = model(frame)
detection_array_msg = create_detection_msg(results)
publisher.publish(detection_array_msg)
```

**Person Tracker Node** (Processing):
```python
# person_tracker_node.py - Only tracking
def detection_callback(msg):
    persons = filter_persons(msg.detections)
    best_person = select_best(persons)
    target_msg = create_target_msg(best_person)
    publisher.publish(target_msg)
```

**Behavior Controller Node** (Decision):
```python
# behavior_controller_node.py - Only decisions
def target_callback(msg):
    robot_cmd = decide_action(msg)
    camera_cmd = calculate_camera_angle(msg)
    robot_pub.publish(robot_cmd)
    camera_pub.publish(camera_cmd)
```

**Robot Controller Node** (Actuation):
```python
# robot_controller_node.py - Only hardware
def command_callback(msg):
    if msg.action == 'forward':
        px.forward(msg.speed)
```

## 🎛️ Key Parameters

### YOLO Detector
- `model_path`: Path to YOLO model (default: yolov8n.pt)
- `confidence_threshold`: Minimum confidence (default: 0.5)
- `publish_rate`: Detection frequency in Hz (default: 10.0)

### Person Tracker
- `min_confidence`: Minimum person confidence (default: 0.5)
- `min_area`: Minimum bounding box area (default: 5000.0)
- `lost_threshold`: Frames before scanning (default: 10)

### Behavior Controller
- `base_speed`: Forward speed (default: 25.0)
- `turn_speed`: Turning speed (default: 20.0)
- `steering_angle`: Turn angle in degrees (default: 15.0)
- `horizontal_deadzone`: Centering tolerance in pixels (default: 80.0)
- `area_too_close`: Back up threshold (default: 150000.0)
- `area_good_min/max`: Good distance range (default: 50000-150000)
- `obstacle_emergency_distance`: Emergency stop in cm (default: 20.0)

## 🐛 Troubleshooting

### Camera Not Found
```bash
# Check camera device
ls -l /dev/video*

# Test camera
ros2 run my_first_pkg yolo_detector_node --ros-args -p camera_device:=/dev/video0
```

### Hardware Not Working
```bash
# Run in simulation mode
ros2 run my_first_pkg robot_controller_node --ros-args -p simulation_mode:=true
```

### No Detections
```bash
# Lower confidence threshold
ros2 param set /yolo_detector confidence_threshold 0.3

# Check if YOLO model is loaded
ros2 topic echo /vision/detections
```

### Robot Not Moving
```bash
# Check commands are being published
ros2 topic echo /control/robot_command

# Verify robot controller is running
ros2 node list | grep robot_controller
```

## 📈 Benefits of ROS2 Architecture

### 1. Modularity
- Can replace YOLO with different detector
- Can swap behavior controllers
- Easy to add new sensors

### 2. Testability
- Record real data with rosbag
- Test nodes individually
- Replay scenarios repeatedly

### 3. Reusability
- Use detector for other tasks
- Reuse robot controller in other projects
- Share nodes across different robots

### 4. Debugging
- Monitor individual topics
- Visualize data flows
- Isolate problematic components

### 5. Scalability
- Add multiple cameras
- Implement multi-robot coordination
- Integrate with other ROS2 packages

## 🔮 Future Enhancements

Potential additions to the system:
- **SLAM Integration**: Add mapping and localization
- **Path Planning**: Use nav2 for navigation
- **Multi-Person Tracking**: Track multiple people
- **Gesture Recognition**: Respond to hand gestures
- **Voice Commands**: Add speech recognition
- **Web Interface**: Control via web dashboard

## 📚 Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [YOLO Documentation](https://docs.ultralytics.com/)
- [PiCar-X Documentation](https://docs.sunfounder.com/projects/picar-x/)
