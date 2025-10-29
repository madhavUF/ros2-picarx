# Quick Start Guide - ROS2 Person Following

## 🚀 Quick Launch

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_first_pkg
source install/setup.bash
```

### 2. Launch the System

**Full system (with hardware):**
```bash
ros2 launch my_first_pkg person_following.launch.py
```

**Simulation mode (no hardware):**
```bash
ros2 launch my_first_pkg person_following.launch.py simulation:=true
```

**Vision testing only:**
```bash
ros2 launch my_first_pkg vision_only.launch.py publish_annotated_image:=true
```

## 📊 Monitoring

**View all topics:**
```bash
ros2 topic list
```

**Watch detections:**
```bash
ros2 topic echo /vision/detections
```

**Watch person tracking:**
```bash
ros2 topic echo /tracking/target_person
```

**Watch robot commands:**
```bash
ros2 topic echo /control/robot_command
```

**View annotated video:**
```bash
ros2 run rqt_image_view rqt_image_view /vision/annotated_image
```

## 🎛️ Tuning Parameters

**Adjust robot speed:**
```bash
ros2 param set /behavior_controller base_speed 30.0
ros2 param set /behavior_controller turn_speed 25.0
```

**Adjust detection confidence:**
```bash
ros2 param set /yolo_detector confidence_threshold 0.4
```

**Adjust tracking sensitivity:**
```bash
ros2 param set /person_tracker min_area 3000.0
```

## 🔍 Debugging

**Check if nodes are running:**
```bash
ros2 node list
```

**Check topic connections:**
```bash
rqt_graph
```

**Record data for later analysis:**
```bash
ros2 bag record -a
```

**Replay recorded data:**
```bash
ros2 bag play <bag_file>
```

## 🛑 Emergency Stop

**Kill all nodes:**
```bash
Ctrl+C (in the terminal running the launch file)
```

**Or stop individual node:**
```bash
ros2 lifecycle set /robot_controller shutdown
```

## 📋 System Status

**Check system health:**
```bash
# Node status
ros2 node list

# Topic frequency
ros2 topic hz /vision/detections
ros2 topic hz /tracking/target_person

# Parameter values
ros2 param list /behavior_controller
```

## 🔧 Common Issues

**Camera not found:**
```bash
# List cameras
ls -l /dev/video*

# Set correct camera
ros2 param set /yolo_detector camera_device /dev/video0
```

**Robot not moving:**
```bash
# Check if in simulation mode
ros2 param get /robot_controller simulation_mode

# Check commands are being sent
ros2 topic echo /control/robot_command
```

**No person detected:**
```bash
# Lower detection threshold
ros2 param set /yolo_detector confidence_threshold 0.3

# Check camera feed
ros2 topic echo /vision/detections
```

## 📈 Performance

**Check CPU/Memory usage:**
```bash
top -p $(pgrep -d',' python3)
```

**Check message rates:**
```bash
ros2 topic hz /vision/detections
ros2 topic bw /vision/detections
```

## 💾 Save Configuration

**Dump current parameters:**
```bash
ros2 param dump /behavior_controller > my_config.yaml
```

**Load saved parameters:**
```bash
ros2 launch my_first_pkg person_following.launch.py \
  --params-file my_config.yaml
```
