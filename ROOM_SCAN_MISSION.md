# Room Scanning Mission

I've created a complete room scanning mission system for your PiCar-X robot!

## What It Does

The robot will:
1. **Rotate 360 degrees** in place, stopping at multiple positions
2. **Detect objects** using YOLO at each position
3. **Report all findings** with detailed statistics
4. Keep track of every object seen during the scan

## Files Created

### 1. Room Scanner Node
**Location:** `src/my_first_pkg/my_first_pkg/room_scanner_node.py`

This is the mission orchestrator that:
- Controls the robot's scanning behavior
- Collects object detections from YOLO
- Tracks unique objects and counts
- Generates a comprehensive mission report

### 2. Launch File
**Location:** `src/my_first_pkg/launch/room_scan.launch.py`

Starts all required nodes:
- YOLO Detector (for object detection)
- Robot Controller (for movement)
- Room Scanner (mission orchestrator)

## How to Use

### Step 1: Setup ROS2 Environment

Your project appears to use Docker. You have two options:

**Option A: Using Docker (Recommended)**
```bash
# Build the Docker image (if not already built)
docker build -t my-picar-autonomous .

# Run the container with hardware access
docker run -it --privileged \
  -v /dev:/dev \
  -v $(pwd):/workspace \
  my-picar-autonomous bash

# Inside the container:
cd /workspace
source /opt/ros/humble/setup.bash
```

**Option B: Install ROS2 Humble on Raspberry Pi**
```bash
# Install ROS2 Humble (if not installed)
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS2
source /opt/ros/humble/setup.bash
```

### Step 2: Build the Package

```bash
# Clean old builds
rm -rf build install log

# Build with ROS2 sourced
colcon build --packages-select my_first_pkg

# Source the workspace
source install/setup.bash
```

### Step 3: Run the Mission

**Full hardware mode (real robot):**
```bash
ros2 launch my_first_pkg room_scan.launch.py
```

**Simulation mode (testing without hardware):**
```bash
ros2 launch my_first_pkg room_scan.launch.py simulation_mode:=true
```

**Custom configuration:**
```bash
# More positions for thorough scanning (12 positions = 30° each)
ros2 launch my_first_pkg room_scan.launch.py scan_positions:=12

# Longer pause to detect more objects
ros2 launch my_first_pkg room_scan.launch.py pause_duration:=3.0

# Different YOLO model
ros2 launch my_first_pkg room_scan.launch.py yolo_model_path:=/path/to/yolov8s.pt
```

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `simulation_mode` | false | Run without hardware |
| `scan_positions` | 8 | Number of stops during 360° rotation |
| `pause_duration` | 2.0 | Seconds to pause at each position |
| `scan_speed` | 20.0 | Robot turning speed |
| `turn_angle` | 30.0 | Steering angle for turns |
| `confidence_threshold` | 0.6 | Minimum YOLO confidence to report |
| `yolo_model_path` | yolov8n.pt | Path to YOLO model |

## What You'll See

During the mission, the terminal will show:

```
[room_scanner_node]: MISSION START: Room Scanning Initiated
[room_scanner_node]: --- Scan Position 1/8 ---
[room_scanner_node]: Pausing for 2.0s to detect objects...
[room_scanner_node]:   Detected at position 1:
[room_scanner_node]:     - person: 3 detections (avg confidence: 0.87)
[room_scanner_node]:     - chair: 2 detections (avg confidence: 0.76)
...
[room_scanner_node]: MISSION COMPLETE: Room Scan Finished
[room_scanner_node]: OBJECTS FOUND IN ROOM:
[room_scanner_node]: 1. PERSON: 24 detections
[room_scanner_node]: 2. CHAIR: 16 detections
[room_scanner_node]: 3. LAPTOP: 8 detections
[room_scanner_node]: Total unique objects: 3
[room_scanner_node]: Total detections: 48
```

## Monitoring the Mission

In separate terminals (while mission is running):

**Watch detections in real-time:**
```bash
ros2 topic echo /vision/detections
```

**Watch robot commands:**
```bash
ros2 topic echo /control/robot_command
```

**Check node status:**
```bash
ros2 node list
```

## Customizing the Mission

### Change Scanning Pattern

Edit `src/my_first_pkg/my_first_pkg/room_scanner_node.py`:

- Modify `start_mission()` to add forward movement between rotations
- Add obstacle avoidance
- Implement grid-based scanning
- Add waypoint navigation

### Add More Features

- **Save results to file**: Add file writing in `report_findings()`
- **Take photos**: Subscribe to camera topic and save images
- **Audio announcements**: Use text-to-speech to announce findings
- **Web dashboard**: Create a web interface to monitor progress

## Troubleshooting

**Camera not working:**
```bash
ls -l /dev/video*  # Check camera device
# Update in launch file if needed
```

**Robot not moving:**
```bash
# Check simulation mode
ros2 param get /robot_controller_node simulation_mode

# Check hardware initialization
ros2 topic echo /rosout | grep robot_controller
```

**No objects detected:**
```bash
# Lower confidence threshold
ros2 param set /yolo_detector_node confidence_threshold 0.3

# Check YOLO model is loaded
ros2 topic echo /rosout | grep yolo
```

## Next Steps

Once the basic mission works, you can:

1. **Multi-room scanning**: Add navigation between rooms
2. **Object localization**: Calculate actual positions using distance sensors
3. **Mapping**: Build a map while scanning
4. **Interactive missions**: Let user specify what objects to find
5. **Continuous patrol**: Repeat the scan periodically

## Technical Details

### ROS2 Topics Used

| Topic | Type | Description |
|-------|------|-------------|
| `/vision/detections` | DetectionArray | Object detections from YOLO |
| `/control/robot_command` | RobotCommand | Movement commands to robot |

### Architecture

```
┌─────────────────┐
│ YOLO Detector   │──────┐
│ Node            │      │
└─────────────────┘      │
                         ↓
                    /vision/detections
                         ↓
┌─────────────────┐      │
│ Room Scanner    │◄─────┘
│ Node            │
└─────────────────┘
        │
        ↓
  /control/robot_command
        │
        ↓
┌─────────────────┐
│ Robot           │
│ Controller Node │
└─────────────────┘
```

## Safety Notes

- Ensure the robot has clear space to rotate 360°
- Keep the robot on the ground (not on tables/elevated surfaces)
- Monitor the first run in simulation mode
- Be ready to press Ctrl+C to stop the mission
- The robot will stop automatically when mission completes

Enjoy your room scanning mission!
