# ROS2 PiCar-X Room Scanner - Project Overview

## What You Have: TWO Implementations

### 🎯 What You're Actually Using Now: **STANDALONE (No ROS2)**

**File:** `room_scan_standalone.py`
**Run:** `./SCAN_ROOM.sh`

```
┌─────────────────────────────────────────┐
│         STANDALONE ARCHITECTURE         │
│         (What's Actually Working)       │
└─────────────────────────────────────────┘

    Direct Python Script (No ROS2)
              │
    ┌─────────┴──────────┐
    │                    │
    ▼                    ▼
┌────────┐          ┌──────────┐
│ PicarX │          │   YOLO   │
│Hardware│          │ Detector │
└────┬───┘          └─────┬────┘
     │                    │
     │  ┌─────────────┐   │
     └─►│ Ultrasonic  │   │
        │   Sensor    │   │
        └─────────────┘   │
                          │
        ┌─────────────────┘
        │
        ▼
    ┌────────────┐
    │   Camera   │
    │  /dev/     │
    │  video10   │
    └────────────┘

Components:
- Direct hardware access (no middleware)
- Single Python process
- Runs natively on Raspberry Pi
- Real-time obstacle avoidance
- YOLO object detection
```

**Why This Works:**
- ✅ Direct GPIO access (no Docker barrier)
- ✅ Simple architecture
- ✅ Fast and reliable
- ✅ Hardware actually moves

---

### 🏗️ What We Also Built: **ROS2 VERSION (Not Currently Used)**

**Files:** `src/my_first_pkg/` with nodes, launch files
**Run:** Would need Docker with proper GPIO access

```
┌─────────────────────────────────────────┐
│          ROS2 ARCHITECTURE              │
│      (Built but not working due to      │
│       Docker GPIO limitations)          │
└─────────────────────────────────────────┘

        ROS2 Launch File
              │
    ┌─────────┼─────────┐
    │         │         │
    ▼         ▼         ▼
┌───────┐ ┌──────┐ ┌──────────┐
│ YOLO  │ │Robot │ │  Room    │
│Detector│ │Ctrl  │ │ Scanner  │
│ Node  │ │ Node │ │  Node    │
└───┬───┘ └──┬───┘ └────┬─────┘
    │        │          │
    │        │          │
    ▼        ▼          ▼
  Topics   Topics    Topics
    │        │          │
/vision/ /control/ (coordinates)
detections robot_cmd

Problem: Docker + GPIO = 💥
"Cannot determine SOC peripheral base address"
```

**Why We Stopped Using It:**
- ❌ Docker can't access GPIO properly
- ❌ Hardware initialization fails
- ❌ Falls back to simulation mode
- ❌ Robot doesn't actually move

---

## Architecture Comparison

### Standalone (Current - Working)
```python
# One file, direct control
from picarx import Picarx
from ultralytics import YOLO

px = Picarx()
model = YOLO('yolov8n.pt')

while scanning:
    px.forward(20)  # Direct hardware call
    detections = model(frame)  # Direct YOLO call
    if obstacle_detected():
        px.backward(20)  # Direct control
```

**Pros:**
- ✅ Actually works with hardware
- ✅ Simple to understand
- ✅ Easy to modify
- ✅ No middleware overhead

**Cons:**
- ❌ No distributed computing
- ❌ No ROS ecosystem tools
- ❌ Everything in one process

---

### ROS2 Version (Built but Not Used)
```python
# Distributed nodes with message passing

# Node 1: YOLO Detector
detections = yolo_model(frame)
pub.publish(DetectionArray(detections))

# Node 2: Room Scanner
def detection_callback(msg):
    for det in msg.detections:
        objects_found[det.class_name] += 1

# Node 3: Robot Controller
def command_callback(msg):
    if msg.action == 'forward':
        px.forward(msg.speed)  # Fails in Docker!
```

**Pros:**
- ✅ Professional ROS2 architecture
- ✅ Distributed nodes
- ✅ Reusable components
- ✅ Standard messaging

**Cons:**
- ❌ Docker GPIO barriers
- ❌ More complex
- ❌ Requires ROS2 environment

---

## What Actually Runs

### When You Execute: `./SCAN_ROOM.sh`

```bash
#!/bin/bash
1. Start camera bridge (rpicam-vid → /dev/video10)
2. Test camera works
3. Run: python3 room_scan_standalone.py
```

### The Standalone Script Does:

```python
class RoomScanner:
    def __init__(self):
        self.px = Picarx()              # Hardware
        self.model = YOLO('yolov8n.pt') # AI
        self.cap = cv2.VideoCapture()   # Camera

    def run_mission(self):
        for position in range(8):       # 8 positions
            rotate_robot()              # Hardware moves
            check_obstacles()           # Ultrasonic
            detect_objects()            # YOLO
            report_findings()           # Print
```

**Data Flow:**
```
Camera → OpenCV → YOLO → Object List
                            ↓
Ultrasonic → Distance Check → Safe?
                            ↓
                    PicarX Hardware
                            ↓
                    Robot Moves!
```

---

## Technology Stack

### What's Actually Being Used:

**Hardware Layer:**
- PicarX robot platform
- Raspberry Pi Camera Module 3
- Ultrasonic distance sensor (HC-SR04)
- DC motors + servo

**Software Stack:**
```
Application:     room_scan_standalone.py
                          │
Libraries:       ┌────────┼────────┐
                 │        │        │
             PicarX    YOLO    OpenCV
                 │        │        │
Low-Level:   robot_hat  PyTorch  v4l2
                 │                 │
Hardware:     GPIO/I2C        Camera
```

**NOT Currently Used:**
- ❌ ROS2 (built but not running)
- ❌ Docker (only for building)
- ❌ ros2_control
- ❌ nav2

---

## Files You Actually Use

### Active Files (What Runs):
```
room_scan_standalone.py    ← Main script
SCAN_ROOM.sh              ← Launcher
camera_bridge.sh          ← Camera setup
camera_control.sh         ← Camera management
yolov8n.pt               ← AI model
```

### Built But Not Used:
```
src/my_first_pkg/        ← ROS2 package
  ├── yolo_detector_node.py
  ├── robot_controller_node.py
  ├── room_scanner_node.py
  └── launch/
      └── room_scan.launch.py
```

---

## Summary

**What You Asked For:**
"Robot to scan room and report objects"

**What We Delivered:**
1. **ROS2 Version** - Professional architecture (in codebase)
2. **Standalone Version** - Actually works (what you run)

**Current Reality:**
```
You're running: Standalone Python script
Architecture:   Direct hardware control
ROS2:          Built but not active
Docker:        Not needed for current version
```

**Why Two Versions?**
- Started with ROS2 (professional, complex)
- Hit Docker + GPIO barriers
- Built standalone (simple, works)
- Kept ROS2 code for future

---

## If You Want ROS2 Later

To actually use the ROS2 version:

**Option 1: Install ROS2 on Pi (Native)**
```bash
# Install ROS2 Humble natively
sudo apt install ros-humble-desktop

# Run without Docker
source /opt/ros/humble/setup.bash
ros2 launch my_first_pkg room_scan.launch.py
```

**Option 2: Fix Docker GPIO**
- Complex kernel module setup
- Not recommended
- Current standalone version is better

---

## Bottom Line

**Current Status:**
```
✅ Room scanning works
✅ Object detection works
✅ Obstacle avoidance works
❌ Not using ROS2 (because hardware access)
✅ Using direct Python instead
```

**You Have:**
- Working room scanner
- Obstacle avoidance
- Object detection
- Simple, maintainable code

**You Don't Have (Currently):**
- ROS2 runtime
- Distributed nodes
- Standard ROS topics
- (But the code exists if you want it later)
