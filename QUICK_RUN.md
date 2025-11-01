# Quick Run Guide - Room Scanning Mission

## ✅ Setup Complete!

Your room scanning mission is **built and ready to run**!

## 🚀 Run the Mission

### Option 1: Test in Simulation (Recommended First)
```bash
./run_room_scan.sh simulation
```
This will simulate the robot movement without actually moving hardware. Perfect for testing!

### Option 2: Run with Real Hardware
```bash
./run_room_scan.sh
```
The robot will physically rotate 360° and scan for objects.

## 🔄 Rebuild (if you make changes)
```bash
./build_package.sh
```

## 📊 What You'll See

The robot will:
1. Rotate in place, stopping at 8 positions
2. Pause 2 seconds at each position to detect objects
3. Display findings at each position
4. Generate final report at the end

Example output:
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
```

## ⚙️ Customize the Mission

Edit the launch file parameters:

```bash
# More scanning positions (12 = every 30°)
docker run -it --rm --entrypoint="" --privileged --network host \
  -v /dev:/dev -v $(pwd):/workspace -w /workspace \
  picarx-ros2:humble bash -c "
    source /opt/ros/humble/setup.bash && source install/setup.bash &&
    ros2 launch my_first_pkg room_scan.launch.py scan_positions:=12
  "

# Longer pause at each position
docker run -it --rm --entrypoint="" --privileged --network host \
  -v /dev:/dev -v $(pwd):/workspace -w /workspace \
  picarx-ros2:humble bash -c "
    source /opt/ros/humble/setup.bash && source install/setup.bash &&
    ros2 launch my_first_pkg room_scan.launch.py pause_duration:=3.0
  "
```

## 🛑 Stop the Mission

Press **Ctrl+C** in the terminal to stop at any time.

## 📖 Full Documentation

See `ROOM_SCAN_MISSION.md` for complete details, troubleshooting, and customization options.

## 🔧 Manual Docker Command (Advanced)

If you prefer to run commands manually:

```bash
# Enter the Docker container
docker run -it --rm --entrypoint="" --privileged --network host \
  -v /dev:/dev -v $(pwd):/workspace -w /workspace \
  picarx-ros2:humble bash

# Inside container:
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run mission
ros2 launch my_first_pkg room_scan.launch.py

# Or run nodes individually
ros2 run my_first_pkg yolo_detector_node
ros2 run my_first_pkg robot_controller_node --ros-args -p simulation_mode:=true
ros2 run my_first_pkg room_scanner_node
```

## 📝 Files Created

- `room_scanner_node.py` - Mission control logic
- `room_scan.launch.py` - Launch configuration
- `run_room_scan.sh` - Easy run script
- `build_package.sh` - Build script
- `ROOM_SCAN_MISSION.md` - Full documentation

Ready to scan! 🎯
