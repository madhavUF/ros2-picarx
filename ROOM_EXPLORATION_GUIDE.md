# Advanced Room Exploration System

## What's New? 🚀

The room scanner has been completely redesigned into a comprehensive **Room Exploration System**! Instead of just spinning in place, the robot now:

### Key Improvements

1. **Actual Room Exploration** 🗺️
   - Moves forward to multiple waypoints (not just rotating in place!)
   - Explores different areas of the room systematically
   - Covers 4 waypoints by default with configurable forward movement

2. **Multi-Angle Camera Scanning** 📷
   - **4 tilt angles**: High (50°), Mid-High (25°), Mid (0°), Low (-15°)
   - **3 pan angles**: Left (-45°), Center (0°), Right (45°)
   - **12 camera positions** per rotation direction
   - Scans ceiling, walls, furniture, and floor objects

3. **360° Coverage at Each Position** 🔄
   - Performs 4 rotations (90° each) at every waypoint
   - Full panoramic coverage from multiple room locations
   - Total: **192 scan positions** (4 waypoints × 4 rotations × 12 camera angles)

4. **Spatial Mapping** 📍
   - Tracks WHERE objects were detected
   - Records which waypoint and camera angle found each object
   - Generates spatial distribution reports

5. **Enhanced Obstacle Avoidance** 🛡️
   - Detects obstacles before forward movement
   - Automatic backup if blocked
   - Continues scanning even when path is blocked

## How It Works

### Exploration Pattern

```
Start Position → Waypoint 1 → Waypoint 2 → Waypoint 3 → Waypoint 4
                     ↓            ↓            ↓            ↓
                 4 Rotations  4 Rotations  4 Rotations  4 Rotations
                 (90° each)   (90° each)   (90° each)   (90° each)
                     ↓            ↓            ↓            ↓
                12 Camera    12 Camera    12 Camera    12 Camera
                 Angles       Angles       Angles       Angles
```

### Camera Scan Pattern (at each rotation)

```
Tilt Angles:      Pan Angles:
    50° (high)  ×  -45° (left), 0° (center), 45° (right)
    25° (mid-high) × -45°, 0°, 45°
     0° (mid)   ×  -45°, 0°, 45°
   -15° (low)   ×  -45°, 0°, 45°
```

## Running the System

### Option 1: Standalone Mode (Recommended - No Docker Issues)

```bash
cd /Users/madhavayyagari/ros2-picarx
python3 room_scan_standalone.py
```

**Advantages:**
- No Docker setup required
- Direct hardware access
- Simpler troubleshooting
- Faster execution

### Option 2: ROS2 Mode

```bash
cd /Users/madhavayyagari/ros2-picarx
./run_ros2_room_scan.sh
```

**Docker Issues?** The standalone version avoids all Docker-related problems with GPIO and camera access.

## Configuration

### Standalone Version

Edit `room_scan_standalone.py`, lines 375-384:

```python
explorer = RoomExplorer(
    num_waypoints=4,              # How many forward positions to explore
    rotations_per_waypoint=4,     # Rotations at each position (4 = 90° each)
    pause_duration=1.5,           # Detection time per camera angle
    scan_speed=20,                # Robot movement speed
    turn_angle=30,                # Steering angle for turns
    confidence_threshold=0.6,     # YOLO detection confidence (0-1)
    obstacle_distance=30,         # Stop distance in cm
    forward_distance_time=2.0     # How long to move forward (seconds)
)
```

### ROS2 Version

Modify parameters in launch file: `src/my_first_pkg/launch/room_scan.launch.py`

Or pass parameters when launching:

```bash
ros2 launch my_first_pkg room_scan.launch.py \
    scan_positions:=4 \
    pause_duration:=1.5 \
    camera_tilt:=35.0
```

## What You'll See

### During Exploration

```
======================================================================
WAYPOINT 1/4
======================================================================
Moving forward to waypoint 1...

--- Rotation 1/4 at Waypoint 1 ---
  📷 Scan 1: Camera high/left (tilt=50°, pan=-45°)
      ✓ Found: chair(3), table(2), book(1)
  📷 Scan 2: Camera high/center (tilt=50°, pan=0°)
      - No objects detected
  ...
```

### Final Report

```
======================================================================
MISSION COMPLETE: Room Exploration Finished
======================================================================

📊 EXPLORATION SUMMARY:
  Waypoints visited: 4
  Total scan positions: 192
  🛡️  Obstacles detected: 2
  ⬅️  Backup maneuvers: 1
  Unique objects found: 8
  Total detections: 127

🎯 OBJECTS DETECTED:
--------------------------------------------------
1. CHAIR: 45 detections
2. TABLE: 32 detections
3. LAPTOP: 18 detections
4. BOOK: 15 detections
5. CUP: 8 detections

📍 SPATIAL DISTRIBUTION:
--------------------------------------------------

CHAIR:
  Waypoint 1: 12 detections - directions: high/center, high/left, mid/center
  Waypoint 2: 18 detections - directions: high/right, mid-high/center, mid/left
  Waypoint 3: 15 detections - directions: mid-high/left, mid/center

TABLE:
  Waypoint 1: 15 detections - directions: mid/center, mid/right
  Waypoint 2: 17 detections - directions: mid-high/center, mid/left
```

## Customization Tips

### For Smaller Rooms
```python
explorer = RoomExplorer(
    num_waypoints=2,              # Only 2 forward positions
    forward_distance_time=1.0     # Shorter forward distance
)
```

### For Faster Scanning
```python
explorer = RoomExplorer(
    pause_duration=0.8,           # Faster detection
    camera_scan_pause=0.5         # Faster camera movement
)
```

### For More Thorough Scanning
```python
explorer = RoomExplorer(
    num_waypoints=6,              # More positions
    pause_duration=2.0            # Longer detection time
)
```

## Troubleshooting

### "Failed to open camera /dev/video10"
- Check camera is connected: `ls -l /dev/video*`
- Try different device: Edit line 52 to use `/dev/video0` or `/dev/video1`

### Robot not moving
- Check battery level
- Verify picarx library is installed: `pip3 show picarx`
- Run hardware test: `./test_hardware.sh`

### "Obstacle detected" immediately
- Check ultrasonic sensor is working
- Adjust `obstacle_distance` parameter (increase for more sensitive)
- Clear area in front of robot

### Docker GPIO issues (ROS2 mode)
- **Best solution**: Use standalone mode instead
- Alternative: Run with `--privileged` flag and proper device mappings

## Performance

- **Standalone mode**: ~8-10 minutes for full exploration (192 positions)
- **ROS2 mode**: ~10-12 minutes (includes ROS2 overhead)
- **Battery life**: Recommend full charge for complete exploration

## Next Steps

1. **Try it**: Run `python3 room_scan_standalone.py`
2. **Adjust settings**: Modify parameters based on your room size
3. **Analyze results**: Check spatial distribution to understand room layout
4. **Experiment**: Try different camera angles or movement patterns

Enjoy your comprehensive room exploration! 🤖✨
