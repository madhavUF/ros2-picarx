#!/usr/bin/env python3
"""
Advanced Room Exploration - Standalone Mode
No ROS2, No Docker - Direct hardware control with YOLO and comprehensive exploration
"""

import time
import cv2
from collections import defaultdict
from picarx import Picarx
from ultralytics import YOLO

class RoomExplorer:
    def __init__(self, num_waypoints=4, rotations_per_waypoint=4, pause_duration=1.5,
                 scan_speed=20, turn_angle=30, confidence_threshold=0.6,
                 obstacle_distance=30, forward_distance_time=2.0):
        # Exploration parameters
        self.num_waypoints = num_waypoints
        self.rotations_per_waypoint = rotations_per_waypoint
        self.pause_duration = pause_duration
        self.scan_speed = scan_speed
        self.turn_angle = turn_angle
        self.confidence_threshold = confidence_threshold
        self.obstacle_distance = obstacle_distance
        self.forward_distance_time = forward_distance_time

        # Camera scan angles
        self.camera_tilt_angles = [50.0, 25.0, 0.0, -15.0]  # High, mid-high, mid, low
        self.camera_pan_angles = [-45.0, 0.0, 45.0]  # Left, center, right
        self.camera_scan_pause = 0.8

        # State tracking
        self.objects_found = defaultdict(int)
        self.spatial_map = []
        self.obstacles_detected = 0
        self.backup_count = 0
        self.current_waypoint = 0
        self.current_rotation = 0

        # Initialize robot
        print("🤖 Initializing robot...")
        self.px = Picarx()
        print("✅ Robot initialized")

        # Initialize YOLO
        print("🔍 Loading YOLO model...")
        self.model = YOLO('yolov8n.pt')
        print("✅ YOLO model loaded")

        # Initialize camera
        print("📷 Opening camera...")
        self.cap = cv2.VideoCapture('/dev/video10', cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera /dev/video10")

        # Let camera stabilize
        for _ in range(5):
            self.cap.read()
        print("✅ Camera ready")

        # Print configuration
        print(f"\n🔧 Configuration:")
        print(f"   Waypoints: {self.num_waypoints}")
        print(f"   Rotations per waypoint: {self.rotations_per_waypoint}")
        print(f"   Camera tilt angles: {len(self.camera_tilt_angles)}")
        print(f"   Camera pan angles: {len(self.camera_pan_angles)}")
        print(f"   Total scan positions: {self.num_waypoints * self.rotations_per_waypoint * len(self.camera_tilt_angles) * len(self.camera_pan_angles)}")
        print(f"   🛡️  Obstacle detection: {self.obstacle_distance}cm threshold")

    def check_obstacle(self):
        """Check if there's an obstacle ahead"""
        distance = self.px.get_distance()

        # Filter out invalid readings
        if distance < 0:
            return False, distance

        is_blocked = distance < self.obstacle_distance
        return is_blocked, distance

    def backup_from_obstacle(self):
        """Back away from obstacle"""
        print("  ⚠️  OBSTACLE DETECTED! Backing up...")
        self.obstacles_detected += 1

        # Stop
        self.px.stop()
        time.sleep(0.2)

        # Back up
        self.px.backward(self.scan_speed)
        time.sleep(1.0)

        # Stop
        self.px.stop()
        self.px.set_dir_servo_angle(0)
        time.sleep(0.2)

        print("  ✅ Backed away from obstacle")
        self.backup_count += 1

    def set_camera(self, pan_angle=0.0, tilt_angle=0.0):
        """Set camera servo angles"""
        try:
            self.px.set_cam_pan_angle(pan_angle)
            self.px.set_cam_tilt_angle(tilt_angle)
        except Exception as e:
            print(f"⚠️  Camera control error: {e}")

    def detect_objects(self):
        """Capture image and detect objects"""
        ret, frame = self.cap.read()
        if not ret:
            print("⚠️  Failed to capture frame")
            return []

        # Run YOLO inference
        results = self.model(frame, verbose=False)

        detections = []
        if len(results[0].boxes) > 0:
            boxes = results[0].boxes
            for box in boxes:
                conf = float(box.conf[0])
                if conf >= self.confidence_threshold:
                    cls = int(box.cls[0])
                    class_name = self.model.names[cls]
                    detections.append({
                        'class_name': class_name,
                        'confidence': conf
                    })

        return detections

    def rotate_to_next_position(self):
        """Rotate robot to next scanning position with obstacle detection"""
        # Check for obstacles before moving
        blocked, distance = self.check_obstacle()

        if blocked:
            print(f"  🛑 Obstacle at {distance:.1f}cm - adjusting position")
            self.backup_from_obstacle()
            # Try a different angle
            print("  🔄 Trying alternate rotation angle...")
            self.px.set_dir_servo_angle(self.turn_angle)  # Turn right instead
        else:
            # Normal left turn
            self.px.set_dir_servo_angle(-self.turn_angle)

        # Start moving
        self.px.forward(self.scan_speed)

        # Rotate with continuous obstacle checking
        rotation_time = 1.5
        start_time = time.time()

        while time.time() - start_time < rotation_time:
            # Check for obstacles while rotating
            blocked, distance = self.check_obstacle()
            if blocked:
                print(f"  ⚠️  Obstacle detected during rotation at {distance:.1f}cm!")
                self.px.stop()
                self.backup_from_obstacle()
                break
            time.sleep(0.1)

        # Stop and center wheels
        self.px.stop()
        self.px.set_dir_servo_angle(0)
        time.sleep(0.3)

    def perform_camera_scan(self, waypoint, rotation):
        """Perform comprehensive multi-angle camera scan"""
        scan_count = 0

        for tilt_angle in self.camera_tilt_angles:
            for pan_angle in self.camera_pan_angles:
                scan_count += 1

                # Describe camera direction
                tilt_desc = "high" if tilt_angle > 30 else "mid-high" if tilt_angle > 10 else "mid" if tilt_angle > -10 else "low"
                pan_desc = "left" if pan_angle < -10 else "right" if pan_angle > 10 else "center"

                print(f"  📷 Scan {scan_count}: Camera {tilt_desc}/{pan_desc} (tilt={tilt_angle:.0f}°, pan={pan_angle:.0f}°)")

                # Move camera
                self.set_camera(pan_angle=pan_angle, tilt_angle=tilt_angle)
                time.sleep(self.camera_scan_pause)

                # Collect detections
                position_objects = defaultdict(int)
                start_time = time.time()

                while time.time() - start_time < self.pause_duration:
                    detections = self.detect_objects()
                    for det in detections:
                        position_objects[det['class_name']] += 1
                        self.objects_found[det['class_name']] += 1

                        # Add to spatial map
                        self.spatial_map.append({
                            'object': det['class_name'],
                            'confidence': det['confidence'],
                            'waypoint': waypoint,
                            'rotation': rotation,
                            'camera_tilt': tilt_angle,
                            'camera_pan': pan_angle,
                            'direction': f'{tilt_desc}/{pan_desc}'
                        })

                    time.sleep(0.2)

                # Report findings
                if position_objects:
                    print(f"      ✓ Found: {', '.join([f'{name}({count})' for name, count in position_objects.items()])}")
                else:
                    print(f"      - No objects detected")

    def run_mission(self):
        """Execute comprehensive room exploration mission"""
        print("\n" + "=" * 70)
        print("🎯 MISSION START: Advanced Room Exploration Initiated")
        print("=" * 70)
        print(f"Strategy: Move to {self.num_waypoints} waypoints,")
        print(f"          {self.rotations_per_waypoint} rotations per waypoint")
        print(f"          {len(self.camera_tilt_angles)} tilt × {len(self.camera_pan_angles)} pan angles per rotation")
        print(f"🛡️  Obstacle avoidance enabled (threshold: {self.obstacle_distance}cm)")
        print("=" * 70)

        try:
            # Reset camera to center
            self.set_camera(pan_angle=0.0, tilt_angle=0.0)
            time.sleep(0.5)

            # Explore waypoints
            for waypoint in range(self.num_waypoints):
                self.current_waypoint = waypoint

                print(f"\n{'='*70}")
                print(f"WAYPOINT {waypoint + 1}/{self.num_waypoints}")
                print(f"{'='*70}")

                # Move forward to new position (except first waypoint)
                if waypoint > 0:
                    print(f"Moving forward to waypoint {waypoint + 1}...")

                    # Check for obstacles before moving forward
                    blocked, distance = self.check_obstacle()
                    if blocked:
                        print(f"  🛑 Obstacle at {distance:.1f}cm - cannot move forward")
                        print(f"  Continuing with rotational scan at current position")
                    else:
                        self.px.forward(self.scan_speed)
                        time.sleep(self.forward_distance_time)
                        self.px.stop()
                        time.sleep(0.3)

                # Perform rotational scans at this waypoint
                degrees_per_rotation = 360.0 / self.rotations_per_waypoint

                for rotation in range(self.rotations_per_waypoint):
                    self.current_rotation = rotation

                    print(f"\n--- Rotation {rotation + 1}/{self.rotations_per_waypoint} at Waypoint {waypoint + 1} ---")

                    # Rotate to next direction (except first rotation)
                    if rotation > 0:
                        print(f"  🔄 Rotating {degrees_per_rotation:.0f}°...")
                        self.rotate_to_next_position()

                    # Multi-angle camera scan at this rotation
                    self.perform_camera_scan(waypoint, rotation)

            # Mission complete
            self.px.stop()
            self.px.set_dir_servo_angle(0)
            self.set_camera(pan_angle=0.0, tilt_angle=0.0)

            # Generate report
            self.print_report()

        except KeyboardInterrupt:
            print("\n\n⚠️  Mission interrupted by user!")
            self.px.stop()
            self.px.set_dir_servo_angle(0)
            self.set_camera(pan_angle=0.0, tilt_angle=0.0)
        except Exception as e:
            print(f"\n\n❌ Error during mission: {e}")
            import traceback
            traceback.print_exc()
            self.px.stop()
            self.px.set_dir_servo_angle(0)
            self.set_camera(pan_angle=0.0, tilt_angle=0.0)
        finally:
            self.cleanup()

    def print_report(self):
        """Print comprehensive mission report with spatial information"""
        print("\n\n" + "=" * 70)
        print("🏁 MISSION COMPLETE: Room Exploration Finished")
        print("=" * 70)

        # Mission statistics
        print(f"\n📊 EXPLORATION SUMMARY:")
        print(f"  Waypoints visited: {self.num_waypoints}")
        print(f"  Total scan positions: {self.num_waypoints * self.rotations_per_waypoint * len(self.camera_tilt_angles) * len(self.camera_pan_angles)}")
        print(f"  🛡️  Obstacles detected: {self.obstacles_detected}")
        print(f"  ⬅️  Backup maneuvers: {self.backup_count}")
        print(f"  Unique objects found: {len(self.objects_found)}")
        print(f"  Total detections: {sum(self.objects_found.values())}")

        if not self.objects_found:
            print("\n⚪ No objects detected during exploration.")
            return

        # Object counts
        print(f"\n🎯 OBJECTS DETECTED:")
        print("-" * 50)

        sorted_objects = sorted(self.objects_found.items(),
                               key=lambda x: x[1], reverse=True)

        for i, (obj_name, count) in enumerate(sorted_objects, 1):
            print(f"{i}. {obj_name.upper()}: {count} detections")

        # Spatial distribution
        print(f"\n📍 SPATIAL DISTRIBUTION:")
        print("-" * 50)

        # Group by object type (top 5)
        for obj_name in sorted([obj for obj, _ in sorted_objects[:5]]):
            obj_locations = [loc for loc in self.spatial_map if loc['object'] == obj_name]

            print(f"\n{obj_name.upper()}:")

            # Group by waypoint
            waypoint_groups = defaultdict(list)
            for loc in obj_locations:
                waypoint_groups[loc['waypoint']].append(loc)

            for wp in sorted(waypoint_groups.keys()):
                locs = waypoint_groups[wp]
                directions = set([loc['direction'] for loc in locs])
                print(f"  Waypoint {wp + 1}: {len(locs)} detections - directions: {', '.join(sorted(directions))}")

        print("\n" + "=" * 70)

    def cleanup(self):
        """Clean up resources"""
        if self.cap:
            self.cap.release()
        print("\n✅ Resources cleaned up")


def main():
    print("🤖 Advanced Room Exploration - Standalone Mode")
    print("=" * 70)
    print("✨ FEATURES:")
    print("   • Multi-waypoint exploration - moves through the room")
    print("   • 360° scanning at each waypoint")
    print("   • Multi-angle camera scanning (high, mid, low)")
    print("   • Pan scanning (left, center, right)")
    print("   • Obstacle avoidance with automatic backup")
    print("   • Spatial mapping of detected objects")
    print("")
    print("⚠️  IMPORTANT: Make sure robot has space to move forward")
    print("   Press Ctrl+C at any time to stop")
    print("=" * 70)

    # Give user time to prepare
    print("\nStarting in 3 seconds...")
    time.sleep(3)

    # Create explorer and run mission
    explorer = RoomExplorer(
        num_waypoints=4,              # Move forward to 4 positions
        rotations_per_waypoint=4,     # 4 rotations (90° each) at each position
        pause_duration=1.5,           # 1.5s detection pause per camera angle
        scan_speed=20,
        turn_angle=30,
        confidence_threshold=0.6,
        obstacle_distance=30,         # Stop if obstacle within 30cm
        forward_distance_time=2.0     # Move forward for 2 seconds between waypoints
    )

    explorer.run_mission()


if __name__ == "__main__":
    main()
