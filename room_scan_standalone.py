#!/usr/bin/env python3
"""
Standalone Room Scanning Mission - No ROS2, No Docker
Direct hardware control with YOLO object detection and obstacle avoidance
"""

import time
import cv2
from collections import defaultdict
from picarx import Picarx
from ultralytics import YOLO

class RoomScanner:
    def __init__(self, scan_positions=8, pause_duration=2.0, scan_speed=20,
                 turn_angle=30, confidence_threshold=0.6,
                 obstacle_distance=30):  # Stop if obstacle within 30cm
        self.scan_positions = scan_positions
        self.pause_duration = pause_duration
        self.scan_speed = scan_speed
        self.turn_angle = turn_angle
        self.confidence_threshold = confidence_threshold
        self.obstacle_distance = obstacle_distance  # cm

        self.objects_found = defaultdict(int)
        self.obstacles_detected = 0
        self.backup_count = 0

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
        print(f"🛡️  Obstacle detection enabled (threshold: {self.obstacle_distance}cm)")

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

    def scan_current_position(self, position):
        """Scan for objects at current position"""
        # Check distance to nearby objects
        _, distance = self.check_obstacle()
        print(f"  📏 Distance to nearest object: {distance:.1f}cm")

        print(f"  ⏸️  Pausing for {self.pause_duration}s to detect objects...")

        # Collect detections during pause
        position_objects = defaultdict(int)
        start_time = time.time()

        while time.time() - start_time < self.pause_duration:
            detections = self.detect_objects()
            for det in detections:
                position_objects[det['class_name']] += 1
                self.objects_found[det['class_name']] += 1
            time.sleep(0.2)  # Sample rate

        # Report findings for this position
        if position_objects:
            print(f"  ✅ Detected at position {position + 1}:")
            for obj_name, count in sorted(position_objects.items()):
                print(f"     - {obj_name}: {count} detections")
        else:
            print(f"  ⚪ No objects detected at this position")

    def run_mission(self):
        """Execute the complete room scanning mission"""
        print("\n" + "=" * 60)
        print("🎯 MISSION START: Room Scanning Initiated")
        print("=" * 60)
        print(f"Configuration: {self.scan_positions} positions, "
              f"{self.pause_duration}s pause")
        print(f"🛡️  Obstacle avoidance enabled (stops at {self.obstacle_distance}cm)")
        print("")

        try:
            for position in range(self.scan_positions):
                print(f"\n--- 📍 Scan Position {position + 1}/{self.scan_positions} ---")

                if position > 0:
                    print(f"  🔄 Rotating to position {position + 1}...")
                    self.rotate_to_next_position()
                else:
                    print(f"  📍 Starting at position 1...")

                # Scan current position
                self.scan_current_position(position)

            # Stop robot
            self.px.stop()
            self.px.set_dir_servo_angle(0)

            # Generate report
            self.print_report()

        except KeyboardInterrupt:
            print("\n\n⚠️  Mission interrupted by user!")
            self.px.stop()
            self.px.set_dir_servo_angle(0)
        except Exception as e:
            print(f"\n\n❌ Error during mission: {e}")
            import traceback
            traceback.print_exc()
            self.px.stop()
            self.px.set_dir_servo_angle(0)
        finally:
            self.cleanup()

    def print_report(self):
        """Print final mission report"""
        print("\n\n" + "=" * 60)
        print("🏁 MISSION COMPLETE: Room Scan Finished")
        print("=" * 60)

        # Mission statistics
        print(f"\n📊 Mission Statistics:")
        print(f"  🛡️  Obstacles detected: {self.obstacles_detected}")
        print(f"  ⬅️  Backup maneuvers: {self.backup_count}")

        if not self.objects_found:
            print("\n⚪ No objects detected during scan.")
            return

        print("\n🎯 OBJECTS FOUND IN ROOM:")
        print("-" * 40)

        # Sort by count (most common first)
        sorted_objects = sorted(self.objects_found.items(),
                               key=lambda x: x[1], reverse=True)

        for i, (obj_name, count) in enumerate(sorted_objects, 1):
            print(f"{i}. {obj_name.upper()}: {count} detections")

        print("-" * 40)
        print(f"Total unique objects: {len(self.objects_found)}")
        print(f"Total detections: {sum(self.objects_found.values())}")
        print("=" * 60)

    def cleanup(self):
        """Clean up resources"""
        if self.cap:
            self.cap.release()
        print("\n✅ Resources cleaned up")


def main():
    print("🤖 Room Scanning Mission - Standalone Mode")
    print("=" * 60)
    print("✨ NEW: Obstacle avoidance enabled!")
    print("   Robot will detect walls and back away automatically")
    print("")
    print("⚠️  IMPORTANT: Make sure robot has space to move")
    print("   Press Ctrl+C at any time to stop")
    print("=" * 60)

    # Give user time to prepare
    print("\nStarting in 3 seconds...")
    time.sleep(3)

    # Create scanner and run mission
    scanner = RoomScanner(
        scan_positions=8,
        pause_duration=2.0,
        scan_speed=20,
        turn_angle=30,
        confidence_threshold=0.6,
        obstacle_distance=30  # Stop if obstacle within 30cm
    )

    scanner.run_mission()


if __name__ == "__main__":
    main()
