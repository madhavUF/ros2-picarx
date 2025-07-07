#!/usr/bin/env python3
"""
YOLOv8 Robot Controller - Headless version for SSH
"""

import cv2
import time
import numpy as np
from ultralytics import YOLO

# Try to import robot control
try:
    from picarx import Picarx
    ROBOT_AVAILABLE = True
    print("✅ PiCar-X library imported")
except ImportError:
    ROBOT_AVAILABLE = False
    print("⚠️ PiCar-X library not available - running in simulation mode")

class YOLORobotHeadless:
    def __init__(self):
        print("🤖 Initializing Headless YOLO Robot...")
        
        # Load YOLO
        self.model = YOLO('yolov8n.pt')
        print("✅ YOLO model loaded")
        
        # Initialize robot
        if ROBOT_AVAILABLE:
            try:
                self.robot = Picarx()
                self.robot_enabled = True
                print("✅ PiCar-X initialized")
            except Exception as e:
                print(f"⚠️ Robot init failed: {e}")
                self.robot_enabled = False
        else:
            self.robot_enabled = False
        
        # Initialize camera
        self.cap = cv2.VideoCapture('/dev/video10', cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise Exception("❌ Cannot connect to camera")
        print("✅ Camera connected")
        
        # Settings
        self.following_enabled = True
        self.base_speed = 30
        self.turn_angle = 20
        self.last_person_time = 0
        self.scan_timeout = 3.0  # seconds
        
    def move_robot(self, action, speed=None):
        """Control robot movement"""
        if not self.robot_enabled:
            print(f"🎮 SIMULATION: {action} (speed: {speed})")
            return
        
        try:
            if action == "forward":
                self.robot.forward(speed or self.base_speed)
                print(f"🚗 Moving forward (speed: {speed or self.base_speed})")
            elif action == "turn_left":
                self.robot.set_dir_servo_angle(-self.turn_angle)
                self.robot.forward(speed or self.base_speed)
                print(f"↰ Turning left")
            elif action == "turn_right":
                self.robot.set_dir_servo_angle(self.turn_angle)
                self.robot.forward(speed or self.base_speed)
                print(f"↱ Turning right")
            elif action == "stop":
                self.robot.stop()
                self.robot.set_dir_servo_angle(0)
                print(f"🛑 Stopping")
            elif action == "scan_left":
                self.robot.set_dir_servo_angle(-self.turn_angle)
                self.robot.forward(15)  # Slow scanning
                print(f"🔍 Scanning left")
        except Exception as e:
            print(f"⚠️ Robot error: {e}")
    
    def process_detections(self, detections):
        """Decide robot behavior based on detections"""
        # Filter for people with good confidence
        persons = [d for d in detections if d['class'] == 'person' and d['confidence'] > 0.6]
        
        if not persons:
            # No person detected
            time_since_person = time.time() - self.last_person_time
            if time_since_person > self.scan_timeout:
                self.move_robot("scan_left")
                return "scanning_for_person"
            else:
                self.move_robot("stop")
                return "waiting_for_person"
        
        # Person detected!
        self.last_person_time = time.time()
        
        # Follow the most confident person
        target = max(persons, key=lambda p: p['confidence'])
        center_x = target['center'][0]
        frame_center = 320  # Assuming 640px width
        area = target['area']
        
        print(f"👤 Person detected! Confidence: {target['confidence']:.2f}, Area: {area}")
        
        # Following logic based on person position and size
        x_offset = center_x - frame_center
        
        if area > 20000:  # Person is very close
            self.move_robot("stop")
            return "person_too_close"
        elif abs(x_offset) < 80:  # Person is roughly centered
            if area < 8000:  # Person is far
                self.move_robot("forward", 25)
                return "following_forward"
            else:  # Person is at good distance
                self.move_robot("stop")
                return "maintaining_distance"
        elif x_offset > 80:  # Person is to the right
            self.move_robot("turn_right", 20)
            return "turning_to_follow_right"
        else:  # Person is to the left
            self.move_robot("turn_left", 20)
            return "turning_to_follow_left"
    
    def run(self):
        """Main control loop"""
        print("🚀 Starting autonomous person-following robot...")
        print("🎯 Robot will follow detected persons")
        print("📋 Behaviors:")
        print("   - Follow person when detected")
        print("   - Maintain safe distance")
        print("   - Scan for person when none detected")
        print("   - Stop when person too close")
        print("\nPress Ctrl+C to stop")
        
        frame_count = 0
        self.last_person_time = time.time()
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    continue
                
                frame_count += 1
                
                # Run YOLO detection
                results = self.model(frame, verbose=False)
                
                # Process detections
                detections = []
                if results[0].boxes is not None:
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    confs = results[0].boxes.conf.cpu().numpy()
                    classes = results[0].boxes.cls.cpu().numpy()
                    
                    for box, conf, cls in zip(boxes, confs, classes):
                        x1, y1, x2, y2 = map(int, box)
                        detections.append({
                            'class': self.model.names[int(cls)],
                            'confidence': float(conf),
                            'center': ((x1 + x2) // 2, (y1 + y2) // 2),
                            'area': (x2 - x1) * (y2 - y1)
                        })
                
                # Control robot based on detections
                behavior = self.process_detections(detections)
                
                # Print status every 20 frames
                if frame_count % 20 == 0:
                    detection_summary = {}
                    for det in detections:
                        cls = det['class']
                        detection_summary[cls] = detection_summary.get(cls, 0) + 1
                    
                    print(f"\n📊 Frame {frame_count}:")
                    print(f"   Detections: {detection_summary}")
                    print(f"   Current Behavior: {behavior}")
                    print(f"   Robot Status: {'ENABLED' if self.robot_enabled else 'SIMULATION'}")
                
                # Save frames with person detections
                if any(d['class'] == 'person' for d in detections) and frame_count % 50 == 0:
                    annotated = results[0].plot()
                    cv2.imwrite(f"robot_following_{frame_count}.jpg", annotated)
                    print(f"📸 Saved robot_following_{frame_count}.jpg")
                
                time.sleep(0.2)  # 5 FPS for robot control
                
        except KeyboardInterrupt:
            print("\n🛑 Emergency stop! Stopping robot...")
        
        finally:
            if self.robot_enabled:
                self.robot.stop()
                self.robot.set_dir_servo_angle(0)
            self.cap.release()
            print("✅ Robot stopped safely")
            print("🎯 Person-following session complete!")

def main():
    try:
        robot = YOLORobotHeadless()
        robot.run()
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
