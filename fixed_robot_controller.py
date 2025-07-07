#!/usr/bin/env python3
"""
Fixed YOLOv8 Robot Controller - Works with actual PiCar-X hardware
"""

import cv2
import time
import numpy as np
from ultralytics import YOLO

try:
    from picarx import Picarx
    ROBOT_AVAILABLE = True
    print("✅ PiCar-X library imported")
except ImportError:
    ROBOT_AVAILABLE = False
    print("⚠️ PiCar-X library not available - running in simulation mode")

class FixedYOLORobot:
    def __init__(self):
        print("🤖 Initializing Fixed YOLO Robot...")
        
        # Load YOLO
        self.model = YOLO('yolov8n.pt')
        print("✅ YOLO model loaded")
        
        # Initialize robot
        if ROBOT_AVAILABLE:
            try:
                self.robot = Picarx()
                self.robot_enabled = True
                print("✅ PiCar-X initialized")
                
                # Check what camera methods are available
                print("🔍 Checking available camera methods...")
                methods = [method for method in dir(self.robot) if 'cam' in method.lower() or 'servo' in method.lower()]
                print(f"Available methods: {methods}")
                
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
        
        # Robot settings
        self.base_speed = 25
        self.turn_angle = 20
        self.last_person_time = 0
        self.scan_timeout = 3.0
        
        # Better detection thresholds (adjusted based on your previous results)
        self.area_thresholds = {
            'too_close': 200000,    # Based on your 240k readings
            'good_distance': 80000,  
            'too_far': 20000        
        }
        
        # Person tracking
        self.person_lost_count = 0
        self.max_lost_frames = 15
        self.scan_direction = 1

    def move_robot(self, action, speed=None):
        """Control robot movement with actual PiCar-X methods"""
        if not self.robot_enabled:
            print(f"🎮 SIM: {action} (speed: {speed})")
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
                self.robot.set_dir_servo_angle(0)  # Straighten wheels
                print(f"🛑 Stopping")
            elif action == "backup":
                self.robot.backward(15)
                print(f"↩️ Backing up")
            elif action == "scan_turn":
                # Scanning by turning the robot instead of camera
                angle = 25 * self.scan_direction
                self.robot.set_dir_servo_angle(angle)
                self.robot.forward(15)
                print(f"🔍 Scanning turn: {angle}°")
                
        except Exception as e:
            print(f"⚠️ Robot movement error: {e}")

    def scan_for_person(self):
        """Scan by turning the robot (since camera servos aren't available)"""
        # Switch scan direction occasionally
        if time.time() % 10 < 0.2:  # Change direction every ~10 seconds
            self.scan_direction *= -1
        
        self.move_robot("scan_turn")
        return "scanning_by_turning"

    def process_detections(self, detections):
        """Process detections and control robot"""
        # Filter for people
        persons = [d for d in detections if d['class'] == 'person' and d['confidence'] > 0.6]
        
        if not persons:
            self.person_lost_count += 1
            
            if self.person_lost_count > self.max_lost_frames:
                # Scan by turning robot
                return self.scan_for_person()
            else:
                self.move_robot("stop")
                return f"waiting_person_lost_{self.person_lost_count}"
        
        # Person found!
        self.person_lost_count = 0
        self.last_person_time = time.time()
        
        # Find best person to follow
        valid_persons = [p for p in persons if p['area'] > 8000]
        if not valid_persons:
            return "person_too_small"
        
        target = max(valid_persons, key=lambda p: p['confidence'])
        center_x, center_y = target['center']
        area = target['area']
        
        print(f"👤 Person: conf={target['confidence']:.2f}, area={area:,}, center=({center_x},{center_y})")
        
        # Decision logic based on your previous data
        frame_center = 320
        x_offset = center_x - frame_center
        
        if area > self.area_thresholds['too_close']:
            # You're too close (like your 240k readings)
            self.move_robot("backup")
            return "backing_up_person_close"
        elif area > self.area_thresholds['good_distance']:
            # Good distance, just adjust direction
            if abs(x_offset) > 80:
                if x_offset > 0:
                    self.move_robot("turn_right", 20)
                    return "fine_tune_right"
                else:
                    self.move_robot("turn_left", 20)
                    return "fine_tune_left"
            else:
                self.move_robot("stop")
                return "perfect_distance"
        else:
            # Person is far, approach
            if abs(x_offset) > 100:
                if x_offset > 0:
                    self.move_robot("turn_right", 25)
                    return "approach_turn_right"
                else:
                    self.move_robot("turn_left", 25)
                    return "approach_turn_left"
            else:
                self.move_robot("forward", 25)
                return "approaching"

    def run(self):
        """Main control loop"""
        print("🚀 Starting Fixed Person-Following Robot...")
        print("🎯 Improved logic based on your detection data")
        print("📊 Distance thresholds:")
        print(f"   Too close: >{self.area_thresholds['too_close']:,} pixels")
        print(f"   Good distance: {self.area_thresholds['good_distance']:,}-{self.area_thresholds['too_close']:,} pixels")
        print(f"   Too far: <{self.area_thresholds['good_distance']:,} pixels")
        print("\nPress Ctrl+C to stop\n")
        
        frame_count = 0
        behavior_history = []
        
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
                
                # Control robot
                behavior = self.process_detections(detections)
                behavior_history.append(behavior)
                
                # Keep only recent behaviors
                if len(behavior_history) > 20:
                    behavior_history.pop(0)
                
                # Count persons in current frame
                person_count = len([d for d in detections if d['class'] == 'person'])
                
                # Print status every 15 frames
                if frame_count % 15 == 0:
                    print(f"\n📊 Frame {frame_count}:")
                    print(f"   Persons detected: {person_count}")
                    print(f"   Current behavior: {behavior}")
                    print(f"   Recent behaviors: {behavior_history[-3:]}")
                    print(f"   Robot status: {'ENABLED' if self.robot_enabled else 'SIMULATION'}")
                
                # Save frames with good person detections
                if person_count > 0 and frame_count % 50 == 0:
                    annotated = results[0].plot()
                    cv2.imwrite(f"fixed_tracking_{frame_count}.jpg", annotated)
                    print(f"📸 Saved fixed_tracking_{frame_count}.jpg")
                
                time.sleep(0.2)  # 5 FPS
                
        except KeyboardInterrupt:
            print("\n🛑 Emergency stop!")
        
        finally:
            if self.robot_enabled:
                self.robot.stop()
                self.robot.set_dir_servo_angle(0)
            self.cap.release()
            print("✅ Robot stopped safely")

def main():
    try:
        robot = FixedYOLORobot()
        robot.run()
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
