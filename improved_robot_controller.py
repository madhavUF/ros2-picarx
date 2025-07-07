#!/usr/bin/env python3
"""
Improved YOLOv8 Robot Controller with Camera Movement and Better Detection
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

class ImprovedYOLORobot:
    def __init__(self):
        print("🤖 Initializing Improved YOLO Robot...")
        
        # Load YOLO
        self.model = YOLO('yolov8n.pt')
        print("✅ YOLO model loaded")
        
        # Initialize robot
        if ROBOT_AVAILABLE:
            try:
                self.robot = Picarx()
                self.robot_enabled = True
                print("✅ PiCar-X initialized")
                
                # Initialize camera servo positions
                self.robot.set_cam_pan(0)   # Center horizontally
                self.robot.set_cam_tilt(30) # Look up slightly to see people
                time.sleep(1)  # Give servos time to move
                print("📷 Camera positioned for person detection")
                
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
        
        # Improved settings
        self.base_speed = 25  # Slower for better control
        self.turn_angle = 15  # Smaller turns
        self.last_person_time = 0
        self.scan_timeout = 2.0  # Shorter timeout
        
        # Camera scanning parameters
        self.cam_pan_angle = 0    # Current camera pan (-90 to 90)
        self.cam_tilt_angle = 30  # Current camera tilt 
        self.scan_direction = 1   # 1 for right, -1 for left
        self.scan_step = 20       # Degrees to move camera each scan
        
        # Better detection thresholds
        self.area_thresholds = {
            'too_close': 150000,    # Much higher threshold
            'good_distance': 50000,  # Good following distance
            'too_far': 15000        # When to move forward
        }
        
        # Person tracking
        self.person_lost_count = 0
        self.max_lost_frames = 10  # How many frames before starting camera scan

    def move_camera(self, pan=None, tilt=None):
        """Move camera servo positions"""
        if not self.robot_enabled:
            if pan is not None:
                print(f"📷 SIM: Camera pan to {pan}°")
            if tilt is not None:
                print(f"📷 SIM: Camera tilt to {tilt}°")
            return
        
        try:
            if pan is not None:
                # Clamp pan angle
                pan = max(-90, min(90, pan))
                self.robot.set_cam_pan(pan)
                self.cam_pan_angle = pan
                
            if tilt is not None:
                # Clamp tilt angle (adjust range based on your setup)
                tilt = max(-30, min(90, tilt))
                self.robot.set_cam_tilt(tilt)
                self.cam_tilt_angle = tilt
                
            time.sleep(0.3)  # Give servo time to move
            
        except Exception as e:
            print(f"⚠️ Camera movement error: {e}")

    def scan_for_person(self):
        """Scan camera to look for person"""
        # Move camera in scanning pattern
        new_pan = self.cam_pan_angle + (self.scan_step * self.scan_direction)
        
        # Check if we hit the limits
        if new_pan >= 90:
            self.scan_direction = -1
            new_pan = 90
            # Also try looking up/down when hitting side limits
            if self.cam_tilt_angle < 60:
                self.move_camera(tilt=self.cam_tilt_angle + 20)
        elif new_pan <= -90:
            self.scan_direction = 1
            new_pan = -90
            # Reset tilt when completing full scan
            if self.cam_tilt_angle > 30:
                self.move_camera(tilt=30)
        
        self.move_camera(pan=new_pan)
        print(f"🔍 Scanning: Camera at pan={self.cam_pan_angle}°, tilt={self.cam_tilt_angle}°")

    def move_robot(self, action, speed=None):
        """Control robot movement"""
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
                self.robot.set_dir_servo_angle(0)
                print(f"🛑 Stopping")
            elif action == "backup":
                self.robot.backward(15)
                print(f"↩️ Backing up")
        except Exception as e:
            print(f"⚠️ Robot movement error: {e}")

    def track_person_with_camera(self, person_center_x):
        """Move camera to track person"""
        frame_center = 320
        x_offset = person_center_x - frame_center
        
        # If person is significantly off-center, pan camera to follow
        if abs(x_offset) > 100:
            pan_adjustment = x_offset // 8  # Proportional control
            new_pan = self.cam_pan_angle + pan_adjustment
            new_pan = max(-90, min(90, new_pan))
            
            if abs(new_pan - self.cam_pan_angle) > 5:  # Only move if significant change
                self.move_camera(pan=new_pan)
                print(f"📷 Tracking person: pan adjusted to {self.cam_pan_angle}°")

    def process_detections(self, detections):
        """Enhanced detection processing with camera control"""
        # Filter for people with good confidence
        persons = [d for d in detections if d['class'] == 'person' and d['confidence'] > 0.5]
        
        if not persons:
            self.person_lost_count += 1
            
            if self.person_lost_count > self.max_lost_frames:
                # Start scanning with camera
                self.scan_for_person()
                return "scanning_with_camera"
            else:
                # Person just disappeared, wait a moment
                self.move_robot("stop")
                return f"person_lost_{self.person_lost_count}"
        
        # Person detected!
        self.person_lost_count = 0
        self.last_person_time = time.time()
        
        # Find the best person to follow (highest confidence + reasonable size)
        valid_persons = [p for p in persons if p['area'] > 5000]  # Filter out tiny detections
        
        if not valid_persons:
            return "person_too_small"
        
        target = max(valid_persons, key=lambda p: p['confidence'])
        center_x, center_y = target['center']
        area = target['area']
        
        print(f"👤 Person: conf={target['confidence']:.2f}, area={area}, pos=({center_x},{center_y})")
        
        # Track person with camera
        self.track_person_with_camera(center_x)
        
        # Robot movement logic with improved thresholds
        frame_center_x = 320
        x_offset = center_x - frame_center_x
        
        if area > self.area_thresholds['too_close']:
            self.move_robot("backup")
            return "person_too_close_backing_up"
        elif area > self.area_thresholds['good_distance']:
            # Person is at good distance, just track
            if abs(x_offset) > 100:
                if x_offset > 0:
                    self.move_robot("turn_right", 15)
                    return "adjusting_right"
                else:
                    self.move_robot("turn_left", 15)
                    return "adjusting_left"
            else:
                self.move_robot("stop")
                return "maintaining_good_distance"
        elif area > self.area_thresholds['too_far']:
            # Person is at medium distance
            if abs(x_offset) > 120:
                # Need to turn to center person
                if x_offset > 0:
                    self.move_robot("turn_right", 20)
                    return "turning_to_center_right"
                else:
                    self.move_robot("turn_left", 20)
                    return "turning_to_center_left"
            else:
                # Move forward slowly
                self.move_robot("forward", 20)
                return "approaching_person"
        else:
            # Person is far, move forward
            if abs(x_offset) > 100:
                if x_offset > 0:
                    self.move_robot("turn_right", 25)
                    return "turning_toward_distant_person_right"
                else:
                    self.move_robot("turn_left", 25)
                    return "turning_toward_distant_person_left"
            else:
                self.move_robot("forward", 30)
                return "moving_toward_distant_person"

    def run(self):
        """Main control loop"""
        print("🚀 Starting Improved Person-Following Robot...")
        print("📷 Features:")
        print("   - Dynamic camera scanning")
        print("   - Person tracking with camera")
        print("   - Improved distance thresholds")
        print("   - Better turning logic")
        print("\n🎯 Robot will follow and track persons")
        print("Press Ctrl+C to stop\n")
        
        frame_count = 0
        self.last_person_time = time.time()
        
        # Start with camera in good position
        if self.robot_enabled:
            self.move_camera(pan=0, tilt=30)
        
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
                
                # Print status every 15 frames
                if frame_count % 15 == 0:
                    person_count = len([d for d in detections if d['class'] == 'person'])
                    print(f"\n📊 Frame {frame_count}:")
                    print(f"   Persons detected: {person_count}")
                    print(f"   Behavior: {behavior}")
                    print(f"   Camera: pan={self.cam_pan_angle}°, tilt={self.cam_tilt_angle}°")
                    print(f"   Lost count: {self.person_lost_count}")
                
                # Save interesting frames
                if person_count > 0 and frame_count % 40 == 0:
                    annotated = results[0].plot()
                    cv2.imwrite(f"improved_tracking_{frame_count}.jpg", annotated)
                    print(f"📸 Saved improved_tracking_{frame_count}.jpg")
                
                time.sleep(0.15)  # ~7 FPS for good control
                
        except KeyboardInterrupt:
            print("\n🛑 Emergency stop! Stopping robot...")
        
        finally:
            if self.robot_enabled:
                self.robot.stop()
                self.robot.set_dir_servo_angle(0)
                self.move_camera(pan=0, tilt=30)  # Reset camera position
            self.cap.release()
            print("✅ Robot stopped safely")
            print("🎯 Improved tracking session complete!")

def main():
    try:
        robot = ImprovedYOLORobot()
        robot.run()
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
