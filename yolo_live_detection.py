#!/usr/bin/env python3
"""
YOLOv8 Live Detection with Pi Camera
"""

import cv2
import time
from ultralytics import YOLO

def main():
    print("🎯 Starting YOLOv8 Live Detection")
    print("Controls: 'q' to quit, 's' to save frame")
    
    # Load YOLO model
    model = YOLO('yolov8n.pt')
    print("✅ YOLO model loaded")
    
    # Connect to camera
    cap = cv2.VideoCapture('/dev/video10', cv2.CAP_V4L2)
    if not cap.isOpened():
        print("❌ Cannot connect to camera bridge")
        print("Make sure camera_bridge.sh is running!")
        return
    
    print("✅ Camera connected")
    
    # Performance tracking
    fps_count = 0
    fps_start = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ No frame received")
            continue
        
        # Run YOLO detection
        results = model(frame, verbose=False)
        
        # Draw detections
        annotated_frame = results[0].plot()
        
        # Add FPS counter
        fps_count += 1
        if time.time() - fps_start >= 1.0:
            fps = fps_count / (time.time() - fps_start)
            fps_count = 0
            fps_start = time.time()
        else:
            fps = 0
        
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Display (this might not work over SSH, but will save frames)
        cv2.imshow('YOLOv8 Live Detection', annotated_frame)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"detection_{int(time.time())}.jpg"
            cv2.imwrite(filename, annotated_frame)
            print(f"📸 Saved {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    print("🎯 Detection complete!")

if __name__ == "__main__":
    main()
