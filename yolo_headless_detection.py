#!/usr/bin/env python3
"""
YOLOv8 Headless Detection - Works over SSH without display
"""

import cv2
import time
from ultralytics import YOLO

def main():
    print("🎯 Starting YOLOv8 Headless Detection")
    print("Running without display - perfect for SSH!")
    
    # Load YOLO model
    model = YOLO('yolov8n.pt')
    print("✅ YOLO model loaded")
    
    # Connect to camera
    cap = cv2.VideoCapture('/dev/video10', cv2.CAP_V4L2)
    if not cap.isOpened():
        print("❌ Cannot connect to camera bridge")
        return
    
    print("✅ Camera connected")
    print("📹 Starting detection loop... Press Ctrl+C to stop")
    
    frame_count = 0
    detection_summary = {}
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ No frame received")
                continue
            
            frame_count += 1
            
            # Run YOLO detection
            results = model(frame, verbose=False)
            
            # Process detections
            detections = []
            if results[0].boxes is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy()
                confs = results[0].boxes.conf.cpu().numpy()
                classes = results[0].boxes.cls.cpu().numpy()
                
                for box, conf, cls in zip(boxes, confs, classes):
                    x1, y1, x2, y2 = map(int, box)
                    class_name = model.names[int(cls)]
                    confidence = float(conf)
                    
                    detections.append({
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': (x1, y1, x2, y2),
                        'center': ((x1 + x2) // 2, (y1 + y2) // 2),
                        'area': (x2 - x1) * (y2 - y1)
                    })
                    
                    # Update summary
                    if class_name not in detection_summary:
                        detection_summary[class_name] = 0
                    detection_summary[class_name] += 1
            
            # Print results every frame
            if detections:
                print(f"📦 Frame {frame_count}: {len(detections)} objects")
                for i, det in enumerate(detections):
                    print(f"   {i+1}. {det['class']} (conf: {det['confidence']:.2f}, center: {det['center']})")
                
                # Save frame with detections
                annotated_frame = results[0].plot()
                filename = f"detection_frame_{frame_count}.jpg"
                cv2.imwrite(filename, annotated_frame)
                print(f"📸 Saved {filename}")
                
            else:
                print(f"📭 Frame {frame_count}: No objects detected")
            
            # Print summary every 20 frames
            if frame_count % 20 == 0:
                print(f"\n📊 Detection Summary (last {frame_count} frames):")
                for obj_class, count in detection_summary.items():
                    print(f"   {obj_class}: {count} detections")
                print()
            
            time.sleep(0.5)  # Slow down for readability
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping detection...")
    
    finally:
        cap.release()
        print("🎯 Detection complete!")
        print(f"📊 Final Summary - Processed {frame_count} frames:")
        for obj_class, count in detection_summary.items():
            print(f"   {obj_class}: {count} total detections")

if __name__ == "__main__":
    main()
