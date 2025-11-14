#!/usr/bin/env python3
"""YOLO Detector for Intel RealSense Cameras"""

import argparse
import cv2
import time
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from collections import defaultdict


class RealSenseYOLODetector:
    def __init__(self, model_path='yolov8n.pt', confidence=0.5,
                 width=640, height=480, fps=30, camera_index=0):
        self.confidence = confidence

        # Initialize YOLO
        print(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        print("YOLO model loaded successfully")

        # Initialize RealSense pipeline
        print(f"Initializing RealSense camera {camera_index}...")
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get connected devices
        ctx = rs.context()
        devices = ctx.query_devices()

        if len(devices) == 0:
            raise RuntimeError("No RealSense devices found!")

        print(f"Found {len(devices)} RealSense device(s)")

        if camera_index >= len(devices):
            raise RuntimeError(f"Camera index {camera_index} not available. Only {len(devices)} cameras found.")

        # Use specific camera
        device = devices[camera_index]
        serial = device.get_info(rs.camera_info.serial_number)
        print(f"Using camera {camera_index}: {device.get_info(rs.camera_info.name)} (S/N: {serial})")

        self.config.enable_device(serial)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # Start pipeline
        profile = self.pipeline.start(self.config)

        # Skip first few frames for auto-exposure
        for _ in range(30):
            self.pipeline.wait_for_frames()

        print(f"RealSense camera initialized: {width}x{height} @ {fps}fps")

        # Statistics
        self.detection_counts = defaultdict(int)
        self.total_frames = 0
        self.start_time = time.time()

    def detect_frame(self):
        # Wait for frames
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            return False, [], None

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Run YOLO
        results = self.model(color_image, verbose=False)

        # Parse detections
        detections = []
        if len(results[0].boxes) > 0:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            confs = results[0].boxes.conf.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy()

            for box, conf, cls in zip(boxes, confs, classes):
                if conf >= self.confidence:
                    class_name = self.model.names[int(cls)]
                    detection = {
                        'class': class_name,
                        'confidence': float(conf),
                        'bbox': (float(box[0]), float(box[1]), float(box[2]), float(box[3])),
                    }
                    detections.append(detection)
                    self.detection_counts[class_name] += 1

        annotated_frame = results[0].plot()
        self.total_frames += 1

        return True, detections, annotated_frame

    def run_continuous(self, duration=None, save_video=None):
        print("\nStarting continuous detection...")
        print("Press Ctrl+C to stop\n")

        video_writer = None
        if save_video:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(save_video, fourcc, 30, (640, 480))
            print(f"Saving video to: {save_video}")

        last_print_time = time.time()

        try:
            while True:
                if duration and (time.time() - self.start_time) >= duration:
                    print(f"\nDuration limit reached ({duration}s)")
                    break

                success, detections, annotated_frame = self.detect_frame()
                if not success:
                    print("Failed to read frame")
                    break

                if video_writer:
                    video_writer.write(annotated_frame)

                if time.time() - last_print_time >= 2.0:
                    if detections:
                        print(f"Frame {self.total_frames}: {len(detections)} detections - " +
                              ", ".join([f"{d['class']} ({d['confidence']:.2f})" for d in detections]))
                    else:
                        print(f"Frame {self.total_frames}: No detections")
                    last_print_time = time.time()

        except KeyboardInterrupt:
            print("\n\nStopped by user")

        finally:
            if video_writer:
                video_writer.release()

        self.print_summary()

    def print_summary(self):
        elapsed = time.time() - self.start_time
        avg_fps = self.total_frames / elapsed if elapsed > 0 else 0

        print("\n" + "="*60)
        print("DETECTION SUMMARY")
        print("="*60)
        print(f"Duration: {elapsed:.1f} seconds")
        print(f"Total Frames: {self.total_frames}")
        print(f"Average FPS: {avg_fps:.1f}")
        print(f"\nObject Detections:")

        if self.detection_counts:
            for obj, count in sorted(self.detection_counts.items(), key=lambda x: x[1], reverse=True):
                print(f"  {obj}: {count}")
        else:
            print("  No objects detected")
        print("="*60 + "\n")

    def cleanup(self):
        self.pipeline.stop()
        print("RealSense pipeline stopped")


def main():
    parser = argparse.ArgumentParser(description='YOLO Object Detector for Intel RealSense Cameras')
    parser.add_argument('--camera', type=int, default=0, help='RealSense camera index (0, 1, 2...)')
    parser.add_argument('--model', default='yolov8n.pt', help='Path to YOLO model')
    parser.add_argument('--confidence', type=float, default=0.5, help='Confidence threshold 0-1')
    parser.add_argument('--width', type=int, default=640, help='Camera width')
    parser.add_argument('--height', type=int, default=480, help='Camera height')
    parser.add_argument('--fps', type=int, default=30, help='Camera FPS')
    parser.add_argument('--duration', type=float, help='Run for specified seconds')
    parser.add_argument('--save-video', help='Save annotated video to file')

    args = parser.parse_args()

    detector = RealSenseYOLODetector(
        model_path=args.model,
        confidence=args.confidence,
        width=args.width,
        height=args.height,
        fps=args.fps,
        camera_index=args.camera
    )

    try:
        detector.run_continuous(duration=args.duration, save_video=args.save_video)
    finally:
        detector.cleanup()


if __name__ == '__main__':
    main()
