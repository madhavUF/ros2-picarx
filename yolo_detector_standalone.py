#!/usr/bin/env python3
"""
Standalone YOLO Object Detector
No ROS dependencies - just Python, OpenCV, and YOLO

Usage:
    python3 yolo_detector_standalone.py [options]

Options:
    --camera DEVICE     Camera device (default: /dev/video0 or 0)
    --model PATH        Path to YOLO model (default: yolov8n.pt)
    --confidence FLOAT  Confidence threshold (default: 0.5)
    --display           Show video with detections (default: False)
    --save-video PATH   Save annotated video to file
    --fps INT           Target FPS (default: 10)
"""

import argparse
import cv2
import time
from ultralytics import YOLO
from collections import defaultdict
from datetime import datetime


class YOLODetector:
    def __init__(self, model_path='yolov8n.pt', camera_device=0,
                 confidence=0.5, display=False, save_video=None, fps=10):
        """
        Initialize YOLO detector

        Args:
            model_path: Path to YOLO model file
            camera_device: Camera device (int or string path like '/dev/video0')
            confidence: Minimum confidence threshold for detections
            display: Whether to display video window
            save_video: Path to save annotated video (optional)
            fps: Target frames per second
        """
        self.confidence = confidence
        self.display = display
        self.save_video = save_video
        self.target_fps = fps
        self.frame_time = 1.0 / fps

        # Initialize YOLO model
        print(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        print("YOLO model loaded successfully")

        # Initialize camera
        print(f"Opening camera: {camera_device}")
        # Try to parse as integer (for /dev/video0 -> 0)
        try:
            if isinstance(camera_device, str) and camera_device.startswith('/dev/video'):
                camera_device = int(camera_device.replace('/dev/video', ''))
        except:
            pass

        self.cap = cv2.VideoCapture(camera_device)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera: {camera_device}")

        # Get camera properties
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera opened: {self.frame_width}x{self.frame_height}")

        # Read a few dummy frames to let camera stabilize
        for _ in range(5):
            self.cap.read()

        # Initialize video writer if saving
        self.video_writer = None
        if save_video:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(
                save_video, fourcc, fps, (self.frame_width, self.frame_height))
            print(f"Saving video to: {save_video}")

        # Statistics
        self.detection_counts = defaultdict(int)
        self.total_frames = 0
        self.start_time = time.time()

    def detect_frame(self):
        """
        Capture and process one frame

        Returns:
            tuple: (success, detections_list)
            detections_list format: [{'class': 'person', 'confidence': 0.95, 'bbox': (x1,y1,x2,y2)}, ...]
        """
        ret, frame = self.cap.read()
        if not ret:
            return False, []

        # Run YOLO inference
        results = self.model(frame, verbose=False)

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
                        'center': ((float(box[0]) + float(box[2])) / 2,
                                  (float(box[1]) + float(box[3])) / 2),
                        'area': (float(box[2]) - float(box[0])) * (float(box[3]) - float(box[1]))
                    }
                    detections.append(detection)
                    self.detection_counts[class_name] += 1

        # Get annotated frame
        annotated_frame = results[0].plot()

        # Display if requested
        if self.display:
            cv2.imshow('YOLO Detection', annotated_frame)
            cv2.waitKey(1)

        # Save to video if requested
        if self.video_writer:
            self.video_writer.write(annotated_frame)

        self.total_frames += 1

        return True, detections

    def run_continuous(self, duration=None, max_frames=None):
        """
        Run continuous detection

        Args:
            duration: Run for this many seconds (None = infinite)
            max_frames: Process this many frames (None = infinite)
        """
        print("\nStarting continuous detection...")
        print("Press Ctrl+C to stop\n")

        frame_count = 0
        last_print_time = time.time()

        try:
            while True:
                loop_start = time.time()

                # Check stopping conditions
                if duration and (time.time() - self.start_time) >= duration:
                    print(f"\nDuration limit reached ({duration}s)")
                    break
                if max_frames and frame_count >= max_frames:
                    print(f"\nFrame limit reached ({max_frames} frames)")
                    break

                # Process frame
                success, detections = self.detect_frame()
                if not success:
                    print("Failed to read frame")
                    break

                frame_count += 1

                # Print detections periodically (every 2 seconds)
                if time.time() - last_print_time >= 2.0:
                    if detections:
                        print(f"Frame {frame_count}: {len(detections)} detections - " +
                              ", ".join([f"{d['class']} ({d['confidence']:.2f})" for d in detections]))
                    else:
                        print(f"Frame {frame_count}: No detections")
                    last_print_time = time.time()

                # Maintain target FPS
                elapsed = time.time() - loop_start
                sleep_time = self.frame_time - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\nStopped by user")

        self.print_summary()

    def print_summary(self):
        """Print detection summary statistics"""
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
            for obj, count in sorted(self.detection_counts.items(),
                                     key=lambda x: x[1], reverse=True):
                print(f"  {obj}: {count}")
        else:
            print("  No objects detected")
        print("="*60 + "\n")

    def cleanup(self):
        """Release resources"""
        if self.cap:
            self.cap.release()
        if self.video_writer:
            self.video_writer.release()
        if self.display:
            cv2.destroyAllWindows()
        print("Resources released")


def main():
    parser = argparse.ArgumentParser(
        description='Standalone YOLO Object Detector',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage with default webcam
  python3 yolo_detector_standalone.py

  # Use specific camera device
  python3 yolo_detector_standalone.py --camera /dev/video0

  # Show live video with detections
  python3 yolo_detector_standalone.py --display

  # Save annotated video
  python3 yolo_detector_standalone.py --save-video output.mp4

  # Higher confidence threshold
  python3 yolo_detector_standalone.py --confidence 0.7

  # Run for 60 seconds
  python3 yolo_detector_standalone.py --duration 60
        """)

    parser.add_argument('--camera', default=0,
                        help='Camera device (default: 0 for /dev/video0)')
    parser.add_argument('--model', default='yolov8n.pt',
                        help='Path to YOLO model (default: yolov8n.pt)')
    parser.add_argument('--confidence', type=float, default=0.5,
                        help='Confidence threshold 0-1 (default: 0.5)')
    parser.add_argument('--display', action='store_true',
                        help='Show video window with detections')
    parser.add_argument('--save-video',
                        help='Save annotated video to file')
    parser.add_argument('--fps', type=int, default=10,
                        help='Target FPS (default: 10)')
    parser.add_argument('--duration', type=float,
                        help='Run for specified seconds (default: infinite)')
    parser.add_argument('--max-frames', type=int,
                        help='Process max number of frames (default: infinite)')

    args = parser.parse_args()

    # Create detector
    detector = YOLODetector(
        model_path=args.model,
        camera_device=args.camera,
        confidence=args.confidence,
        display=args.display,
        save_video=args.save_video,
        fps=args.fps
    )

    try:
        # Run detection
        detector.run_continuous(
            duration=args.duration,
            max_frames=args.max_frames
        )
    finally:
        detector.cleanup()


if __name__ == '__main__':
    main()
