#!/usr/bin/env python3
"""
YOLO Detector Node

Subscribes to: /dev/video10 camera (direct OpenCV capture)
Publishes:
  - /vision/detections (DetectionArray): All detected objects
  - /vision/annotated_image (Image): Annotated image with bounding boxes
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from my_first_pkg.msg import Detection, DetectionArray
from ultralytics import YOLO
import cv2


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_device', '/dev/video10')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('publish_annotated_image', True)

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        camera_device = self.get_parameter('camera_device').value
        publish_rate = self.get_parameter('publish_rate').value
        self.publish_annotated = self.get_parameter('publish_annotated_image').value

        # Initialize YOLO
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'YOLO model loaded successfully')

        # Initialize camera
        self.get_logger().info(f'Opening camera: {camera_device}')
        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera: {camera_device}')
            raise RuntimeError('Camera initialization failed')

        # Read a few dummy frames to let camera stabilize
        for _ in range(5):
            self.cap.read()

        self.get_logger().info('Camera initialized successfully')
        self.bridge = CvBridge()

        # Publishers
        self.detection_pub = self.create_publisher(
            DetectionArray, '/vision/detections', 10)
        self.image_pub = self.create_publisher(
            Image, '/vision/annotated_image', 10)

        # Timer for processing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.detect_callback)

        self.get_logger().info(f'YOLO Detector Node started at {publish_rate} Hz')

    def detect_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        # Run YOLO inference
        results = self.model(frame, verbose=False)

        # Create detection message
        detection_array = DetectionArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_frame'
        detection_array.frame_height = frame.shape[0]
        detection_array.frame_width = frame.shape[1]

        if len(results[0].boxes) > 0:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            confs = results[0].boxes.conf.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy()

            for box, conf, cls in zip(boxes, confs, classes):
                if conf < self.conf_threshold:
                    continue

                detection = Detection()
                detection.class_name = self.model.names[int(cls)]
                detection.confidence = float(conf)
                detection.x1 = float(box[0])
                detection.y1 = float(box[1])
                detection.x2 = float(box[2])
                detection.y2 = float(box[3])
                detection.center_x = (detection.x1 + detection.x2) / 2
                detection.center_y = (detection.y1 + detection.y2) / 2
                detection.area = (detection.x2 - detection.x1) * (detection.y2 - detection.y1)

                detection_array.detections.append(detection)

        # Publish detections
        self.detection_pub.publish(detection_array)

        if len(detection_array.detections) > 0:
            self.get_logger().debug(
                f'Published {len(detection_array.detections)} detections')

        # Optionally publish annotated image
        if self.publish_annotated:
            annotated = results[0].plot()
            image_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            image_msg.header = detection_array.header
            self.image_pub.publish(image_msg)

    def destroy_node(self):
        """Clean up resources"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
