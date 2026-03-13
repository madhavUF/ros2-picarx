#!/usr/bin/env python
"""
YOLO Detector Node for ROS1

Subscribes to: Camera device (direct OpenCV capture)
Publishes:
  - /vision/detections (DetectionArray): All detected objects
  - /vision/annotated_image (Image): Annotated image with bounding boxes
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo_detector_ros1.msg import Detection, DetectionArray
from ultralytics import YOLO
import cv2


class YoloDetectorNode:
    def __init__(self):
        rospy.init_node('yolo_detector_node', anonymous=False)

        # Parameters
        self.model_path = rospy.get_param('~model_path', 'yolov8n.pt')
        self.conf_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.camera_device = rospy.get_param('~camera_device', '/dev/video0')
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Hz
        self.publish_annotated = rospy.get_param('~publish_annotated_image', True)

        # Initialize YOLO
        rospy.loginfo('Loading YOLO model: %s', self.model_path)
        self.model = YOLO(self.model_path)
        rospy.loginfo('YOLO model loaded successfully')

        # Initialize camera
        rospy.loginfo('Opening camera: %s', self.camera_device)
        self.cap = cv2.VideoCapture(self.camera_device)

        if not self.cap.isOpened():
            rospy.logerr('Failed to open camera: %s', self.camera_device)
            raise RuntimeError('Camera initialization failed')

        # Read a few dummy frames to let camera stabilize
        for _ in range(5):
            self.cap.read()

        rospy.loginfo('Camera initialized successfully')
        self.bridge = CvBridge()

        # Publishers
        self.detection_pub = rospy.Publisher(
            '/vision/detections', DetectionArray, queue_size=10)
        self.image_pub = rospy.Publisher(
            '/vision/annotated_image', Image, queue_size=10)

        # Timer for processing
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.detect_callback)

        rospy.loginfo('YOLO Detector Node started at %.1f Hz', self.publish_rate)

    def detect_callback(self, event):
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn('Failed to capture frame')
            return

        # Run YOLO inference
        results = self.model(frame, verbose=False)

        # Create detection message
        detection_array = DetectionArray()
        detection_array.header.stamp = rospy.Time.now()
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
            rospy.logdebug('Published %d detections', len(detection_array.detections))

        # Optionally publish annotated image
        if self.publish_annotated:
            annotated = results[0].plot()
            image_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            image_msg.header = detection_array.header
            self.image_pub.publish(image_msg)

    def cleanup(self):
        """Clean up resources"""
        if self.cap is not None:
            self.cap.release()
        rospy.loginfo('YOLO Detector Node shutting down')


def main():
    try:
        node = YoloDetectorNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down YOLO Detector Node')
    finally:
        if 'node' in locals():
            node.cleanup()


if __name__ == '__main__':
    main()
