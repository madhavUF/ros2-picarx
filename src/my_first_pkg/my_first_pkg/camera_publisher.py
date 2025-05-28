import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 FPS
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video10', cv2.CAP_V4L2)
        self.get_logger().info(f'Camera open status: {self.cap.isOpened()}')
        time.sleep(2)  # Give camera hardware a moment

        if not self.cap.isOpened():
            self.get_logger().error('❌ Failed to open camera.')
        else:
            self.get_logger().info('📷 CameraPublisher node started!')

    def publish_frame(self):
        for attempt in range(5):
            ret, frame = self.cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher_.publish(msg)
                self.get_logger().info('📤 Published camera frame')
                return
            else:
                self.get_logger().warn(f'⚠️ Attempt {attempt + 1}: Failed to grab frame.')
                time.sleep(0.1)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
