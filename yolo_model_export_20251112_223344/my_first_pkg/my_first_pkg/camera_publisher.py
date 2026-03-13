# Update the camera publisher to work more reliably
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
        
        # Use the working configuration
        self.cap = cv2.VideoCapture('/dev/video10')
        
        if self.cap.isOpened():
            # Give camera time to initialize
            time.sleep(2)
            # Do a few dummy reads to stabilize
            for i in range(5):
                ret, frame = self.cap.read()
                time.sleep(0.1)
        
        self.get_logger().info(f'Camera open status: {self.cap.isOpened()}')
        
        if not self.cap.isOpened():
            self.get_logger().error('❌ Failed to open camera.')
        else:
            self.get_logger().info('📷 CameraPublisher node started!')

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('📤 Published camera frame')
        else:
            self.get_logger().warn('⚠️ Failed to grab frame.')

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

