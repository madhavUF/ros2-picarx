import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('📥 CameraSubscriber node started!')

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(f"/workspace/images/frame_{int(time.time())}.jpg", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('🛑 Quit signal received, shutting down.')
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'❌ Error converting image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
