import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class MultiSensorSubscriber(Node):
    def __init__(self):
        super().__init__('multi_sensor_subscriber')

        self.subscription_distance = self.create_subscription(
            Float32,
            '/sensor/distance',
            self.listener_callback_distance,
            10)

        self.subscription_greyscale = self.create_subscription(
            String,
            '/sensor/greyscale',
            self.listener_callback_greyscale,
            10)

        self.get_logger().info('✅ MultiSensorSubscriber node started.')

    def listener_callback_distance(self, msg):
        self.get_logger().info(f'Received Distance: {msg.data:.2f}')

    def listener_callback_greyscale(self, msg):
        self.get_logger().info(f'Received Greyscale: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
