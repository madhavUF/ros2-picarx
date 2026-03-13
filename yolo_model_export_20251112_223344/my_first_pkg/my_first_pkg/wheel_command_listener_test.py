import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WheelCommandListener(Node):
    def __init__(self):
        super().__init__('wheel_command_listener_test')
        self.subscription = self.create_subscription(
            String,
            'cmd_drive',
            self.listener_callback,
            10
        )
        self.get_logger().info('🛰️ WheelCommandListener TEST ready (no hardware)!')

    def listener_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f'🛰️ TEST Received: {command}')
        
        # Just log what we would do instead of actually driving
        if command == "forward":
            self.get_logger().info('🚗 Would move FORWARD')
        elif command == "backward":
            self.get_logger().info('🚗 Would move BACKWARD')
        elif command == "left":
            self.get_logger().info('🚗 Would turn LEFT')
        elif command == "right":
            self.get_logger().info('🚗 Would turn RIGHT')
        elif command == "stop":
            self.get_logger().info('🚗 Would STOP')
        else:
            self.get_logger().warn("⚠️ Unknown command received.")

def main(args=None):
    rclpy.init(args=args)
    node = WheelCommandListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
