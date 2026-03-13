import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WheelCommandPublisher(Node):
    def __init__(self):
        super().__init__('wheel_command_publisher')
        self.publisher_ = self.create_publisher(String, 'cmd_drive', 10)
        self.timer = self.create_timer(2.0, self.publish_command)
        self.commands = ["forward", "stop", "backward", "stop"]
        self.index = 0
        self.get_logger().info('🚀 WheelCommandPublisher started')

    def publish_command(self):
        msg = String()
        msg.data = self.commands[self.index]
        self.publisher_.publish(msg)
        self.get_logger().info(f'📤 Publishing: {msg.data}')
        self.index = (self.index + 1) % len(self.commands)

def main(args=None):
    rclpy.init(args=args)
    node = WheelCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
