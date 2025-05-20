import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from picarx import Picarx

class WheelCommandListener(Node):
    def __init__(self):
        super().__init__('wheel_command_listener')
        self.subscription = self.create_subscription(
            String,
            'cmd_drive',
            self.listener_callback,
            10
        )
        self.px = Picarx()
        self.get_logger().info('🛰️ WheelCommandListener ready to drive!')

    def listener_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f'🛰️ Received: {command}')

        if command == "forward":
            self.px.forward(50)  # Adjust speed as needed
        elif command == "backward":
            self.px.backward(50)
        elif command == "left":
            self.px.set_dir_servo_angle(-30)
        elif command == "right":
            self.px.set_dir_servo_angle(30)
        elif command == "stop":
            self.px.stop()
            self.px.set_dir_servo_angle(0)  # Reset steering
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
