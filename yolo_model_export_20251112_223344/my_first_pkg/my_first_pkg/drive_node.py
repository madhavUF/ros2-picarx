import rclpy
from rclpy.node import Node
from picarx import Picarx
from time import sleep

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.px = Picarx()
        self.get_logger().info('PiCar-X Ready! Moving forward...')
        self.px.forward(20)
        sleep(2)
        self.get_logger().info('Stopping.')
        self.px.stop()

def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    rclpy.spin_once(node, timeout_sec=0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
