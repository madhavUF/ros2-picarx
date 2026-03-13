#!/usr/bin/env python3
"""
Robot Controller Node

Subscribes to: /control/robot_command (RobotCommand)
Controls: PiCar-X motors and steering servo

This node is the hardware abstraction layer for robot movement.
"""

import rclpy
from rclpy.node import Node
from my_first_pkg.msg import RobotCommand


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        # Parameters
        self.declare_parameter('simulation_mode', False)

        # Initialize hardware
        sim_mode = self.get_parameter('simulation_mode').value

        if not sim_mode:
            try:
                from picarx import Picarx
                self.px = Picarx()
                self.simulation_mode = False
                self.get_logger().info('PiCar-X hardware initialized successfully')
            except Exception as e:
                self.get_logger().warn(
                    f'Hardware initialization failed: {e}. '
                    'Running in simulation mode.')
                self.px = None
                self.simulation_mode = True
        else:
            self.px = None
            self.simulation_mode = True
            self.get_logger().info('Running in simulation mode (by parameter)')

        # Subscriber
        self.cmd_sub = self.create_subscription(
            RobotCommand, '/control/robot_command',
            self.command_callback, 10)

        self.get_logger().info('Robot Controller Node started')

    def command_callback(self, msg):
        """Execute robot movement commands"""
        action = msg.action
        speed = int(msg.speed)
        angle = int(msg.steering_angle)

        if self.simulation_mode:
            self.get_logger().info(
                f'[SIM] Action: {action}, Speed: {speed}, Angle: {angle}',
                throttle_duration_sec=1.0)
            return

        try:
            if action == 'forward':
                self.px.set_dir_servo_angle(angle)
                self.px.forward(speed)
                self.get_logger().debug(f'Moving forward at speed {speed}')

            elif action == 'backward':
                self.px.set_dir_servo_angle(angle)
                self.px.backward(speed)
                self.get_logger().debug(f'Moving backward at speed {speed}')

            elif action == 'turn_left':
                self.px.set_dir_servo_angle(-angle)
                self.px.forward(speed)
                self.get_logger().debug(f'Turning left at angle {-angle}')

            elif action == 'turn_right':
                self.px.set_dir_servo_angle(angle)
                self.px.forward(speed)
                self.get_logger().debug(f'Turning right at angle {angle}')

            elif action == 'stop':
                self.px.stop()
                self.px.set_dir_servo_angle(0)
                self.get_logger().debug('Stopped')

            else:
                self.get_logger().warn(f'Unknown action: {action}')

        except Exception as e:
            self.get_logger().error(f'Hardware command failed: {e}')

    def destroy_node(self):
        """Clean up - stop the robot"""
        if not self.simulation_mode and self.px is not None:
            try:
                self.px.stop()
                self.px.set_dir_servo_angle(0)
                self.get_logger().info('Robot stopped')
            except Exception as e:
                self.get_logger().error(f'Error stopping robot: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
