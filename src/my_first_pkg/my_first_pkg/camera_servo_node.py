#!/usr/bin/env python3
"""
Camera Servo Node

Subscribes to: /control/camera_command (CameraCommand)
Controls: PiCar-X camera pan and tilt servos

This node is the hardware abstraction layer for camera movement.
"""

import rclpy
from rclpy.node import Node
from my_first_pkg.msg import CameraCommand


class CameraServoNode(Node):
    def __init__(self):
        super().__init__('camera_servo_node')

        # Parameters
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('default_pan', 0.0)
        self.declare_parameter('default_tilt', 0.0)

        # Initialize hardware
        sim_mode = self.get_parameter('simulation_mode').value
        self.default_pan = self.get_parameter('default_pan').value
        self.default_tilt = self.get_parameter('default_tilt').value

        if not sim_mode:
            try:
                from picarx import Picarx
                self.px = Picarx()
                self.simulation_mode = False
                self.get_logger().info('Camera servo hardware initialized successfully')

                # Set to default position
                self.px.set_cam_pan_angle(int(self.default_pan))
                self.px.set_cam_tilt_angle(int(self.default_tilt))

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

        # Current servo positions
        self.current_pan = self.default_pan
        self.current_tilt = self.default_tilt

        # Subscriber
        self.cmd_sub = self.create_subscription(
            CameraCommand, '/control/camera_command',
            self.command_callback, 10)

        self.get_logger().info('Camera Servo Node started')

    def command_callback(self, msg):
        """Execute camera servo commands"""
        if msg.reset_to_center:
            pan_angle = self.default_pan
            tilt_angle = self.default_tilt
            self.get_logger().info('Resetting camera to center')
        else:
            # Clamp angles to valid ranges
            pan_angle = max(-90.0, min(90.0, msg.pan_angle))
            tilt_angle = max(-30.0, min(90.0, msg.tilt_angle))

            # Warn if clamping occurred
            if pan_angle != msg.pan_angle:
                self.get_logger().warn(
                    f'Pan angle clamped from {msg.pan_angle:.1f} to {pan_angle:.1f}')
            if tilt_angle != msg.tilt_angle:
                self.get_logger().warn(
                    f'Tilt angle clamped from {msg.tilt_angle:.1f} to {tilt_angle:.1f}')

        if self.simulation_mode:
            self.get_logger().info(
                f'[SIM] Camera: Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°',
                throttle_duration_sec=1.0)
            self.current_pan = pan_angle
            self.current_tilt = tilt_angle
            return

        try:
            # Set camera pan servo
            self.px.set_cam_pan_angle(int(pan_angle))
            # Set camera tilt servo
            self.px.set_cam_tilt_angle(int(tilt_angle))

            self.current_pan = pan_angle
            self.current_tilt = tilt_angle

            self.get_logger().debug(
                f'Camera moved: Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°')

        except Exception as e:
            self.get_logger().error(f'Camera servo command failed: {e}')

    def destroy_node(self):
        """Clean up - reset camera to center"""
        if not self.simulation_mode and self.px is not None:
            try:
                self.px.set_cam_pan_angle(int(self.default_pan))
                self.px.set_cam_tilt_angle(int(self.default_tilt))
                self.get_logger().info('Camera reset to center')
            except Exception as e:
                self.get_logger().error(f'Error resetting camera: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraServoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
