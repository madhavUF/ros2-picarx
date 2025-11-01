#!/usr/bin/env python3
"""
Behavior Controller Node

Subscribes to:
  - /tracking/target_person (TargetPerson): Person tracking information
  - /tracking/person_state (String): Tracking state
  - /sensor/distance (Float32): Ultrasonic distance for obstacle avoidance

Publishes:
  - /control/robot_command (RobotCommand): Movement commands for robot
  - /control/camera_command (CameraCommand): Camera servo commands
"""

import rclpy
from rclpy.node import Node
from my_first_pkg.msg import TargetPerson, RobotCommand, CameraCommand
from std_msgs.msg import String, Float32


class BehaviorControllerNode(Node):
    def __init__(self):
        super().__init__('behavior_controller_node')

        # Parameters for robot control
        self.declare_parameter('base_speed', 25.0)
        self.declare_parameter('turn_speed', 20.0)
        self.declare_parameter('steering_angle', 15.0)
        self.declare_parameter('horizontal_deadzone', 80.0)

        # Parameters for distance thresholds (based on area)
        self.declare_parameter('area_too_close', 150000.0)
        self.declare_parameter('area_good_min', 50000.0)
        self.declare_parameter('area_good_max', 150000.0)
        self.declare_parameter('area_medium', 15000.0)

        # Parameters for obstacle avoidance
        self.declare_parameter('obstacle_emergency_distance', 20.0)  # cm
        self.declare_parameter('obstacle_warning_distance', 30.0)  # cm

        # Parameters for camera tracking
        self.declare_parameter('camera_pan_gain', 0.1)
        self.declare_parameter('camera_tilt_angle', 0.0)

        # Get parameters
        self.base_speed = self.get_parameter('base_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.steering_angle = self.get_parameter('steering_angle').value
        self.h_deadzone = self.get_parameter('horizontal_deadzone').value
        self.area_too_close = self.get_parameter('area_too_close').value
        self.area_good_min = self.get_parameter('area_good_min').value
        self.area_good_max = self.get_parameter('area_good_max').value
        self.area_medium = self.get_parameter('area_medium').value
        self.obstacle_emergency = self.get_parameter('obstacle_emergency_distance').value
        self.obstacle_warning = self.get_parameter('obstacle_warning_distance').value
        self.camera_pan_gain = self.get_parameter('camera_pan_gain').value
        self.camera_tilt_angle = self.get_parameter('camera_tilt_angle').value

        # State
        self.current_state = 'idle'
        self.scan_direction = 1  # 1 for right, -1 for left
        self.latest_distance = None

        # Subscribers
        self.target_sub = self.create_subscription(
            TargetPerson, '/tracking/target_person',
            self.target_callback, 10)
        self.person_state_sub = self.create_subscription(
            String, '/tracking/person_state',
            self.person_state_callback, 10)
        self.distance_sub = self.create_subscription(
            Float32, '/sensor/distance',
            self.distance_callback, 10)

        # Publishers
        self.robot_cmd_pub = self.create_publisher(
            RobotCommand, '/control/robot_command', 10)
        self.camera_cmd_pub = self.create_publisher(
            CameraCommand, '/control/camera_command', 10)

        self.get_logger().info('Behavior Controller Node started')

    def person_state_callback(self, msg):
        """Handle changes in person tracking state"""
        previous_state = self.current_state
        self.current_state = msg.data

        if msg.data == 'scanning' and previous_state != 'scanning':
            self.get_logger().info('Person lost, initiating scan')

            # Command camera to scan
            cam_cmd = CameraCommand()
            cam_cmd.header.stamp = self.get_clock().now().to_msg()
            cam_cmd.header.frame_id = 'camera_frame'
            cam_cmd.pan_angle = self.scan_direction * 45.0
            cam_cmd.tilt_angle = self.camera_tilt_angle
            cam_cmd.reset_to_center = False
            self.camera_cmd_pub.publish(cam_cmd)

            # Toggle scan direction for next time
            self.scan_direction *= -1

            # Stop robot
            self.publish_stop_command()

    def distance_callback(self, msg):
        """Handle ultrasonic distance sensor updates"""
        self.latest_distance = msg.data

        # Emergency stop if obstacle too close
        if self.latest_distance < self.obstacle_emergency:
            self.get_logger().warn(
                f'Emergency stop! Obstacle at {self.latest_distance:.1f}cm',
                throttle_duration_sec=1.0)
            self.publish_stop_command()

    def target_callback(self, msg):
        """Handle person tracking updates and generate control commands"""
        if not msg.person_detected:
            return

        # Camera tracking - follow person with camera
        cam_cmd = CameraCommand()
        cam_cmd.header = msg.header

        # Proportional pan based on horizontal offset
        # Negative sign for correct direction (opposite of offset)
        cam_cmd.pan_angle = -msg.horizontal_offset * self.camera_pan_gain
        cam_cmd.tilt_angle = self.camera_tilt_angle
        cam_cmd.reset_to_center = False

        # Clamp angles
        cam_cmd.pan_angle = max(-90.0, min(90.0, cam_cmd.pan_angle))

        self.camera_cmd_pub.publish(cam_cmd)

        # Robot movement control
        robot_cmd = RobotCommand()
        robot_cmd.header = msg.header

        area = msg.area

        # Check for obstacle override
        if self.latest_distance and self.latest_distance < self.obstacle_warning:
            robot_cmd.action = 'stop'
            robot_cmd.speed = 0.0
            robot_cmd.steering_angle = 0.0
            self.get_logger().info(
                'Stopped due to obstacle',
                throttle_duration_sec=2.0)

        elif area > self.area_too_close:
            # Too close - back up
            robot_cmd.action = 'backward'
            robot_cmd.speed = self.base_speed
            robot_cmd.steering_angle = 0.0
            self.get_logger().info(
                'Person too close, backing up',
                throttle_duration_sec=2.0)

        elif self.area_good_min < area <= self.area_good_max:
            # Good distance - just adjust direction
            if abs(msg.horizontal_offset) > self.h_deadzone:
                if msg.horizontal_offset > 0:
                    robot_cmd.action = 'turn_right'
                else:
                    robot_cmd.action = 'turn_left'
                robot_cmd.speed = self.turn_speed
                robot_cmd.steering_angle = self.steering_angle
                self.get_logger().info(
                    f'Adjusting direction: offset={msg.horizontal_offset:.1f}px',
                    throttle_duration_sec=2.0)
            else:
                robot_cmd.action = 'stop'
                robot_cmd.speed = 0.0
                robot_cmd.steering_angle = 0.0
                self.get_logger().info(
                    'At good distance and centered',
                    throttle_duration_sec=2.0)

        elif self.area_medium < area <= self.area_good_min:
            # Medium distance - approach while adjusting direction
            if abs(msg.horizontal_offset) > self.h_deadzone:
                if msg.horizontal_offset > 0:
                    robot_cmd.action = 'turn_right'
                else:
                    robot_cmd.action = 'turn_left'
                robot_cmd.speed = self.turn_speed
                robot_cmd.steering_angle = self.steering_angle
            else:
                robot_cmd.action = 'forward'
                robot_cmd.speed = self.base_speed
                robot_cmd.steering_angle = 0.0
            self.get_logger().info(
                'Approaching person',
                throttle_duration_sec=2.0)

        else:
            # Too far - move forward
            robot_cmd.action = 'forward'
            robot_cmd.speed = self.base_speed
            robot_cmd.steering_angle = 0.0
            self.get_logger().info(
                'Person far, moving forward',
                throttle_duration_sec=2.0)

        # Publish robot command
        self.robot_cmd_pub.publish(robot_cmd)

    def publish_stop_command(self):
        """Helper method to publish a stop command"""
        robot_cmd = RobotCommand()
        robot_cmd.header.stamp = self.get_clock().now().to_msg()
        robot_cmd.header.frame_id = 'base_link'
        robot_cmd.action = 'stop'
        robot_cmd.speed = 0.0
        robot_cmd.steering_angle = 0.0
        self.robot_cmd_pub.publish(robot_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
