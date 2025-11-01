#!/usr/bin/env python3
"""
Room Scanner Mission Node

This node orchestrates a room scanning mission where the robot:
1. Rotates in place to scan 360 degrees
2. Collects and reports all detected objects
3. Can optionally move to different positions for comprehensive scanning

Subscribes to: /vision/detections (DetectionArray)
Publishes to: /control/robot_command (RobotCommand)
"""

import rclpy
from rclpy.node import Node
from my_first_pkg.msg import DetectionArray, RobotCommand
from std_msgs.msg import Header
import time
from collections import defaultdict


class RoomScannerNode(Node):
    def __init__(self):
        super().__init__('room_scanner_node')

        # Parameters
        self.declare_parameter('scan_speed', 20.0)
        self.declare_parameter('turn_angle', 30.0)
        self.declare_parameter('scan_positions', 8)  # 360/45 = 8 positions
        self.declare_parameter('pause_duration', 2.0)  # seconds to pause and observe
        self.declare_parameter('confidence_threshold', 0.6)

        self.scan_speed = self.get_parameter('scan_speed').value
        self.turn_angle = self.get_parameter('turn_angle').value
        self.scan_positions = self.get_parameter('scan_positions').value
        self.pause_duration = self.get_parameter('pause_duration').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # State variables
        self.objects_found = defaultdict(int)  # {object_name: count}
        self.mission_active = False
        self.current_position = 0
        self.detections_buffer = []

        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(RobotCommand, '/control/robot_command', 10)
        self.detection_sub = self.create_subscription(
            DetectionArray, '/vision/detections',
            self.detection_callback, 10)

        self.get_logger().info('Room Scanner Node initialized')
        self.get_logger().info(f'Configuration: {self.scan_positions} positions, '
                              f'{self.pause_duration}s pause, '
                              f'{self.confidence_threshold} confidence threshold')

        # Start mission after a short delay
        self.startup_timer = self.create_timer(2.0, self.start_mission_timer)

    def start_mission_timer(self):
        """Start the scanning mission after initialization"""
        self.destroy_timer(self.startup_timer)  # One-time timer
        self.start_mission()

    def detection_callback(self, msg):
        """Collect detections during scanning"""
        if self.mission_active:
            for detection in msg.detections:
                if detection.confidence >= self.confidence_threshold:
                    self.detections_buffer.append({
                        'class_name': detection.class_name,
                        'confidence': detection.confidence,
                        'position': self.current_position
                    })

    def send_command(self, action, speed=0.0, angle=0.0):
        """Send a movement command to the robot"""
        cmd = RobotCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.action = action
        cmd.speed = float(speed)
        cmd.steering_angle = float(angle)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Command sent: {action} (speed={speed}, angle={angle})')

    def start_mission(self):
        """Execute the room scanning mission"""
        self.mission_active = True
        self.get_logger().info('=' * 60)
        self.get_logger().info('MISSION START: Room Scanning Initiated')
        self.get_logger().info('=' * 60)

        # Calculate rotation per step
        degrees_per_step = 360.0 / self.scan_positions

        for position in range(self.scan_positions):
            self.current_position = position

            self.get_logger().info(f'\n--- Scan Position {position + 1}/{self.scan_positions} ---')
            self.get_logger().info(f'Rotating to position {position + 1}...')

            # Rotate to next position
            if position > 0:  # Don't rotate on first position
                self.send_command('turn_left', speed=self.scan_speed, angle=self.turn_angle)
                # Calculate rotation time based on desired angle
                rotation_time = degrees_per_step / 90.0  # Approximate: 90 degrees takes ~1 second
                time.sleep(rotation_time)
                self.send_command('stop')

            # Pause and collect detections
            self.get_logger().info(f'Pausing for {self.pause_duration}s to detect objects...')
            self.detections_buffer = []  # Clear buffer
            time.sleep(self.pause_duration)

            # Process detections from this position
            self.process_position_detections()

        # Mission complete
        self.send_command('stop')
        self.mission_active = False
        self.report_findings()

    def process_position_detections(self):
        """Process detections collected during the pause"""
        if not self.detections_buffer:
            self.get_logger().info('  No objects detected at this position')
            return

        # Count unique objects at this position
        position_objects = defaultdict(int)
        for det in self.detections_buffer:
            position_objects[det['class_name']] += 1
            self.objects_found[det['class_name']] += 1

        # Report position findings
        self.get_logger().info(f'  Detected at position {self.current_position + 1}:')
        for obj_name, count in sorted(position_objects.items()):
            avg_conf = sum(d['confidence'] for d in self.detections_buffer
                          if d['class_name'] == obj_name) / count
            self.get_logger().info(f'    - {obj_name}: {count} detections '
                                  f'(avg confidence: {avg_conf:.2f})')

    def report_findings(self):
        """Generate final mission report"""
        self.get_logger().info('\n')
        self.get_logger().info('=' * 60)
        self.get_logger().info('MISSION COMPLETE: Room Scan Finished')
        self.get_logger().info('=' * 60)

        if not self.objects_found:
            self.get_logger().info('No objects detected during scan.')
            return

        self.get_logger().info('\nOBJECTS FOUND IN ROOM:')
        self.get_logger().info('-' * 40)

        # Sort by count (most common first)
        sorted_objects = sorted(self.objects_found.items(),
                               key=lambda x: x[1], reverse=True)

        for i, (obj_name, count) in enumerate(sorted_objects, 1):
            self.get_logger().info(f'{i}. {obj_name.upper()}: {count} detections')

        self.get_logger().info('-' * 40)
        self.get_logger().info(f'Total unique objects: {len(self.objects_found)}')
        self.get_logger().info(f'Total detections: {sum(self.objects_found.values())}')
        self.get_logger().info('=' * 60)

    def destroy_node(self):
        """Clean up - stop the robot"""
        self.send_command('stop')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoomScannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
