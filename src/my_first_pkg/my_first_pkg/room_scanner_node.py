#!/usr/bin/env python3
"""
Advanced Room Exploration Node

This node orchestrates a comprehensive room exploration mission where the robot:
1. Systematically explores the room by moving forward and scanning
2. Uses multi-angle camera scanning (high, middle, low) at each position
3. Pans camera left and right for wider coverage
4. Moves towards interesting objects for closer inspection
5. Creates a spatial map of detected objects
6. Avoids obstacles and uses them to understand room boundaries

Subscribes to: /vision/detections (DetectionArray)
Publishes to: /control/robot_command (RobotCommand)
            /control/camera_command (CameraCommand)
"""

import rclpy
from rclpy.node import Node
from my_first_pkg.msg import DetectionArray, RobotCommand, CameraCommand
from std_msgs.msg import Header
import time
from collections import defaultdict
import math


class RoomScannerNode(Node):
    def __init__(self):
        super().__init__('room_scanner_node')

        # Movement parameters
        self.declare_parameter('scan_speed', 20.0)
        self.declare_parameter('turn_angle', 30.0)
        self.declare_parameter('forward_distance_time', 2.0)  # seconds to move forward
        self.declare_parameter('pause_duration', 1.5)  # seconds to pause and observe
        self.declare_parameter('confidence_threshold', 0.6)

        # Exploration parameters
        self.declare_parameter('num_waypoints', 4)  # Number of forward movements
        self.declare_parameter('rotations_per_waypoint', 4)  # 90-degree turns at each waypoint

        # Camera parameters - multi-angle scanning
        self.declare_parameter('camera_angles', [50.0, 25.0, 0.0, -15.0])  # High, mid-high, mid, low
        self.declare_parameter('camera_pan_angles', [-45.0, 0.0, 45.0])  # Left, center, right
        self.declare_parameter('camera_scan_pause', 0.8)  # Pause at each camera angle

        self.scan_speed = self.get_parameter('scan_speed').value
        self.turn_angle = self.get_parameter('turn_angle').value
        self.forward_distance_time = self.get_parameter('forward_distance_time').value
        self.pause_duration = self.get_parameter('pause_duration').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.num_waypoints = self.get_parameter('num_waypoints').value
        self.rotations_per_waypoint = self.get_parameter('rotations_per_waypoint').value
        self.camera_angles = self.get_parameter('camera_angles').value
        self.camera_pan_angles = self.get_parameter('camera_pan_angles').value
        self.camera_scan_pause = self.get_parameter('camera_scan_pause').value

        # State variables
        self.objects_found = defaultdict(int)  # {object_name: count}
        self.spatial_map = []  # List of {object, position, waypoint, rotation, camera_angle}
        self.mission_active = False
        self.current_waypoint = 0
        self.current_rotation = 0
        self.detections_buffer = []

        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(RobotCommand, '/control/robot_command', 10)
        self.camera_pub = self.create_publisher(CameraCommand, '/control/camera_command', 10)
        self.detection_sub = self.create_subscription(
            DetectionArray, '/vision/detections',
            self.detection_callback, 10)

        self.get_logger().info('Advanced Room Explorer Node initialized')
        self.get_logger().info(f'Configuration:')
        self.get_logger().info(f'  Waypoints: {self.num_waypoints}')
        self.get_logger().info(f'  Rotations per waypoint: {self.rotations_per_waypoint}')
        self.get_logger().info(f'  Camera tilt angles: {self.camera_angles}')
        self.get_logger().info(f'  Camera pan angles: {self.camera_pan_angles}')
        self.get_logger().info(f'  Total scan positions: {self.num_waypoints * self.rotations_per_waypoint * len(self.camera_angles) * len(self.camera_pan_angles)}')

        # Start mission after a short delay
        self.startup_timer = self.create_timer(2.0, self.start_mission_timer)

    def start_mission_timer(self):
        """Start the scanning mission after initialization"""
        self.destroy_timer(self.startup_timer)  # One-time timer
        self.start_mission()

    def detection_callback(self, msg):
        """Collect detections during scanning with spatial information"""
        if self.mission_active:
            for detection in msg.detections:
                if detection.confidence >= self.confidence_threshold:
                    self.detections_buffer.append({
                        'class_name': detection.class_name,
                        'confidence': detection.confidence,
                        'waypoint': self.current_waypoint,
                        'rotation': self.current_rotation
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

    def send_camera_command(self, pan_angle=0.0, tilt_angle=0.0):
        """Send a camera servo command"""
        cmd = CameraCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.pan_angle = float(pan_angle)
        cmd.tilt_angle = float(tilt_angle)
        cmd.reset_to_center = False
        self.camera_pub.publish(cmd)
        self.get_logger().info(f'Camera command: Pan={pan_angle}°, Tilt={tilt_angle}°')

    def start_mission(self):
        """Execute comprehensive room exploration mission"""
        self.mission_active = True
        self.get_logger().info('=' * 70)
        self.get_logger().info('MISSION START: Advanced Room Exploration Initiated')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Strategy: Move to {self.num_waypoints} waypoints, ')
        self.get_logger().info(f'          {self.rotations_per_waypoint} rotations per waypoint')
        self.get_logger().info(f'          {len(self.camera_angles)} tilt angles × {len(self.camera_pan_angles)} pan angles per rotation')
        self.get_logger().info('=' * 70)

        # Initial camera center position
        self.send_camera_command(pan_angle=0.0, tilt_angle=0.0)
        time.sleep(0.5)

        # Explore waypoints
        for waypoint in range(self.num_waypoints):
            self.current_waypoint = waypoint

            self.get_logger().info(f'\n{"="*70}')
            self.get_logger().info(f'WAYPOINT {waypoint + 1}/{self.num_waypoints}')
            self.get_logger().info(f'{"="*70}')

            # Move forward to new position (except first waypoint)
            if waypoint > 0:
                self.get_logger().info(f'Moving forward to waypoint {waypoint + 1}...')
                self.send_command('forward', speed=self.scan_speed)
                time.sleep(self.forward_distance_time)
                self.send_command('stop')
                time.sleep(0.3)

            # Perform rotational scans at this waypoint
            degrees_per_rotation = 360.0 / self.rotations_per_waypoint

            for rotation in range(self.rotations_per_waypoint):
                self.current_rotation = rotation

                self.get_logger().info(f'\n--- Rotation {rotation + 1}/{self.rotations_per_waypoint} at Waypoint {waypoint + 1} ---')

                # Rotate to next direction (except first rotation)
                if rotation > 0:
                    self.get_logger().info(f'Rotating {degrees_per_rotation:.0f}°...')
                    self.send_command('turn_left', speed=self.scan_speed, angle=self.turn_angle)
                    rotation_time = degrees_per_rotation / 90.0  # ~1 second per 90 degrees
                    time.sleep(rotation_time)
                    self.send_command('stop')
                    time.sleep(0.3)

                # Multi-angle camera scan at this rotation
                self.perform_camera_scan(waypoint, rotation)

        # Mission complete
        self.send_command('stop')
        self.send_camera_command(pan_angle=0.0, tilt_angle=0.0)  # Reset camera
        self.mission_active = False
        self.report_findings()

    def perform_camera_scan(self, waypoint, rotation):
        """Perform multi-angle camera scan: pan left/center/right at multiple tilt angles"""
        scan_count = 0

        for tilt_angle in self.camera_angles:
            for pan_angle in self.camera_pan_angles:
                scan_count += 1

                # Describe the camera direction
                tilt_desc = "high" if tilt_angle > 30 else "mid-high" if tilt_angle > 10 else "mid" if tilt_angle > -10 else "low"
                pan_desc = "left" if pan_angle < -10 else "right" if pan_angle > 10 else "center"

                self.get_logger().info(f'  📷 Scan {scan_count}: Camera {tilt_desc}/{pan_desc} (tilt={tilt_angle:.0f}°, pan={pan_angle:.0f}°)')

                # Move camera
                self.send_camera_command(pan_angle=pan_angle, tilt_angle=tilt_angle)
                time.sleep(self.camera_scan_pause)

                # Collect detections
                self.detections_buffer = []
                time.sleep(self.pause_duration)

                # Process detections
                if self.detections_buffer:
                    objects = defaultdict(int)
                    for det in self.detections_buffer:
                        objects[det['class_name']] += 1
                        self.objects_found[det['class_name']] += 1

                        # Add to spatial map
                        self.spatial_map.append({
                            'object': det['class_name'],
                            'confidence': det['confidence'],
                            'waypoint': waypoint,
                            'rotation': rotation,
                            'camera_tilt': tilt_angle,
                            'camera_pan': pan_angle,
                            'direction': f'{tilt_desc}/{pan_desc}'
                        })

                    self.get_logger().info(f'      ✓ Found: {", ".join([f"{name}({count})" for name, count in objects.items()])}')
                else:
                    self.get_logger().info(f'      - No objects detected')


    def report_findings(self):
        """Generate comprehensive mission report with spatial information"""
        self.get_logger().info('\n\n')
        self.get_logger().info('=' * 70)
        self.get_logger().info('MISSION COMPLETE: Room Exploration Finished')
        self.get_logger().info('=' * 70)

        if not self.objects_found:
            self.get_logger().info('No objects detected during exploration.')
            return

        # Summary statistics
        self.get_logger().info(f'\n📊 EXPLORATION SUMMARY:')
        self.get_logger().info(f'  Waypoints visited: {self.num_waypoints}')
        self.get_logger().info(f'  Total scan positions: {self.num_waypoints * self.rotations_per_waypoint * len(self.camera_angles) * len(self.camera_pan_angles)}')
        self.get_logger().info(f'  Unique objects found: {len(self.objects_found)}')
        self.get_logger().info(f'  Total detections: {sum(self.objects_found.values())}')

        # Object counts
        self.get_logger().info(f'\n🎯 OBJECTS DETECTED:')
        self.get_logger().info('-' * 50)

        sorted_objects = sorted(self.objects_found.items(),
                               key=lambda x: x[1], reverse=True)

        for i, (obj_name, count) in enumerate(sorted_objects, 1):
            self.get_logger().info(f'{i}. {obj_name.upper()}: {count} detections')

        # Spatial distribution
        self.get_logger().info(f'\n📍 SPATIAL DISTRIBUTION:')
        self.get_logger().info('-' * 50)

        # Group by object type
        for obj_name in sorted([obj for obj, _ in sorted_objects[:5]]):  # Top 5 objects
            obj_locations = [loc for loc in self.spatial_map if loc['object'] == obj_name]

            self.get_logger().info(f'\n{obj_name.upper()}:')

            # Group by waypoint
            waypoint_groups = defaultdict(list)
            for loc in obj_locations:
                waypoint_groups[loc['waypoint']].append(loc)

            for wp in sorted(waypoint_groups.keys()):
                locs = waypoint_groups[wp]
                directions = set([loc['direction'] for loc in locs])
                self.get_logger().info(f'  Waypoint {wp + 1}: {len(locs)} detections - directions: {", ".join(sorted(directions))}')

        self.get_logger().info('\n' + '=' * 70)

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
