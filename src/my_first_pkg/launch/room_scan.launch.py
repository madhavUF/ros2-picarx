#!/usr/bin/env python3
"""
Launch file for Room Scanning Mission

This launches:
1. YOLO Detector Node - for object detection
2. Robot Controller Node - for movement control
3. Camera Servo Node - for camera control
4. Room Scanner Node - mission orchestrator

Usage:
  ros2 launch my_first_pkg room_scan.launch.py

  # For simulation mode (no hardware):
  ros2 launch my_first_pkg room_scan.launch.py simulation_mode:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='Run in simulation mode without hardware'
    )

    yolo_model_path_arg = DeclareLaunchArgument(
        'yolo_model_path',
        default_value='yolov8n.pt',
        description='Path to YOLO model file'
    )

    scan_positions_arg = DeclareLaunchArgument(
        'scan_positions',
        default_value='8',
        description='Number of positions to scan (360 degrees / positions)'
    )

    pause_duration_arg = DeclareLaunchArgument(
        'pause_duration',
        default_value='2.0',
        description='Seconds to pause at each position for detection'
    )

    camera_tilt_arg = DeclareLaunchArgument(
        'camera_tilt',
        default_value='35.0',
        description='Camera tilt angle to look up at room (degrees)'
    )

    # Get launch configurations
    simulation_mode = LaunchConfiguration('simulation_mode')
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    scan_positions = LaunchConfiguration('scan_positions')
    pause_duration = LaunchConfiguration('pause_duration')
    camera_tilt = LaunchConfiguration('camera_tilt')

    # YOLO Detector Node
    yolo_detector = Node(
        package='my_first_pkg',
        executable='yolo_detector_node.py',
        name='yolo_detector_node',
        output='screen',
        parameters=[{
            'model_path': yolo_model_path,
            'confidence_threshold': 0.5,
            'camera_device': '/dev/video10',
            'publish_rate': 5.0,  # Lower rate for mission
            'publish_annotated_image': False,  # Don't need annotated images
        }]
    )

    # Robot Controller Node
    robot_controller = Node(
        package='my_first_pkg',
        executable='robot_controller_node.py',
        name='robot_controller_node',
        output='screen',
        parameters=[{
            'simulation_mode': simulation_mode,
        }]
    )

    # Camera Servo Node
    camera_servo = Node(
        package='my_first_pkg',
        executable='camera_servo_node.py',
        name='camera_servo_node',
        output='screen',
        parameters=[{
            'simulation_mode': simulation_mode,
            'default_pan': 0.0,
            'default_tilt': 0.0,
        }]
    )

    # Room Scanner Mission Node
    room_scanner = Node(
        package='my_first_pkg',
        executable='room_scanner_node.py',
        name='room_scanner_node',
        output='screen',
        parameters=[{
            'scan_speed': 20.0,
            'turn_angle': 30.0,
            'scan_positions': scan_positions,
            'pause_duration': pause_duration,
            'confidence_threshold': 0.6,
            'camera_tilt_angle': camera_tilt,
        }]
    )

    return LaunchDescription([
        # Arguments
        simulation_mode_arg,
        yolo_model_path_arg,
        scan_positions_arg,
        pause_duration_arg,
        camera_tilt_arg,

        # Nodes
        yolo_detector,
        robot_controller,
        camera_servo,
        room_scanner,
    ])
