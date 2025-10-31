#!/usr/bin/env python3
"""
Vision Only Launch File

Launches only the vision system for testing:
- YOLO detector
- Person tracker

Useful for testing vision without moving the robot.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLO model file'
    )

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video10',
        description='Camera device path'
    )

    publish_image_arg = DeclareLaunchArgument(
        'publish_annotated_image',
        default_value='true',
        description='Publish annotated images for visualization'
    )

    # Get launch configurations
    model_path = LaunchConfiguration('model_path')
    camera_device = LaunchConfiguration('camera_device')
    publish_image = LaunchConfiguration('publish_annotated_image')

    # YOLO Detector Node
    yolo_detector = Node(
        package='my_first_pkg',
        executable='yolo_detector_node.py',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'camera_device': camera_device,
            'confidence_threshold': 0.5,
            'publish_rate': 10.0,
            'publish_annotated_image': publish_image
        }]
    )

    # Person Tracker Node
    person_tracker = Node(
        package='my_first_pkg',
        executable='person_tracker_node.py',
        name='person_tracker',
        output='screen',
        parameters=[{
            'min_confidence': 0.5,
            'min_area': 5000.0,
            'lost_threshold': 10
        }]
    )

    return LaunchDescription([
        model_path_arg,
        camera_device_arg,
        publish_image_arg,
        yolo_detector,
        person_tracker
    ])
