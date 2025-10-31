#!/usr/bin/env python3
"""
Person Following Launch File

Launches the complete person-following system:
- YOLO detector
- Person tracker
- Behavior controller
- Robot controller
- Camera servo controller
- Multi-sensor publisher (ultrasonic + grayscale)

This is the main launch file for autonomous person following.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode (no hardware)'
    )

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

    # Get launch configurations
    simulation = LaunchConfiguration('simulation')
    model_path = LaunchConfiguration('model_path')
    camera_device = LaunchConfiguration('camera_device')

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
            'publish_annotated_image': False  # Set to True for debugging
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

    # Behavior Controller Node
    behavior_controller = Node(
        package='my_first_pkg',
        executable='behavior_controller_node.py',
        name='behavior_controller',
        output='screen',
        parameters=[{
            'base_speed': 25.0,
            'turn_speed': 20.0,
            'steering_angle': 15.0,
            'horizontal_deadzone': 80.0,
            'area_too_close': 150000.0,
            'area_good_min': 50000.0,
            'area_good_max': 150000.0,
            'area_medium': 15000.0,
            'obstacle_emergency_distance': 20.0,
            'obstacle_warning_distance': 30.0,
            'camera_pan_gain': 0.1,
            'camera_tilt_angle': 0.0
        }]
    )

    # Robot Controller Node
    robot_controller = Node(
        package='my_first_pkg',
        executable='robot_controller_node.py',
        name='robot_controller',
        output='screen',
        parameters=[{
            'simulation_mode': simulation
        }]
    )

    # Camera Servo Node
    camera_servo = Node(
        package='my_first_pkg',
        executable='camera_servo_node.py',
        name='camera_servo',
        output='screen',
        parameters=[{
            'simulation_mode': simulation,
            'default_pan': 0.0,
            'default_tilt': 0.0
        }]
    )

    # Multi Sensor Publisher (ultrasonic + grayscale)
    multi_sensor = Node(
        package='my_first_pkg',
        executable='multi_sensor_publisher.py',
        name='multi_sensor_publisher',
        output='screen'
    )

    return LaunchDescription([
        simulation_arg,
        model_path_arg,
        camera_device_arg,
        yolo_detector,
        person_tracker,
        behavior_controller,
        robot_controller,
        camera_servo,
        multi_sensor
    ])
