#!/usr/bin/env python3
"""
MCP Bridge Launch File

Launches only the MCP bridge components for use with external clients
like Claude Desktop or Claude CLI. Does not include voice processing.

Use this when you want to control the robot via text commands from
an external application.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode (no hardware)'
    )

    # Get launch configurations
    simulation = LaunchConfiguration('simulation')

    # Core robot nodes
    robot_nodes = GroupAction([
        # YOLO Detector
        Node(
            package='my_first_pkg',
            executable='yolo_detector_node',
            name='yolo_detector',
            parameters=[{
                'model_path': 'yolov8n.pt',
                'confidence_threshold': 0.5,
                'camera_device': '/dev/video10',
                'publish_rate': 5.0,
            }]
        ),

        # Robot Controller
        Node(
            package='my_first_pkg',
            executable='robot_controller_node',
            name='robot_controller',
            parameters=[{
                'simulation_mode': simulation,
            }]
        ),

        # Camera Servo Controller
        Node(
            package='my_first_pkg',
            executable='camera_servo_node',
            name='camera_servo',
            parameters=[{
                'simulation_mode': simulation,
            }]
        ),

        # Sensor Publisher
        Node(
            package='my_first_pkg',
            executable='multi_sensor_publisher',
            name='sensor_publisher',
            condition=UnlessCondition(simulation)
        ),
    ])

    # MCP Gateway only
    mcp_nodes = GroupAction([
        Node(
            package='picarx_assistant',
            executable='ros2_mcp_server',
            name='mcp_gateway',
        ),
    ])

    return LaunchDescription([
        simulation_arg,
        robot_nodes,
        mcp_nodes,
    ])
