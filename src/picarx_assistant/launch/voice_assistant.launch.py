#!/usr/bin/env python3
"""
Voice Assistant Launch File

Launches the complete voice-controlled robot assistant system:
- Core robot nodes (from my_first_pkg)
- Voice I/O nodes (STT, TTS)
- Conversation orchestrator (Claude integration)
- ROS2 MCP gateway
- VLM node (optional)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    assistant_pkg = get_package_share_directory('picarx_assistant')
    robot_pkg = get_package_share_directory('my_first_pkg')

    # Declare launch arguments
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode (no hardware)'
    )

    stt_backend_arg = DeclareLaunchArgument(
        'stt_backend',
        default_value='whisper_local',
        description='STT backend: whisper_local, whisper_api'
    )

    tts_backend_arg = DeclareLaunchArgument(
        'tts_backend',
        default_value='piper',
        description='TTS backend: piper, openai, espeak'
    )

    enable_vlm_arg = DeclareLaunchArgument(
        'enable_vlm',
        default_value='true',
        description='Enable Vision-Language Model node'
    )

    api_key_arg = DeclareLaunchArgument(
        'anthropic_api_key',
        default_value='',
        description='Anthropic API key (or use ANTHROPIC_API_KEY env var)'
    )

    # Get launch configurations
    simulation = LaunchConfiguration('simulation')
    stt_backend = LaunchConfiguration('stt_backend')
    tts_backend = LaunchConfiguration('tts_backend')
    enable_vlm = LaunchConfiguration('enable_vlm')
    api_key = LaunchConfiguration('anthropic_api_key')

    # =========================================================================
    # CORE ROBOT NODES (from my_first_pkg)
    # =========================================================================

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
                'publish_rate': 5.0,  # Lower rate for assistant mode
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

    # =========================================================================
    # VOICE ASSISTANT NODES
    # =========================================================================

    voice_nodes = GroupAction([
        # Voice Input (Microphone + VAD)
        Node(
            package='picarx_assistant',
            executable='voice_input_node',
            name='voice_input',
            parameters=[{
                'vad_aggressiveness': 2,
                'min_speech_duration': 0.3,
                'silence_duration': 0.8,
            }]
        ),

        # Speech-to-Text
        Node(
            package='picarx_assistant',
            executable='stt_node',
            name='stt',
            parameters=[{
                'backend': stt_backend,
                'model': 'base.en',
                'language': 'en',
                'device': 'cpu',
            }]
        ),

        # Text-to-Speech
        Node(
            package='picarx_assistant',
            executable='tts_node',
            name='tts',
            parameters=[{
                'backend': tts_backend,
                'voice': 'en_US-lessac-medium',
                'speed': 1.0,
            }]
        ),
    ])

    # =========================================================================
    # AI/ORCHESTRATION NODES
    # =========================================================================

    ai_nodes = GroupAction([
        # Conversation Orchestrator (Claude integration)
        Node(
            package='picarx_assistant',
            executable='conversation_node',
            name='conversation_orchestrator',
            parameters=[{
                'api_key': api_key,
                'model': 'claude-sonnet-4-20250514',
                'max_tokens': 500,
                'temperature': 0.7,
            }]
        ),

        # ROS2 MCP Gateway
        Node(
            package='picarx_assistant',
            executable='ros2_mcp_server',
            name='mcp_gateway',
        ),

        # Vision-Language Model (optional)
        Node(
            package='picarx_assistant',
            executable='vlm_node',
            name='vlm',
            condition=IfCondition(enable_vlm),
            parameters=[{
                'backend': 'claude',
                'model': 'claude-sonnet-4-20250514',
                'api_key': api_key,
            }]
        ),
    ])

    return LaunchDescription([
        # Arguments
        simulation_arg,
        stt_backend_arg,
        tts_backend_arg,
        enable_vlm_arg,
        api_key_arg,

        # Node groups
        robot_nodes,
        voice_nodes,
        ai_nodes,
    ])
