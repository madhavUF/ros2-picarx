from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_pkg',
            executable='camera_publisher',
            name='camera_publisher'
        ),
        Node(
            package='my_first_pkg',
            executable='camera_subscriber',
            name='camera_subscriber'
        ),
        # You can uncomment and add more nodes below when ready
        # Node(
        #     package='my_first_pkg',
        #     executable='multi_sensor_publisher',
        #     name='multi_sensor_publisher'
        # ),
        # Node(
        #     package='my_first_pkg',
        #     executable='drive_node',
        #     name='drive_node'
        # ),
    ])
