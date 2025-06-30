from setuptools import find_packages, setup

package_name = 'my_first_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node = my_first_pkg.drive_node:main',
            'wheel_command_publisher = my_first_pkg.wheel_command_publisher:main',
            'wheel_command_listener_test = my_first_pkg.wheel_command_listener_test:main',
            'wheel_command_listener = my_first_pkg.wheel_command_listener:main',
            'sensor_reader = my_first_pkg.sensor_reader:main',
            'multi_sensor_publisher = my_first_pkg.multi_sensor_publisher:main',
            'camera_publisher = my_first_pkg.camera_publisher:main',
            'camera_subscriber = my_first_pkg.camera_subscriber:main',
            'autonomous_obstacle_avoidance = my_first_pkg.autonomous_obstacle_avoidance:main',
            'multi_sensor_subscriber = my_first_pkg.multi_sensor_subscriber:main',
        ],
    },
)
