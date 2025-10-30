from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_first_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            # Active sensor publisher (used in launch files)
            'multi_sensor_publisher = my_first_pkg.multi_sensor_publisher:main',

            # Core ROS2 person-following pipeline nodes
            'yolo_detector_node = my_first_pkg.yolo_detector_node:main',
            'person_tracker_node = my_first_pkg.person_tracker_node:main',
            'behavior_controller_node = my_first_pkg.behavior_controller_node:main',
            'robot_controller_node = my_first_pkg.robot_controller_node:main',
            'camera_servo_node = my_first_pkg.camera_servo_node:main',
        ],
    },
)
