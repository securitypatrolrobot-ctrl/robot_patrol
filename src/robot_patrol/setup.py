from setuptools import setup
import os
from glob import glob

package_name = 'robot_patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        'robot_patrol.failsafe',
        'robot_patrol.vision',
        'robot_patrol.navigation',
        'robot_patrol.communication',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='patrol',
    maintainer_email='patrol@patrol.com',
    description='Robot Patrol ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ultrasonic_node = robot_patrol.ros_ultrasonic_node:main',
            'lidar_node      = robot_patrol.ros_lidar_node:main',
            'vision_node     = robot_patrol.ros_vision_node:main',
            'lora_node       = robot_patrol.ros_lora_node:main',
            'navigation_node = robot_patrol.ros_navigation_node:main',
            'failsafe_node   = robot_patrol.ros_failsafe_node:main',
        ],
    },
)
