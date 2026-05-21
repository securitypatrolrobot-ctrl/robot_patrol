from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='robot_patrol',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen'
        ),

        Node(
            package='robot_patrol',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),

        Node(
            package='robot_patrol',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ),

        Node(
            package='robot_patrol',
            executable='failsafe_node',
            name='failsafe_node',
            output='screen'
        ),

        Node(
            package='robot_patrol',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        ),

        Node(
            package='robot_patrol',
            executable='lora_node',
            name='lora_node',
            output='screen'
        ),

    ])
