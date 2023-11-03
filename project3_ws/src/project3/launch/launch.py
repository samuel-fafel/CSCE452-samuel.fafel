from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project3',
            executable='detector',
            name='people_detector',
            output='screen'
        ),
        Node(
            package='project3',
            executable='counter',
            name='people_counter',
            output='screen'
        ),
    ])
