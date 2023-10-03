from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_turtles', default_value='2', description='Number of turtles to use'),
        
        Node(
            package='your_package_name',  # Replace with your actual package name
            executable='tam_logo_drawer',  # Replace with your actual executable name
            name='tam_logo_drawer',
            namespace='',
            output='screen',
            parameters=[
                {'num_turtles': LaunchConfiguration('num_turtles')}
            ],
        ),
    ])

