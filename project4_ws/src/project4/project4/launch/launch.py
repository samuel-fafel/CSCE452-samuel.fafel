from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    #NOT FINISHED
    
    return LaunchDescription([
        Node(
            package='project4',
            executable='simulator',
            name='simulator',
            output='screen'
        ),
        Node(
            package='project4',
            executable='velocity_translator',
            name='velocity_translator',
            output='screen'
        ),
        Node(
            package='project4',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        )
    ])
