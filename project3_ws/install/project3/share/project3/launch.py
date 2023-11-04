from launch import LaunchDescription
from launch_ros.actions import *
from launch.substitutions import *
from launch.actions import *

def generate_launch_description():

    #Commented out code is for recording results
    #Not needed until that part is done
    #Must put full path in bag_in arg

    bag_in = DeclareLaunchArgument('bag_in')
    #bag_out = DeclareLaunchArgument('bag_out')

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
        bag_in,
        #bag_out,
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_in')],
            output='screen',
            shell=True
        ),
        #ExecuteProcess(
            #cmd=['ros2', 'bag', 'record', LaunchConfiguration('bag_out')],
            #output='screen',
            #shell=True
        #)
    ])
