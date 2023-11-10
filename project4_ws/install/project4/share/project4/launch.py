from launch import LaunchDescription
from launch_ros.actions import *
from launch.substitutions import *
from launch.actions import *
from launch.event_handlers import *
from launch.events import *

def generate_launch_description():

    #Must put full path in bag_in arg
    bag_in = DeclareLaunchArgument('bag_in')
    bag_out = DeclareLaunchArgument('bag_out')

    bag_play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_in')],
            output='screen',
            shell=True
        )
    
    bag_record = ExecuteProcess(
            #Put desired topics after LaunchConfiguration
            cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_out'), '/scan', '/person_locations', '/person_count', '/object_locations'],
            output='screen',
            shell=True
        ) 

    event_handler = OnProcessExit(target_action = bag_play, on_exit = [EmitEvent(event=Shutdown())])
    
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
        ),
        bag_in,
        bag_out,
        bag_play,
        bag_record,
        RegisterEventHandler(event_handler)
    ])
