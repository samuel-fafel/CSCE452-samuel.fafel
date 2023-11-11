from launch import LaunchDescription
from launch_ros.actions import *
from launch.substitutions import *
from launch.actions import *
from launch.event_handlers import *
from launch.events import *

# The load_disc_robot method reads a file that describes a disc-shaped robot
# and returns a dictionary describing the properties of that robot.

#disc_robot.py, errors with imports
import yaml

def load_disc_robot(file_name):
    with open(file_name) as f:
        robot = yaml.safe_load(f)
    robot['urdf'] = disc_robot_urdf(robot)
    return robot

def disc_robot_urdf(robot):
    radius = robot['body']['radius']
    height = robot['body']['height']

    return f"""<?xml version="1.0"?>
                  <robot name="disc">
                      <material name="light_blue">
                          <color rgba="0.5 0.5 1 1"/>
                      </material>
                      <material name="dark_blue">
                          <color rgba="0.1 0.1 1 1"/>
                      </material>
                      <material name="dark_red">
                          <color rgba="1 0.1 0.1 1"/>
                      </material>
                      <link name="base_link">
                          <visual>
                              <geometry>
                                  <cylinder length="{height}" radius="{radius}"/>
                              </geometry>
                              <material name="light_blue"/>
                          </visual>
                      </link>
                      <link name="heading_box">
                          <visual>
                              <geometry>
                                  <box size="{0.9*radius} {0.2*radius} {1.2*height}"/>
                              </geometry>
                              <material name="dark_blue"/>
                          </visual>
                      </link>
                      <link name="laser" />
                      <joint name="base_to_heading_box" type="fixed">
                          <parent link="base_link"/>
                          <child link="heading_box"/>
                          <origin xyz='{0.45*radius} 0.0 0.0'/>
                      </joint>
                      <joint name="base_to_laser" type="fixed">
                          <parent link="base_link"/>
                          <child link="laser"/>
                          <origin xyz="{0.5*radius} 0.0 0.0"/>
                      </joint>
                  </robot>
                  """

def generate_launch_description():

    robot = load_disc_robot('ideal.robot') #HARDCODED

    robot_urdf = robot['urdf']

    #Must put full path in bag_in arg
    bag_in = DeclareLaunchArgument('bag_in')
    bag_out = DeclareLaunchArgument('bag_out')

    bag_play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_in')],
            output='screen',
            shell=True
        )
    
    bag_record = ExecuteProcess(
            #Put desired topics after LaunchConfiguration (add any I may have missed)
            cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_out'), 
                '/robot_description', 
                '/robot_state_publisher', 
                '/cmd_vel',
                '/vl', 
                '/vr', 
                '/tf',
                '/tf_static'
            ],
            output='screen',
            shell=True
        ) 

    #event_handler = OnProcessExit(target_action = bag_play, on_exit = [EmitEvent(event=Shutdown())])

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
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description' : robot_urdf}]
        ),
        bag_in,
        bag_out,
        bag_play,
        bag_record,
        #event_handler
    ])
