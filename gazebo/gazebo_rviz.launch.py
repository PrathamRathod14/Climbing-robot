import os
import re
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = '/mnt/c/Users/prath/Desktop/geckogrip-climber/external/go2_description/urdf/go2_description.urdf'
    rviz_config_path = '/mnt/c/Users/prath/Desktop/geckogrip-climber/external/go2_description/config/go2_description.rviz'
    bridge_path = '/mnt/c/Users/prath/Desktop/geckogrip-climber/gazebo/gazebo_rviz_bridge.py'

    # Read and replace package path with absolute local path in the URDF content
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    
    # Remap package paths to the real local mesh folder so RViz can load the model.
    urdf_content = urdf_content.replace(
        'package://go2_description',
        'file:///mnt/c/Users/prath/Desktop/geckogrip-climber/external/go2_description')
    urdf_content = re.sub(
        r'(<link\s+name="base_link">\s*)<inertial>.*?</inertial>',
        r'\1',
        urdf_content,
        count=1,
        flags=re.DOTALL)

    return LaunchDescription([
        # Robot State Publisher under namespace 'robot0'
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='robot0',
            output='screen',
            parameters=[{
                'robot_description': urdf_content,
                'publish_frequency': 100.0,
                'frame_prefix': 'robot0/'
            }]
        ),

        # RViz2 pre-loaded with the configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),

        # Gazebo to RViz Bridge Node
        ExecuteProcess(
            cmd=[bridge_path],
            output='screen'
        )
    ])
