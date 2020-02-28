import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                        arguments=['-entity', 'thymio', '-topic', 'robot_description',
                                   '-x', '0', '-y', '0', '-z', '0'],
                        output='screen')
    return LaunchDescription([
        spawn_entity,
    ])
