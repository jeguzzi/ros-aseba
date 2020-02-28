import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> None:
    urdf_xacro = os.path.join(get_package_share_directory('thymio_description'),
                              'urdf', 'thymio.urdf.xacro')
    opts, input_file_name = xacro.process_args([urdf_xacro])
    try:
        doc = xacro.process_file(input_file_name, **vars(opts))
    except Exception as e:
        print(e)
    robot_desc = doc.toprettyxml(indent='  ')

    # with open('/Users/Jerome/Desktop/thymio.urdf', 'w+') as f:
    #     f.write(robot_desc)

    params = {'robot_description': robot_desc}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            parameters=[params], output='screen'),
    ])
