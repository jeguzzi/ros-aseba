# import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import xacro
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description() -> None:
    # TODO(wjwwood): Use a substitution to find share directory once this is implemented in launch
    # urdf = os.path.join(get_package_share_directory('dummy_robot_bringup'),
    #                     'models', 'single_rrbot.urdf')

    model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('thymio_description'), 'launch',
                         'model2.launch.py')))

    asebaros = Node(package='asebaros', node_executable='asebaros',
                    output='screen', arguments=[
                        launch.substitutions.LaunchConfiguration('device'),
                        '-p', launch.substitutions.LaunchConfiguration('port')])

    driver = Node(package='thymio_driver', node_executable='thymio_driver', output='screen')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'device', default_value=['ser:device=/dev/cu.usbmodem14201'],
            description='The device which dashel connects to'),
        launch.actions.DeclareLaunchArgument(
            'port', default_value=['33333'],
            description='The local port dashel bind to'),
        # launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('device')),
        # launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('port')),
        model,
        asebaros,
        driver,
    ])
