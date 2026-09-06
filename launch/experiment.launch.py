import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('mechalino_observer')
    params_path = os.path.join(package_share, 'config', 'params.yaml')
    robot_count = LaunchConfiguration('N')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'N',
                default_value='1',
                description='Number of robots, starting at robot ID 15',
            ),
            Node(
                package='mechalino_observer',
                executable='experiment_supervisor',
                name='experiment_supervisor',
                output='screen',
                parameters=[
                    params_path,
                    {'N': ParameterValue(robot_count, value_type=int)},
                ],
            ),
        ]
    )
