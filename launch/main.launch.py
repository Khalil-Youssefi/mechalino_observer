import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(get_package_share_directory('mechalino_observer'), 'config', 'params.yaml')
    rviz_config_path = os.path.join(get_package_share_directory('mechalino_observer'), 'config', 'rviz_config.rviz')
    cam2topic = Node(
        package='mechalino_observer',
        executable='cam2topic',
        name='cam2topic',
        output='screen',
        parameters=[params]
    )

    undistorted_img_pub = Node(
        package='mechalino_observer',
        executable='undistorted_img_pub',
        name='undistorted_img_pub',
        output='screen',
        parameters=[params]
    )

    pose_estimator = Node(
        package='mechalino_observer',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        parameters=[params]
    )

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]  # Load RViz config file
        )
    nodes = []
    nodes.append(cam2topic)
    nodes.append(undistorted_img_pub)
    nodes.append(pose_estimator)
    nodes.append(rviz2)

    return LaunchDescription(nodes)
