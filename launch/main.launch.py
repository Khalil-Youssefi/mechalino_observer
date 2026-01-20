import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_share = get_package_share_directory('mechalino_observer')
    params_path = os.path.join(pkg_share, 'config', 'params.yaml')
    rviz_config_path = os.path.join(pkg_share, 'config', 'rviz_config_2.rviz')

    # ---- load arena_marker_xy from params.yaml ----
    with open(params_path, 'r') as f:
        params_yaml = yaml.safe_load(f)

    arena_xy = params_yaml['/**']['ros__parameters']['arena_marker_xy']
    arena_x = float(arena_xy[0])
    arena_y = float(arena_xy[1])

    # ---- STATIC TF: arena -> arena_ArUcoID ----
    arena_aruco_to_arena = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='arena_aruco_to_arena',
        output='screen',
        arguments=[
            str(-arena_x), str(-arena_y), '0.0',
            '0.0', '0.0', '0.0', '1.0',
            'arena_ArUcoID',
            'arena',
        ],
    )

    cam2topic = Node(
        package='mechalino_observer',
        executable='cam2topic',
        name='cam2topic',
        output='screen',
        parameters=[params_path]
    )

    undistorted_img_pub = Node(
        package='mechalino_observer',
        executable='undistorted_img_pub',
        name='undistorted_img_pub',
        output='screen',
        parameters=[params_path]
    )

    pose_estimator = Node(
        package='mechalino_observer',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        parameters=[params_path]
    )

    # odom_publisher = Node(
    #     package='mechalino_observer',
    #     executable='odom_publisher',
    #     name='odom_publisher',
    #     output='screen',
    #     parameters=[params_path]
    # )

    tf_pose_tcp_server = Node(
        package='mechalino_observer',
        executable='tf_pose_tcp_server',
        name='tf_pose_tcp_server',
        output='screen',
        parameters=[params_path]
    )

    TableMarkerPub = Node(
            package='mechalino_observer',
            executable='TableMarkerPub',
            name='TableMarkerPub',
            output='screen',
        parameters=[params_path]
        )
    
    visited_area_marker_pub_node = Node(
            package='mechalino_observer',
            executable='visited_area_marker_pub_node',
            name='visited_area_marker_pub_node',
            output='screen',
        parameters=[params_path]
        )

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]  # Load RViz config file
        )

    # pose_compare_test = Node(
    #         package='mechalino_observer',
    #         executable='pose_compare_test',
    #         name='pose_compare_test',
    #         output='screen',
    #     parameters=[params_path]
    #     )
    
    nodes = []
    nodes.append(cam2topic)
    # nodes.append(undistorted_img_pub)
    nodes.append(pose_estimator)
    nodes.append(arena_aruco_to_arena)
    # nodes.append(odom_publisher)
    nodes.append(tf_pose_tcp_server)
    nodes.append(TableMarkerPub)
    # nodes.append(pose_compare_test)
    nodes.append(visited_area_marker_pub_node)
    nodes.append(rviz2)

    return LaunchDescription(nodes)
