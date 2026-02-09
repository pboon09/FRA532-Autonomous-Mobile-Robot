#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('turtle_bringup')
    description_pkg = get_package_share_directory('turtle_description')
    ekf_pkg = get_package_share_directory('turtle_ekf')

    rviz_config = os.path.join(bringup_pkg, 'rviz', 'icp.rviz')
    ekf_config = os.path.join(ekf_pkg, 'config', 'ekf_params.yaml')

    use_sim_time = {'use_sim_time': True}

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'robot_state_publisher.launch.py')
        )
    )

    ekf_node = Node(
        package='turtle_ekf',
        executable='turtle_ekf_node',
        name='turtle_ekf_node',
        output='screen',
        parameters=[ekf_config, use_sim_time, {'publish_tf': False}]
    )

    wheel_odometry = Node(
        package='turtle_odometry',
        executable='turtle_wheel_odometry.py',
        name='turtle_wheel_odometry',
        output='screen',
        parameters=[use_sim_time]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[use_sim_time]
    )

    turtle_path = Node(
        package='turtle_bringup',
        executable='turtle_path.py',
        name='turtle_path',
        output='screen',
        parameters=[use_sim_time]
    )

    icp_odometry = Node(
        package='turtle_icp',
        executable='turtle_icp_odom.py',
        name='turtle_icp_odom',
        output='screen',
        parameters=[use_sim_time, {'publish_tf': True}]
    )

    return LaunchDescription([
        robot_state_publisher,
        wheel_odometry,
        ekf_node,
        icp_odometry,
        turtle_path,
        rviz2,
    ])
