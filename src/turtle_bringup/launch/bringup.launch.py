#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('turtle_bringup')
    description_pkg = get_package_share_directory('turtle_description')
    ekf_pkg = get_package_share_directory('turtle_ekf')

    type_arg = DeclareLaunchArgument(
        'type',
        default_value='ekf',
        description='Type of rviz config to use: ekf or slam'
    )

    rviz_type = LaunchConfiguration('type')

    ekf_rviz_config = os.path.join(bringup_pkg, 'rviz', 'ekf.rviz')
    slam_rviz_config = os.path.join(bringup_pkg, 'rviz', 'slam.rviz')

    use_sim_time = {'use_sim_time': True}

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'robot_state_publisher.launch.py')
        )
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ekf_pkg, 'launch', 'ekf.launch.py')
        )
    )

    wheel_odometry = Node(
        package='turtle_odometry',
        executable='turtle_wheel_odometry.py',
        name='turtle_wheel_odometry',
        output='screen',
        parameters=[use_sim_time]
    )

    rviz2_ekf = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', ekf_rviz_config],
        output='screen',
        parameters=[use_sim_time],
        condition=IfCondition(PythonExpression(["'", rviz_type, "' == 'ekf'"]))
    )

    rviz2_slam = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', slam_rviz_config],
        output='screen',
        parameters=[use_sim_time],
        condition=IfCondition(PythonExpression(["'", rviz_type, "' == 'slam'"]))
    )

    turtle_path = Node(
        package='turtle_bringup',
        executable='turtle_path.py',
        name='turtle_path',
        output='screen',
        parameters=[use_sim_time]
    )

    return LaunchDescription([
        type_arg,
        robot_state_publisher,
        wheel_odometry,
        ekf_launch,
        turtle_path,
        rviz2_ekf,
        rviz2_slam,
    ])
