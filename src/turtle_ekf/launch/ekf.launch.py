import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('turtle_ekf'),
        'config',
        'ekf_params.yaml'
    )

    ekf_node = Node(
        package='turtle_ekf',
        executable='turtle_ekf_node',
        name='turtle_ekf_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': True}]
    )

    return LaunchDescription([
        ekf_node
    ])
