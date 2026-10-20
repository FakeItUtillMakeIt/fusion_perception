import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('fp_factory_sim')
    params_file = os.path.join(pkg, 'config', 'ekf.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True}],
        ),
    ])
