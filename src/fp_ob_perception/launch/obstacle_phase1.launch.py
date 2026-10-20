import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('fp_ob_perception')
    params = os.path.join(pkg, 'config', 'obstacle_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/lidar/points_raw'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='fp_ob_perception',
            executable='pointcloud_preprocess_node',
            name='pointcloud_preprocess',
            output='screen',
            parameters=[params, {'input_topic': LaunchConfiguration('lidar_topic'),
                                 'use_sim_time': True}],
        ),
        Node(
            package='fp_ob_perception',
            executable='obstacle_detect_node',
            name='obstacle_detect',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'obstacle_check.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
