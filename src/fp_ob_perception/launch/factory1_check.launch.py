import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('fp_ob_perception')
    bringup = get_package_share_directory('fp_bringup')
    calib = os.path.join(pkg, 'config', 'sim_truth_robot1.yaml')
    policy = os.path.join(bringup, 'config', 'policy.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/lidar/points_raw'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # 复用已验收的感知链路：segformer 分割 -> 逐点打标 -> 可通行栅格 -> 画面叠加
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'fusion.launch.py')),
            launch_arguments={
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'image_topic': LaunchConfiguration('image_topic'),
                'calib': calib,
                'policy': policy,
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'factory1_check.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
