import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('fp_factory_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml = os.path.join(pkg, 'maps', 'factory_map.yaml')
    params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_yaml,
                              description='slam 阶段 map_saver_cli 保存的 .yaml'),
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),

        # 定位：map_server + amcl（发布 map->odom）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'false',
            }.items(),
        ),
        # 导航：controller/planner/behavior/bt/waypoint/velocity_smoother
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'false',
            }.items(),
        ),
        # 统一激活管理器：放宽超时，规避 smoother/costmap 启动竞态
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_all',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'attempt_timeout': 120.0,
                'retry_timeout': 5.0,
                'node_names': [
                    'map_server', 'amcl',
                    'controller_server', 'smoother_server', 'planner_server',
                    'behavior_server', 'bt_navigator', 'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'factory_nav.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
