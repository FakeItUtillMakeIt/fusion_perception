import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('fp_semantic_nav')
    fs = get_package_share_directory('fp_factory_sim')
    glass_params = os.path.join(pkg, 'config', 'nav2_glass_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(fs, 'maps', 'factory1_map.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
        # 玻璃检测器（语义->地脚线->点云）
        Node(
            package='fp_semantic_nav',
            executable='glass_detector_node',
            name='glass_detector',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        # 复用现有导航 launch，仅替换参数文件（glass 作为 costmap 第三观测源）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fs, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': glass_params,
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
