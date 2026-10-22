import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('fp_ob_perception')
    params = os.path.join(pkg, 'config', 'obstacle_params.yaml')

    ground_mode = LaunchConfiguration('ground_mode')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/lidar/points_raw'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('ground_mode', default_value='zband',
                              description='zband=原z带法(保留) | patchwork=Patchwork++地面分割'),
        DeclareLaunchArgument('semantics', default_value='false',
                              description='true=同时启动 segformer+打标+overlay 语义管线'),
        Node(
            package='fp_ob_perception',
            executable='pointcloud_preprocess_node',
            name='pointcloud_preprocess',
            output='screen',
            parameters=[params, {'input_topic': LaunchConfiguration('lidar_topic'),
                                 'use_sim_time': True}],
        ),
        Node(
            package='fp_ob_ground',
            executable='patchwork_ground_node',
            name='patchwork_ground',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'input_topic': LaunchConfiguration('lidar_topic'),
                         'sensor_height': 0.525,
                         'enable_TGR': False,
                         'th_seeds': 0.05,
                         'th_dist': 0.05,
                         'num_min_pts': 5}],
            condition=IfCondition(PythonExpression(["'", ground_mode, "' == 'patchwork'"])),
        ),
        Node(
            package='fp_ob_perception',
            executable='obstacle_detect_node',
            name='obstacle_detect',
            output='screen',
            parameters=[params, {'use_sim_time': True,
                                 'input_topic': PythonExpression([
                                     "'/fp_ob/points_non_ground_pw' if '", ground_mode,
                                     "' == 'patchwork' else '/fp_ob/points_non_ground'"])}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('fp_ob_perception'), 'launch', 'factory1_check.launch.py')),
            launch_arguments={'rviz': 'false'}.items(),
            condition=IfCondition(LaunchConfiguration('semantics')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'obstacle_check.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
