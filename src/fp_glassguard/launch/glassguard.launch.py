import os

FP_PYTHON = os.environ.get('FP_PYTHON', os.path.expanduser('~/miniforge3/envs/robot_ros2/bin/python3'))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('fp_glassguard')
    fs = get_package_share_directory('fp_factory_sim')
    sg = get_package_share_directory('fp_surface_guard')
    params = os.path.join(pkg, 'config', 'glassguard.yaml')
    guard_nav = os.path.join(sg, 'config', 'nav2_guard_params.yaml')
    prefix = os.path.dirname(os.path.dirname(pkg))
    node = os.path.join(prefix, 'bin', 'glassguard_node')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(fs, 'maps', 'factory1_map.yaml')),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument(
            'python', default_value=FP_PYTHON),
        ExecuteProcess(
            cmd=[LaunchConfiguration('python'), node, '--ros-args',
                 '-p', 'use_sim_time:=true', '--params-file', params],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fs, 'launch', 'navigation.launch.py')),
            condition=IfCondition(LaunchConfiguration('nav')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': guard_nav,
            }.items(),
        ),
    ])
