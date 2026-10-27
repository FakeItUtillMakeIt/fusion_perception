import os

FP_PYTHON = os.environ.get('FP_PYTHON', os.path.expanduser('~/miniforge3/envs/robot_ros2/bin/python3'))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('fp_semantic_nav')
    prefix = os.path.dirname(os.path.dirname(pkg))
    node = os.path.join(prefix, 'lib', 'fp_semantic_nav', 'glass_detector_node')
    params = os.path.join(pkg, 'config', 'glass_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('python', default_value=FP_PYTHON),
        ExecuteProcess(
            cmd=[LaunchConfiguration('python'), node, '--ros-args', '-p', 'use_sim_time:=true',
                 '--params-file', params],
            output='screen',
        ),
    ])
