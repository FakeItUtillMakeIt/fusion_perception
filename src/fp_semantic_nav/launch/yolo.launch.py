import os

FP_PYTHON = os.environ.get('FP_PYTHON', os.path.expanduser('~/miniforge3/envs/robot_ros2/bin/python3'))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('fp_semantic_nav')
    params = os.path.join(pkg, 'config', 'yolo_params.yaml')
    node = os.path.join(os.path.dirname(os.path.dirname(pkg)), 'lib', 'fp_semantic_nav', 'yolo_roi_node')
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='yolov8s-worldv2.pt'),
        DeclareLaunchArgument('python', default_value=FP_PYTHON,
                              description='显式解释器，避免 conda activate 偶发失效'),
        ExecuteProcess(
            cmd=[LaunchConfiguration('python'), node, '--ros-args',
                     '-p', 'use_sim_time:=true',
                     '--params-file', params],
            output='screen',
        ),
    ])
