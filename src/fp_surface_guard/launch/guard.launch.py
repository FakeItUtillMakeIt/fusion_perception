import os

FP_PYTHON = os.environ.get('FP_PYTHON', os.path.expanduser('~/miniforge3/envs/robot_ros2/bin/python3'))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('fp_surface_guard')
    fs = get_package_share_directory('fp_factory_sim')
    params = os.path.join(pkg, 'config', 'surface_guard.yaml')
    guard_nav = os.path.join(pkg, 'config', 'nav2_guard_params.yaml')
    prefix = os.path.dirname(os.path.dirname(pkg))
    node = os.path.join(prefix, 'lib', 'fp_surface_guard', 'surface_builder.py')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(fs, 'maps', 'factory1_map.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'python', default_value=FP_PYTHON),
        # 表面构建器（conda 解释器：依赖 cv2/numpy/fp_common）
        ExecuteProcess(
            cmd=[LaunchConfiguration('python'), node, '--ros-args',
                 '-p', 'use_sim_time:=true', '--params-file', params],
            output='screen',
        ),
        # 导航：costmap 已内嵌 SurfaceOverrideLayer 插件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fs, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': guard_nav,
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
