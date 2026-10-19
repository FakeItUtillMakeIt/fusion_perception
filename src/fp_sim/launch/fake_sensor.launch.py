import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    truth = os.path.join(os.getcwd(), 'sim_truth.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('max_range', default_value='20.0'),
        DeclareLaunchArgument('board_motion', default_value='true'),
        DeclareLaunchArgument('publish_image', default_value='true'),
        Node(
            package='fp_sim', executable='fake_sensor_node', name='fp_fake_sensor',
            output='screen',
            parameters=[{
                'max_range': LaunchConfiguration('max_range'),
                'board_motion': ParameterValue(LaunchConfiguration('board_motion'), value_type=bool),
                'publish_image': ParameterValue(LaunchConfiguration('publish_image'), value_type=bool),
                'truth_out': truth,
            }],
        ),
    ])
