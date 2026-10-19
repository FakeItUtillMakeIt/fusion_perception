import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('fp_bringup'), 'config')

    image_topic = LaunchConfiguration('image_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')
    calib = LaunchConfiguration('calib')
    policy = LaunchConfiguration('policy')
    grid_frame = LaunchConfiguration('grid_frame')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('lidar_topic', default_value='/hesai/points'),
        DeclareLaunchArgument('calib', default_value=os.path.join(cfg, 'extrinsics.yaml')),
        DeclareLaunchArgument('policy', default_value=os.path.join(cfg, 'policy.yaml')),
        DeclareLaunchArgument('grid_frame', default_value='odom'),
        Node(
            package='fp_perception', executable='segformer_node', name='fp_segformer',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'every_n': 1,
                'policy_path': policy,
            }],
        ),
        Node(
            package='fp_perception', executable='lidar_label_node', name='fp_lidar_label',
            output='screen',
            parameters=[{
                'lidar_topic': lidar_topic,
                'calib_path': calib,
                'policy_path': policy,
            }],
        ),
        Node(
            package='fp_perception', executable='overlay_node', name='fp_overlay',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'calib_path': calib,
                'policy_path': policy,
            }],
        ),
        Node(
            package='fp_perception', executable='traversability_grid', name='fp_traversability_grid',
            output='screen',
            parameters=[{
                'policy_path': policy,
                'grid_frame': grid_frame,
            }],
        ),
    ])
