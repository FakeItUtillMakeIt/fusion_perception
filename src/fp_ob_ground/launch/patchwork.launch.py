from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fp_ob_ground',
            executable='patchwork_ground_node',
            name='patchwork_ground',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
