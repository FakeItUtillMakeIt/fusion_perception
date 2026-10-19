import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def p2l(name, scan_out, band):
    return Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name=name,
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_in')),
            ('scan', scan_out),
        ],
        parameters=[{
            'use_sim_time': True,
            'transform_tolerance': 0.3,
            'min_height': band[0],
            'max_height': band[1],
            'angle_min': -3.1415,
            'angle_max': 3.1415,
            'angle_increment': 0.017453,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cloud_in', default_value='/lidar/points_raw'),
        # 高位切片(雷达平面±0.05m)：给 SLAM，看墙/机器等结构
        p2l('p2l_high', '/scan', (-0.05, 0.05)),
        # 低位切片(lidar系 z -0.75~-0.6，即绝对高度 0.05~0.2m)：
        # 用 -15° 线扫托盘顶面(0.15m)/线槽(0.06m)/下层箱，专供 costmap 避障
        p2l('p2l_low', '/scan_low', (-0.75, -0.6)),
    ])
