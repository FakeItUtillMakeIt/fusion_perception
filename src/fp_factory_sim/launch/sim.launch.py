import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('fp_factory_sim')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    xacro_file = LaunchConfiguration('xacro')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'factory.world')),
        DeclareLaunchArgument('xacro', default_value=os.path.join(pkg, 'urdf', 'factory_robot.xacro'),
                              description='机器人 xacro 文件路径（换 factory_robot1.xacro 用新车型）'),
        DeclareLaunchArgument('entity', default_value='factory_robot'),
        DeclareLaunchArgument('headless', default_value='true',
                              description='true=只跑 gzserver (WSL 友好); false=带 gzclient 窗口'),
        DeclareLaunchArgument('rviz', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': world,
                'gui': PythonExpression(["'false' if '", headless, "' == 'true' else 'true'"]),
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
                'use_sim_time': True,
            }],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', LaunchConfiguration('entity'),
                       '-x', '0', '-y', '0', '-z', '0.05'],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
        ),
    ])
