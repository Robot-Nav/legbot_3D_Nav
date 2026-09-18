"""Record current FAST-LIO odometry as a reusable multi-floor SCAN route."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_output = PathJoinSubstitution([
        EnvironmentVariable('LEGBOT_DATA_DIR', default_value='/tmp/legbot_data'),
        'routes', 'scan_keypoints.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('odom_topic', default_value='/fast_lio/odometry_base'),
        DeclareLaunchArgument('output', default_value=default_output),
        DeclareLaunchArgument('route_topic', default_value='/scan/recorded_keypoints'),
        DeclareLaunchArgument('autosave', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('load_existing', default_value='false', choices=['true', 'false']),
        Node(
            package='scan_planner', executable='scan_keypoint_recorder',
            name='scan_keypoint_recorder', output='screen', emulate_tty=True,
            arguments=[
                '--odom-topic', LaunchConfiguration('odom_topic'),
                '--output', LaunchConfiguration('output'),
                '--route-topic', LaunchConfiguration('route_topic'),
                '--autosave', LaunchConfiguration('autosave'),
                '--load-existing', LaunchConfiguration('load_existing'),
            ],
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
