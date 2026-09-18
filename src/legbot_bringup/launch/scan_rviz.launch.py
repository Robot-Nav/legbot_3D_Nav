"""RViz2 view for FAST-LIO localization and SCAN local planning."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('legbot_bringup'), 'rviz', 'scan_fastlio_flat.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='rviz2', executable='rviz2', name='scan_rviz', output='screen',
            arguments=['-d', LaunchConfiguration('config')],
            parameters=[{'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
