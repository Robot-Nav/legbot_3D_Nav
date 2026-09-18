from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pcd', default_value='building2_9.pcd'),
        Node(package='pct_planner', executable='pct_tomography', output='screen',
             arguments=['--scene','Go2','--pcd',LaunchConfiguration('pcd')]),
    ])
