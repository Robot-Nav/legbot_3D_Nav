"""Check or explicitly release the GO2 high-level controller before LowCmd."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('network_interface',
                              description='Ethernet adapter connected to GO2 EDU'),
        DeclareLaunchArgument('action', default_value='check', choices=['check', 'release', 'select']),
        DeclareLaunchArgument('confirm_action', default_value='false', choices=['false', 'true']),
        DeclareLaunchArgument('stand_down_first', default_value='true', choices=['false', 'true']),
        DeclareLaunchArgument('select_mode', default_value='normal'),
        Node(
            package='hardware_unitree_sdk2', executable='go2_mode_manager',
            name='go2_mode_manager', output='screen',
            parameters=[{
                'network_interface': LaunchConfiguration('network_interface'),
                'action': LaunchConfiguration('action'),
                'confirm_action': ParameterValue(
                    LaunchConfiguration('confirm_action'), value_type=bool),
                'stand_down_first': ParameterValue(
                    LaunchConfiguration('stand_down_first'), value_type=bool),
                'select_mode': LaunchConfiguration('select_mode'),
            }]),
    ])
