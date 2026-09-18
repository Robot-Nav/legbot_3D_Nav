"""Bridge the GO2 EDU built-in Unitree lidar DDS stream into ROS 2 topics."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('network_interface',
                              description='Ethernet adapter connected to GO2 EDU'),
        DeclareLaunchArgument('cloud_dds_topic', default_value='rt/utlidar/cloud'),
        DeclareLaunchArgument('imu_dds_topic', default_value='rt/utlidar/imu'),
        DeclareLaunchArgument('cloud_frame', default_value='utlidar_lidar'),
        DeclareLaunchArgument('imu_frame', default_value='utlidar_imu'),
        DeclareLaunchArgument('use_source_stamp', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('preserve_source_timing', default_value='true',
                              choices=['true', 'false']),
        Node(
            package='hardware_unitree_sdk2', executable='unitree_lidar_bridge',
            name='unitree_lidar_bridge', output='screen',
            parameters=[{
                'network_interface': LaunchConfiguration('network_interface'),
                'cloud_dds_topic': LaunchConfiguration('cloud_dds_topic'),
                'imu_dds_topic': LaunchConfiguration('imu_dds_topic'),
                'cloud_frame': LaunchConfiguration('cloud_frame'),
                'imu_frame': LaunchConfiguration('imu_frame'),
                'use_source_stamp': ParameterValue(
                    LaunchConfiguration('use_source_stamp'), value_type=bool),
                'preserve_source_timing': ParameterValue(
                    LaunchConfiguration('preserve_source_timing'), value_type=bool),
            }]),
    ])
