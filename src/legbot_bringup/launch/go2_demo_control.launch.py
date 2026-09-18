"""Standalone GO2 stand/SCAN-plan/RL transition coordinator."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('crouch_wait_seconds', default_value='3.0'),
        DeclareLaunchArgument('stand_wait_seconds', default_value='5.0'),
        DeclareLaunchArgument('goal_settle_seconds', default_value='1.5'),
        DeclareLaunchArgument('odom_topic', default_value='/fast_lio/odometry_base'),
        DeclareLaunchArgument('cloud_topic', default_value='/fast_lio/cloud_registered'),
        DeclareLaunchArgument('localization_label', default_value='FAST-LIO'),
        Node(
            package='legbot_bringup', executable='go2_demo_sequencer',
            name='go2_demo_sequencer', output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'crouch_wait_seconds': ParameterValue(
                    LaunchConfiguration('crouch_wait_seconds'), value_type=float),
                'stand_wait_seconds': ParameterValue(
                    LaunchConfiguration('stand_wait_seconds'), value_type=float),
                'goal_settle_seconds': ParameterValue(
                    LaunchConfiguration('goal_settle_seconds'), value_type=float),
                'odom_topic': ParameterValue(
                    LaunchConfiguration('odom_topic'), value_type=str),
                'cloud_topic': ParameterValue(
                    LaunchConfiguration('cloud_topic'), value_type=str),
                'localization_label': ParameterValue(
                    LaunchConfiguration('localization_label'), value_type=str),
            }]),
    ])
