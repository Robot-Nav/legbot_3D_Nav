"""Standalone RViz waypoint queue for SCAN."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'waypoints',
            default_value=''),
        DeclareLaunchArgument('input_topic', default_value='/scan/waypoint_input'),
        DeclareLaunchArgument('mode', default_value='direct', choices=['queue', 'direct']),
        DeclareLaunchArgument('delete_topic', default_value='/scan/waypoint_delete'),
        DeclareLaunchArgument('delete_radius', default_value='0.50'),
        DeclareLaunchArgument('odom_topic', default_value='/fast_lio/odometry_base'),
        DeclareLaunchArgument('ready_topic', default_value='/go2/demo_ready'),
        DeclareLaunchArgument('output', default_value='/tmp/legbot_scan_waypoint_mission.json'),
        DeclareLaunchArgument('reach_xy', default_value='0.40'),
        DeclareLaunchArgument(
            'reach_z', default_value='0.0',
            description='Optional same-floor Z tolerance; <=0 ignores Z for RViz 2D goals'),
        DeclareLaunchArgument('dwell_seconds', default_value='0.15'),
        DeclareLaunchArgument('max_reach_speed', default_value='0.0',
                              description='Maximum XY speed at reach; <=0 disables the noisy velocity gate'),
        Node(
            package='legbot_bringup', executable='scan_waypoint_mission',
            name='scan_waypoint_mission', output='screen',
            arguments=[
                '--waypoints', LaunchConfiguration('waypoints'),
                '--input-topic', LaunchConfiguration('input_topic'),
                '--mode', LaunchConfiguration('mode'),
                '--delete-topic', LaunchConfiguration('delete_topic'),
                '--delete-radius', LaunchConfiguration('delete_radius'),
                '--output', LaunchConfiguration('output'),
                '--odom-topic', LaunchConfiguration('odom_topic'),
                '--ready-topic', LaunchConfiguration('ready_topic'),
                '--reach-xy', LaunchConfiguration('reach_xy'),
                '--reach-z', LaunchConfiguration('reach_z'),
                '--dwell-seconds', LaunchConfiguration('dwell_seconds'),
                '--max-reach-speed', LaunchConfiguration('max_reach_speed'),
                '--ack-timeout', '8.0',
                '--max-attempts', '5',
            ],
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }]),
    ])
