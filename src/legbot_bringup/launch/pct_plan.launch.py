from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tomogram', default_value='building2_9_go2'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='pct_planner', executable='pct_plan', output='screen',
             parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}],
             arguments=['--scene','Go2','--tomogram',LaunchConfiguration('tomogram')]),
    ])
