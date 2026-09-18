import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    actions = []
    config = os.path.join(get_package_share_directory("legbot_bringup"), "config", "ego.yaml")
    actions.append(DeclareLaunchArgument('map_size_x', default_value='50.0'))
    actions.append(DeclareLaunchArgument('map_size_y', default_value='50.0'))
    actions.append(DeclareLaunchArgument('map_size_z', default_value='50.0'))
    actions.append(DeclareLaunchArgument('planning_horizon', default_value='2.5'))
    actions.append(DeclareLaunchArgument('max_vel', default_value='0.75'))
    actions.append(DeclareLaunchArgument('max_acc', default_value='0.5'))
    actions.append(DeclareLaunchArgument('collision_clearance', default_value='0.35'))
    actions.append(DeclareLaunchArgument('robot_clearance_x', default_value='0.25'))
    actions.append(DeclareLaunchArgument('robot_clearance_y', default_value='0.25'))
    actions.append(DeclareLaunchArgument('robot_clearance_z', default_value='0.15'))
    actions.append(DeclareLaunchArgument('odom_topic', default_value='/go2/odom'))
    actions.append(DeclareLaunchArgument('cloud_topic', default_value='/go2/cloud_world'))
    actions.append(DeclareLaunchArgument('bspline_topic', default_value='/planning/bspline'))
    actions.append(DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'))
    actions.append(DeclareLaunchArgument('occupancy_topic', default_value='/grid_map/occupancy'))
    actions.append(DeclareLaunchArgument('occupancy_inflate_topic', default_value='/grid_map/occupancy_inflate'))
    actions.append(DeclareLaunchArgument('optimal_path_topic', default_value='/ego_planner_node/optimal_list'))
    actions.append(DeclareLaunchArgument('goal_topic', default_value='/ego_planner_node/goal_point'))
    actions.append(DeclareLaunchArgument('global_path_topic', default_value='/pct_path'))
    actions.append(DeclareLaunchArgument('use_sim_time', default_value='false'))
    actions.append(DeclareLaunchArgument('frame_id', default_value='map'))
    actions.append(Node(package='ego_planner', executable='traj_server', name='traj_server', output="screen",
        parameters=[config, {
            'odometry_topic': ParameterValue(LaunchConfiguration('odom_topic'), value_type=str),
            'global_path_topic': ParameterValue(LaunchConfiguration('global_path_topic'), value_type=str),
            'traj_server.max_command_speed': ParameterValue(LaunchConfiguration('max_vel'), value_type=float),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'body_pose_topic': ParameterValue(LaunchConfiguration('odom_topic'), value_type=str),
        }], remappings=[
            ('/planning/bspline', LaunchConfiguration('bspline_topic')),
            ('/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ('/control_point_state_', '/ego/control_point_state'),
            ('/position_cmd', 'planning/pos_cmd'),
            ('/odom_world', LaunchConfiguration('odom_topic')),
        ]))
    actions.append(Node(package='ego_planner', executable='ego_planner_node', name='ego_planner_node', output="screen",
        parameters=[config, {
            'fsm.odometry_topic': ParameterValue(LaunchConfiguration('odom_topic'), value_type=str),
            'fsm.planning_horizon': ParameterValue(LaunchConfiguration('planning_horizon'), value_type=float),
            'grid_map.map_size_x': ParameterValue(LaunchConfiguration('map_size_x'), value_type=float),
            'grid_map.map_size_y': ParameterValue(LaunchConfiguration('map_size_y'), value_type=float),
            'grid_map.map_size_z': ParameterValue(LaunchConfiguration('map_size_z'), value_type=float),
            'grid_map.robot_clearance_x': ParameterValue(LaunchConfiguration('robot_clearance_x'), value_type=float),
            'grid_map.robot_clearance_y': ParameterValue(LaunchConfiguration('robot_clearance_y'), value_type=float),
            'grid_map.robot_clearance_z': ParameterValue(LaunchConfiguration('robot_clearance_z'), value_type=float),
            'grid_map.frame_id': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
            'manager.max_vel': ParameterValue(LaunchConfiguration('max_vel'), value_type=float),
            'manager.max_acc': ParameterValue(LaunchConfiguration('max_acc'), value_type=float),
            'manager.planning_horizon': ParameterValue(LaunchConfiguration('planning_horizon'), value_type=float),
            'optimization.dist0': ParameterValue(LaunchConfiguration('collision_clearance'), value_type=float),
            'optimization.max_vel': ParameterValue(LaunchConfiguration('max_vel'), value_type=float),
            'optimization.max_acc': ParameterValue(LaunchConfiguration('max_acc'), value_type=float),
            'bspline.limit_vel': ParameterValue(LaunchConfiguration('max_vel'), value_type=float),
            'bspline.limit_acc': ParameterValue(LaunchConfiguration('max_acc'), value_type=float),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'body_pose_topic': ParameterValue(LaunchConfiguration('odom_topic'), value_type=str),
        }], remappings=[
            ('/ego_planner_node/optimal_list', LaunchConfiguration('optimal_path_topic')),
            ('/ego_planner_node/goal_point', LaunchConfiguration('goal_topic')),
            ('/pct_path', LaunchConfiguration('global_path_topic')),
            ('/planning/bspline', LaunchConfiguration('bspline_topic')),
            ('/planning/data_display', '/ego/planning/data_display'),
            ('/grid_map/occupancy', LaunchConfiguration('occupancy_topic')),
            ('/grid_map/occupancy_inflate', LaunchConfiguration('occupancy_inflate_topic')),
            ('/grid_map/unknown', '/ego/grid_map/unknown'),
            ('/odom_world', LaunchConfiguration('odom_topic')),
            ('/grid_map/odom', LaunchConfiguration('odom_topic')),
            ('/grid_map/cloud', LaunchConfiguration('cloud_topic')),
            ('/grid_map/pose', '/pcl_render_node/camera_pose'),
            ('/grid_map/depth', '/pcl_render_node/depth'),
        ]))
    return LaunchDescription(actions)
