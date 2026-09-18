import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    actions = []
    actions.append(DeclareLaunchArgument("sensor_pose_topic", default_value="/fast_lio/odometry_lidar"))
    config = os.path.join(get_package_share_directory("legbot_bringup"), "config", "scan.yaml")
    empty_keypoints = os.path.join(
        get_package_share_directory('legbot_bringup'), 'config', 'scan_keypoints_empty.yaml')
    actions.append(DeclareLaunchArgument('odom_topic', default_value='/fast_lio/odometry_base'))
    actions.append(DeclareLaunchArgument('cloud_topic', default_value='/fast_lio/cloud_registered'))
    actions.append(DeclareLaunchArgument('global_path_topic', default_value='/pct_path'))
    actions.append(DeclareLaunchArgument('frame_id', default_value='odom'))
    actions.append(DeclareLaunchArgument('lidar_horizontal_fov_deg', default_value='240.0'))
    actions.append(DeclareLaunchArgument(
        'lidar_clear_outside_fov', default_value='true', choices=['true', 'false']))
    actions.append(DeclareLaunchArgument('max_vel', default_value='0.50'))
    actions.append(DeclareLaunchArgument('max_vy', default_value='0.22'))
    actions.append(DeclareLaunchArgument('max_vyaw', default_value='1.00'))
    actions.append(DeclareLaunchArgument('heading_error_threshold', default_value='1.20'))
    actions.append(DeclareLaunchArgument('rotate_exit_ratio', default_value='0.75'))
    actions.append(DeclareLaunchArgument('kp_pos', default_value='0.70'))
    actions.append(DeclareLaunchArgument('kp_yaw', default_value='1.20'))
    actions.append(DeclareLaunchArgument('steer_with_position_error', default_value='false', choices=['true', 'false']))
    actions.append(DeclareLaunchArgument('min_stair_speed', default_value='0.0'))
    # Leave policy dead-zone compensation disabled by default.  It must be
    # measured for each replacement policy before it is enabled.
    actions.append(DeclareLaunchArgument('min_walk_speed', default_value='0.0'))
    actions.append(DeclareLaunchArgument('goal_slowdown_distance', default_value='0.80'))
    actions.append(DeclareLaunchArgument('max_acc', default_value='0.30'))
    actions.append(DeclareLaunchArgument('finish_dist', default_value='0.40'))
    actions.append(DeclareLaunchArgument(
        'finish_dist_z', default_value='1.5',
        description='Same-floor Z tolerance; waypoint arrival itself is decided in XY'))
    actions.append(DeclareLaunchArgument('map_resolution', default_value='0.1'))
    actions.append(DeclareLaunchArgument('inflation_z_up', default_value='0.03'))
    actions.append(DeclareLaunchArgument('inflation_z_down', default_value='0.03'))
    actions.append(DeclareLaunchArgument('robot_radius', default_value='0.20'))
    actions.append(DeclareLaunchArgument('robot_half_length', default_value='0.07'))
    actions.append(DeclareLaunchArgument('turn_slowdown_angle', default_value='0.30'))
    actions.append(DeclareLaunchArgument('min_turn_speed_scale', default_value='0.30'))
    actions.append(DeclareLaunchArgument('cmd_accel_limit', default_value='0.75'))
    actions.append(DeclareLaunchArgument('cmd_yaw_accel_limit', default_value='2.00'))
    actions.append(DeclareLaunchArgument('planning_horizon', default_value='3.0'))
    actions.append(DeclareLaunchArgument('replan_thresh', default_value='0.2'))
    actions.append(DeclareLaunchArgument('no_replan_thresh', default_value='0.10'))
    actions.append(DeclareLaunchArgument('enable_collision_check', default_value='true'))
    actions.append(DeclareLaunchArgument('reference_mode', default_value='piecewise_linear'))
    actions.append(DeclareLaunchArgument('z_projection_weight', default_value='8.0'))
    actions.append(DeclareLaunchArgument('use_sim_time', default_value='false'))
    actions.append(DeclareLaunchArgument('navi_mode', default_value='1'))
    actions.append(DeclareLaunchArgument('keypoints_yaml', default_value=empty_keypoints))
    actions.append(DeclareLaunchArgument('manual_goal_use_message_z', default_value='false', choices=['true', 'false']))
    actions.append(DeclareLaunchArgument('odom_twist_in_body_frame', default_value='true', choices=['true', 'false']))
    actions.append(Node(package='scan_planner', executable='scan_planner_node', name='scan_planner_node', output="screen",
        parameters=[config, LaunchConfiguration('keypoints_yaml'), {
            'fsm.global_reference_mode': ParameterValue(LaunchConfiguration('reference_mode'), value_type=str),
            'fsm.planning_horizon': ParameterValue(LaunchConfiguration('planning_horizon'), value_type=float),
            'fsm.thresh_replan': ParameterValue(LaunchConfiguration('replan_thresh'), value_type=float),
            'fsm.thresh_no_replan': ParameterValue(LaunchConfiguration('no_replan_thresh'), value_type=float),
            'fsm.z_projection_weight': ParameterValue(LaunchConfiguration('z_projection_weight'), value_type=float),
            'grid_map.frame_id': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
            'grid_map.lidar_horizontal_fov_deg': ParameterValue(
                LaunchConfiguration('lidar_horizontal_fov_deg'), value_type=float),
            'grid_map.lidar_clear_outside_fov': ParameterValue(
                LaunchConfiguration('lidar_clear_outside_fov'), value_type=bool),
            'grid_map.collision_check_enabled': ParameterValue(LaunchConfiguration('enable_collision_check'), value_type=bool),
            'grid_map.resolution': ParameterValue(LaunchConfiguration('map_resolution'), value_type=float),
            'grid_map.obstacles_inflation_z_up': ParameterValue(LaunchConfiguration('inflation_z_up'), value_type=float),
            'grid_map.obstacles_inflation_z_down': ParameterValue(LaunchConfiguration('inflation_z_down'), value_type=float),
            'grid_map.double_cylinder_radius': ParameterValue(LaunchConfiguration('robot_radius'), value_type=float),
            'grid_map.double_cylinder_offset': ParameterValue(LaunchConfiguration('robot_half_length'), value_type=float),
            'manager.max_vel': ParameterValue(LaunchConfiguration('max_vel'), value_type=float),
            'manager.max_acc': ParameterValue(LaunchConfiguration('max_acc'), value_type=float),
            'manager.planning_horizon': ParameterValue(LaunchConfiguration('planning_horizon'), value_type=float),
            'optimization.max_vel': ParameterValue(LaunchConfiguration('max_vel'), value_type=float),
            'optimization.max_acc': ParameterValue(LaunchConfiguration('max_acc'), value_type=float),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'body_pose_topic': ParameterValue(LaunchConfiguration('odom_topic'), value_type=str),
            'fsm.navi_mode': ParameterValue(LaunchConfiguration('navi_mode'), value_type=int),
            'fsm.manual_goal_use_message_z': ParameterValue(LaunchConfiguration('manual_goal_use_message_z'), value_type=bool),
            'fsm.odom_twist_in_body_frame': ParameterValue(LaunchConfiguration('odom_twist_in_body_frame'), value_type=bool),
            'fsm.finish_dist': ParameterValue(LaunchConfiguration('finish_dist'), value_type=float),
            'fsm.finish_dist_z': ParameterValue(LaunchConfiguration('finish_dist_z'), value_type=float),
        }], remappings=[
            ('/scan_planner_node/optimal_list', '/local_planner/optimal_path'),
            ('/scan_planner_node/goal_point', '/local_planner/goal'),
            ('/scan_planner_node/manual_ground_goal', '/scan/manual_ground_goal'),
            ('/scan_planner_node/local_target', '/scan/local_target'),
            ('/scan_planner_node/heading_vector', '/scan/heading_vector'),
            ('/grid_map/body_pose', LaunchConfiguration('odom_topic')),
            ('/grid_map/sensor_pose', LaunchConfiguration('sensor_pose_topic')),
            ('/grid_map/cloud', LaunchConfiguration('cloud_topic')),
            ('/initial_path', LaunchConfiguration('global_path_topic')),
            ('/planning/bspline', '/scan/planning/bspline'),
            ('/planning/data_display', '/scan/planning/data_display'),
            ('/planning/go2_execution_frozen', '/scan/planning/execution_frozen'),
            ('/grid_map/occupancy', '/local_planner/grid_map/occupancy'),
            ('/grid_map/occupancy_inflate', '/local_planner/grid_map/occupancy_inflate'),
            ('/grid_map/sliding_map_bbox', '/scan/grid_map/sliding_map_bbox'),
            ('/grid_map/unknown', '/scan/grid_map/unknown'),
            ('/grid_map/depth_cloud', '/scan/grid_map/depth_cloud'),
            ('/grid_map/sensor_pose_extrinsic', '/scan/grid_map/sensor_pose_extrinsic'),
            ('/move_base_simple/goal', '/scan/goal'),
        ]))
    actions.append(Node(package='scan_planner', executable='scan_go2_cmd_adapter', name='scan_go2_cmd_adapter', output="screen",
        parameters=[config, {
            'max_vx': ParameterValue(LaunchConfiguration('max_vel'), value_type=float),
            'max_vy': ParameterValue(LaunchConfiguration('max_vy'), value_type=float),
            'max_vyaw': ParameterValue(LaunchConfiguration('max_vyaw'), value_type=float),
            'heading_error_threshold': ParameterValue(LaunchConfiguration('heading_error_threshold'), value_type=float),
            'rotate_exit_ratio': ParameterValue(LaunchConfiguration('rotate_exit_ratio'), value_type=float),
            'kp_pos': ParameterValue(LaunchConfiguration('kp_pos'), value_type=float),
            'kp_yaw': ParameterValue(LaunchConfiguration('kp_yaw'), value_type=float),
            'steer_with_position_error': ParameterValue(LaunchConfiguration('steer_with_position_error'), value_type=bool),
            'min_stair_speed': ParameterValue(LaunchConfiguration('min_stair_speed'), value_type=float),
            'min_walk_speed': ParameterValue(LaunchConfiguration('min_walk_speed'), value_type=float),
            'goal_slowdown_distance': ParameterValue(LaunchConfiguration('goal_slowdown_distance'), value_type=float),
            'turn_slowdown_angle': ParameterValue(LaunchConfiguration('turn_slowdown_angle'), value_type=float),
            'min_turn_speed_scale': ParameterValue(LaunchConfiguration('min_turn_speed_scale'), value_type=float),
            'cmd_accel_limit': ParameterValue(LaunchConfiguration('cmd_accel_limit'), value_type=float),
            'cmd_yaw_accel_limit': ParameterValue(LaunchConfiguration('cmd_yaw_accel_limit'), value_type=float),
            'finish_dist': ParameterValue(LaunchConfiguration('finish_dist'), value_type=float),
            'finish_dist_z': ParameterValue(LaunchConfiguration('finish_dist_z'), value_type=float),
            'use_goal_z': ParameterValue(LaunchConfiguration('manual_goal_use_message_z'), value_type=bool),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'body_pose_topic': ParameterValue(LaunchConfiguration('odom_topic'), value_type=str),
        }], remappings=[
            ('/planning/bspline', '/scan/planning/bspline'),
            ('/cmd_vel', '/cmd_vel'),
            ('/planning/go2_execution_frozen', '/scan/planning/execution_frozen'),
        ]))
    return LaunchDescription(actions)
