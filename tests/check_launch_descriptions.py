"""Evaluate launch configuration without executing Node or simulator actions."""
import importlib.util
import sys
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

sys.dont_write_bytecode = True

for path in sorted(Path('src/legbot_bringup/launch').glob('*.launch.py')):
    spec = importlib.util.spec_from_file_location('candidate', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument) and action.default_value is not None:
            action.execute(context)
    if path.name in ('go2_mode.launch.py', 'unitree_lidar.launch.py'):
        context.launch_configurations['network_interface'] = next(
            (item.name for item in Path('/sys/class/net').iterdir() if item.name != 'lo'),
            'test_eth0')
    actions = list(description.entities)
    # setup only constructs actions: never call execute() on a Node or include.
    if path.name == 'simulation.launch.py':
        actions += module.setup(context)
    if path.name == 'real.launch.py':
        ethernet = next((item.name for item in Path('/sys/class/net').iterdir() if item.name != 'lo'), None)
        if ethernet:
            assert context.launch_configurations['policy_profile'] == 'moe_cts_77k'
            context.launch_configurations['network_interface'] = ethernet
            context.launch_configurations['policy_profile'] = 'himloco'
            # Compatibility environment + robot_state_publisher + manager + event handler.
            assert len(module.setup(context)) == 4
            context.launch_configurations['policy_profile'] = 'stairs_trot'
            stairs_real_actions = module.setup(context)
            manager = next(action for action in stairs_real_actions if isinstance(action, Node)
                           and action._Node__package == 'controller_manager')
            manager_parameters = evaluate_parameters(context, manager._Node__parameters)
            assert any('stairs_trot_controller.yaml' in str(item) for item in manager_parameters)
            for profile, filename in (
                    ('legged_gym', 'legged_gym_controller.yaml'),
                    ('go2_cts', 'go2_cts_controller.yaml'),
                    ('mjlab_flat', 'mjlab_flat_controller.yaml'),
                    ('moe_cts_77k', 'moe_cts_77k_controller.yaml')):
                context.launch_configurations['policy_profile'] = profile
                profile_actions = module.setup(context)
                profile_manager = next(
                    action for action in profile_actions if isinstance(action, Node)
                    and action._Node__package == 'controller_manager')
                profile_parameters = evaluate_parameters(
                    context, profile_manager._Node__parameters)
                assert any(filename in str(item) for item in profile_parameters)
            context.launch_configurations['enable_commands'] = 'true'
            context.launch_configurations['odom_topic'] = '/test/odom'
            context.launch_configurations['cloud_topic'] = '/test/cloud'
            command_actions = module.setup(context)
            adapter = next(action for action in command_actions if isinstance(action, Node)
                           and action._Node__package == 'legbot_bringup')
            adapter_parameters = evaluate_parameters(context, adapter._Node__parameters)
            assert any(item.get('odom_topic') == '/test/odom' and item.get('cloud_topic') == '/test/cloud'
                       and item.get('require_navigation_data') is True
                       and item.get('max_vx') == 0.20
                       for item in adapter_parameters if isinstance(item, dict))
    for action in actions:
        if isinstance(action, Node):
            evaluate_parameters(context, action._Node__parameters)
    if path.name in ('scan.launch.py', 'ego.launch.py'):
        assert sum(isinstance(action, Node) for action in actions) == 2
    if path.name == 'scan.launch.py':
        assert Path(context.launch_configurations['keypoints_yaml']).is_file()
        assert context.launch_configurations['odom_topic'] == '/fast_lio/odometry_base'
        assert context.launch_configurations['cloud_topic'] == '/fast_lio/cloud_registered'
        assert context.launch_configurations['sensor_pose_topic'] == '/fast_lio/odometry_lidar'
        assert context.launch_configurations['max_vel'] == '0.50'
        assert context.launch_configurations['max_vy'] == '0.22'
        assert context.launch_configurations['max_vyaw'] == '1.00'
        assert context.launch_configurations['max_acc'] == '0.30'
        assert context.launch_configurations['heading_error_threshold'] == '1.20'
        assert context.launch_configurations['rotate_exit_ratio'] == '0.75'
        assert context.launch_configurations['kp_yaw'] == '1.20'
        assert context.launch_configurations['steer_with_position_error'] == 'false'
        assert context.launch_configurations['min_walk_speed'] == '0.0'
        assert context.launch_configurations['goal_slowdown_distance'] == '0.80'
        assert context.launch_configurations['finish_dist'] == '0.40'
        assert context.launch_configurations['finish_dist_z'] == '1.5'
        assert context.launch_configurations['robot_radius'] == '0.20'
        assert context.launch_configurations['robot_half_length'] == '0.07'
        assert context.launch_configurations['inflation_z_up'] == '0.03'
        assert context.launch_configurations['inflation_z_down'] == '0.03'
        assert context.launch_configurations['lidar_horizontal_fov_deg'] == '240.0'
        assert context.launch_configurations['lidar_clear_outside_fov'] == 'true'
        assert context.launch_configurations['planning_horizon'] == '3.0'
        assert context.launch_configurations['replan_thresh'] == '0.2'
        adapter = next(
            action for action in actions
            if isinstance(action, Node)
            and str(action.node_executable) == 'scan_go2_cmd_adapter')
        adapter_parameters = evaluate_parameters(context, adapter._Node__parameters)
        assert any(item.get('use_goal_z') is False
                   for item in adapter_parameters if isinstance(item, dict))
    if path.name == 'scan_waypoints.launch.py':
        assert context.launch_configurations['input_topic'] == '/scan/waypoint_input'
        assert context.launch_configurations['mode'] == 'direct'
        assert context.launch_configurations['delete_topic'] == '/scan/waypoint_delete'
        assert context.launch_configurations['delete_radius'] == '0.50'
        assert context.launch_configurations['reach_z'] == '0.0'
        assert context.launch_configurations['max_reach_speed'] == '0.0'
        assert context.launch_configurations['reach_xy'] == '0.40'
        assert context.launch_configurations['dwell_seconds'] == '0.15'
    if path.name == 'go2_demo_control.launch.py':
        assert context.launch_configurations['odom_topic'] == '/fast_lio/odometry_base'
        assert context.launch_configurations['cloud_topic'] == '/fast_lio/cloud_registered'
        assert context.launch_configurations['localization_label'] == 'FAST-LIO'
        assert context.launch_configurations['goal_settle_seconds'] == '1.5'
    if path.name == 'scan_keypoint_recorder.launch.py':
        assert context.launch_configurations['odom_topic'] == '/fast_lio/odometry_base'
        assert context.launch_configurations['route_topic'] == '/scan/recorded_keypoints'
    if path.name == 'fastlio.launch.py':
        assert context.launch_configurations['sensor_frame'] == 'livox_imu_link'
        assert context.launch_configurations['lidar_frame'] == 'livox_link'
    if path.name == 'unitree_lidar.launch.py':
        assert context.launch_configurations['cloud_dds_topic'] == 'rt/utlidar/cloud'
        assert context.launch_configurations['use_source_stamp'] == 'false'
    if path.name == 'go2_mode.launch.py':
        assert context.launch_configurations['action'] == 'check'
        assert context.launch_configurations['confirm_action'] == 'false'
        assert context.launch_configurations['stand_down_first'] == 'true'
        assert context.launch_configurations['select_mode'] == 'normal'
    if path.name == 'simulation.launch.py':
        simulation_source = path.read_text()
        assert "'-topic', '/go2/robot_description'" in simulation_source
        assert "('robot_description', '/go2/robot_description')" in simulation_source
        assert "'third_party', 'libtorch', 'lib'" in simulation_source
        assert "'third_party', 'onnxruntime', 'lib'" in simulation_source
        assert context.launch_configurations['policy_profile'] == 'moe_cts_77k'
        assert context.launch_configurations['navigation_source'] == 'fastlio'
        assert context.launch_configurations['publish_ground_truth'] == 'false'
        assert context.launch_configurations['diagnose_fastlio'] == 'false'
        fastlio_actions = module.setup(context)
        for action in fastlio_actions:
            if isinstance(action, Node):
                evaluate_parameters(context, action._Node__parameters)
        legbot_parameter_sets = [
            evaluate_parameters(context, action._Node__parameters)
            for action in fastlio_actions
            if isinstance(action, Node) and action._Node__package == 'legbot_bringup']
        assert any(
            item.get('odom_topic') == '/fast_lio/odometry_base'
            and item.get('cloud_topic') == '/fast_lio/cloud_registered'
            and item.get('command_timeout') == 2.0
            and item.get('navigation_timeout') == 2.0
            for parameter_set in legbot_parameter_sets
            for item in parameter_set if isinstance(item, dict))
        context.launch_configurations['publish_ground_truth'] = 'true'
        context.launch_configurations['diagnose_fastlio'] = 'true'
        diagnostic_actions = module.setup(context)
        assert any(isinstance(action, Node)
                   and str(action.node_executable) == 'fastlio_truth_monitor'
                   for action in diagnostic_actions)
        # Publishing truth for evaluation must not silently switch command
        # gating away from FAST-LIO.
        diagnostic_parameter_sets = [
            evaluate_parameters(context, action._Node__parameters)
            for action in diagnostic_actions
            if isinstance(action, Node) and action._Node__package == 'legbot_bringup']
        assert any(
            item.get('odom_topic') == '/fast_lio/odometry_base'
            and item.get('cloud_topic') == '/fast_lio/cloud_registered'
            for parameter_set in diagnostic_parameter_sets
            for item in parameter_set if isinstance(item, dict))
        context.launch_configurations['publish_ground_truth'] = 'false'
        context.launch_configurations['diagnose_fastlio'] = 'false'
        context.launch_configurations['policy_profile'] = 'stairs_trot'
        stairs_actions = module.setup(context)
        robot_publishers = [action for action in stairs_actions if isinstance(action, Node)
                            and action._Node__package == 'robot_state_publisher']
        assert len(robot_publishers) == 1
        for profile in ('legged_gym', 'go2_cts', 'mjlab_flat', 'moe_cts_77k'):
            context.launch_configurations['policy_profile'] = profile
            profile_actions = module.setup(context)
            assert any(isinstance(action, Node)
                       and action._Node__package == 'robot_state_publisher'
                       for action in profile_actions)
    print('PASS launch description:', path.name)
