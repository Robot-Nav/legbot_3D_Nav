"""URDF/SDF parsing only: no Gazebo server, ROS node or robot connection."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

share = Path(get_package_share_directory('go2_description'))
bringup_share = Path(get_package_share_directory('legbot_bringup'))
hardware_prefix = Path(get_package_prefix('hardware_unitree_sdk2'))
for executable in ('unitree_lidar_bridge', 'go2_mode_manager'):
    assert (hardware_prefix/'lib/hardware_unitree_sdk2'/executable).is_file()
scan_config = yaml.safe_load((bringup_share/'config/scan.yaml').read_text())
scan_fsm = scan_config['scan_planner_node']['ros__parameters']
scan_adapter = scan_config['scan_go2_cmd_adapter']['ros__parameters']
scan_fsm_source = Path(
    'src/scan_planner/algorithm/plan_manage/src/scan_replan_fsm.cpp').read_text()
scan_adapter_source = Path(
    'src/scan_planner/algorithm/plan_manage/src/go2_cmd_adapter.cpp').read_text()
rl_controller_source = Path(
    'src/rl_quadruped_controller/src/RlQuadrupedController.cpp').read_text()
grid_map_source = Path(
    'src/scan_planner/algorithm/plan_env/src/grid_map.cpp').read_text()
gazebo_xacro_source = Path(
    'src/go2_description/xacro/gazebo.xacro').read_text()
assert 'FSM_TRAJ_EXPIRED_NOT_REACHED' in scan_fsm_source
assert 'Keep the target' in scan_fsm_source
assert 'checkGlobalTargetOccupancy' in scan_fsm_source
assert 'Keeping it as the mission goal' in scan_fsm_source
assert 'use backward collision-free point' not in scan_fsm_source
assert '(!use_goal_z || std::abs(end_pt.z() - odom_pos.z()) <= finish_dist_z)' in scan_adapter_source
assert 'forcing spline-tangent steering' in scan_adapter_source
assert 'isWithinLidarHorizontalFov' in grid_map_source
assert 'clearOccupancyOutsideLidarFov' in grid_map_source
assert 'ground_filter_clear_rays' not in grid_map_source
assert 'string("/go2/robot_description")' not in grid_map_source
assert '"/go2/robot_description"' in rl_controller_source
assert '"/robot_description", rclcpp::QoS' not in rl_controller_source
assert '<remapping>robot_description:=/go2/robot_description</remapping>' in gazebo_xacro_source
assert scan_fsm['fsm.thresh_replan'] == 0.2
assert scan_fsm['fsm.finish_dist'] == 0.40
assert scan_fsm['fsm.finish_dist_z'] == 1.5
assert scan_fsm['fsm.replan_retry_delay'] == 0.5
assert scan_fsm['grid_map.double_cylinder_radius'] == 0.20
assert scan_fsm['grid_map.double_cylinder_offset'] == 0.07
assert scan_fsm['grid_map.lidar_horizontal_fov_deg'] == 240.0
assert scan_fsm['grid_map.lidar_clear_outside_fov'] is True
assert scan_fsm['grid_map.obstacles_inflation_z_down'] == 0.03
assert scan_fsm['manager.planning_horizon'] == 3.0
assert scan_fsm['optimization.dist0'] == 0.25
assert scan_adapter['max_vx'] == 0.50
assert scan_adapter['max_vy'] == 0.22
assert scan_adapter['max_vyaw'] == 1.00
assert scan_adapter['kp_yaw'] == 1.2
assert scan_adapter['rotate_exit_ratio'] == 0.75
assert scan_adapter['steer_with_position_error'] is False
assert scan_adapter['goal_slowdown_distance'] == 0.80
assert scan_adapter['min_walk_speed'] == 0.0
assert scan_adapter['use_goal_z'] is False
assert scan_adapter['finish_dist_z'] == 1.5
unitree_l1 = yaml.safe_load((bringup_share/'config/fastlio_unitree_l1.yaml').read_text())['/**']['ros__parameters']
assert unitree_l1['common']['lid_topic'] == '/unitree/lidar'
assert unitree_l1['common']['imu_topic'] == '/unitree/lidar_imu'
assert unitree_l1['common']['body_frame'] == 'utlidar_imu'
assert unitree_l1['preprocess']['lidar_type'] == 5
assert unitree_l1['preprocess']['scan_line'] == 18
assert unitree_l1['preprocess']['timestamp_unit'] == 0
assert unitree_l1['mapping']['extrinsic_T'] == [0.007698, 0.014655, -0.00667]
assert unitree_l1['mapping']['extrinsic_R'] == [1.0, 0.0, 0.0,
                                                 0.0, 1.0, 0.0,
                                                 0.0, 0.0, 1.0]
unitree_l1_body = yaml.safe_load(
    (bringup_share/'config/fastlio_unitree_l1_body_imu.yaml').read_text())['/**']['ros__parameters']
assert unitree_l1_body['common']['imu_topic'] == '/imu_sensor_broadcaster/imu'
assert unitree_l1_body['common']['body_frame'] == 'imu_link'
assert unitree_l1_body['mapping']['extrinsic_T'] == [0.31502, 0.0, -0.089145]
sim_lio = yaml.safe_load(
    (bringup_share/'config/fastlio_sim.yaml').read_text())['/**']['ros__parameters']
assert sim_lio['common']['body_frame'] == 'livox_imu_link'
assert sim_lio['preprocess']['lidar_type'] == 6
assert sim_lio['mapping']['extrinsic_est_en'] is False
# robot.xacro places the IMU at this translation in the LiDAR frame, so the
# FAST-LIO LiDAR-in-IMU translation must be its inverse.
assert sim_lio['mapping']['extrinsic_T'] == [-0.011, -0.02329, 0.04412]
config = yaml.safe_load((share/'config/gazebo.yaml').read_text())['rl_quadruped_controller']['ros__parameters']
real = yaml.safe_load((share/'config/robot_control.yaml').read_text())['rl_quadruped_controller']['ros__parameters']
assert config == real
stairs = yaml.safe_load((share/'config/stairs_trot_controller.yaml').read_text())['rl_quadruped_controller']['ros__parameters']
assert stairs['model_folder'] == 'stairs_trot'
assert stairs['joints'] == config['joints']
assert stairs['stand_pos'][7] == 1.0 and stairs['stand_pos'][10] == 1.0
robot_lab = yaml.safe_load((share/'config/robot_lab_controller.yaml').read_text())['rl_quadruped_controller']['ros__parameters']
assert robot_lab['model_folder'] == 'robot_lab'
assert robot_lab['joints'] == [
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
]
assert robot_lab['stand_pos'] == [0.0, 0.8, -1.5] * 4
robot_lab_policy_config = yaml.safe_load((share/'config/robot_lab/config.yaml').read_text())
assert robot_lab_policy_config['commands_scale'] == [2.0, 2.0, -0.25]
legged_gym = yaml.safe_load((share/'config/legged_gym_controller.yaml').read_text())['rl_quadruped_controller']['ros__parameters']
assert legged_gym['model_folder'] == 'legged_gym'
assert legged_gym['joints'] == config['joints']
flat = yaml.safe_load((share/'config/mjlab_flat_controller.yaml').read_text())['rl_quadruped_controller']['ros__parameters']
assert flat['model_folder'] == 'mjlab_flat'
assert flat['joints'] == [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]
flat_policy_config = yaml.safe_load((share/'config/mjlab_flat/config.yaml').read_text())
assert flat_policy_config['num_observations'] == 45
assert flat_policy_config['history_length'] == 1
assert flat_policy_config['action_scale'] == 0.5
assert flat_policy_config['rl_kp'] == [20.0, 20.0, 40.0] * 4
assert flat_policy_config['rl_kd'] == [1.0, 1.0, 2.0] * 4
moe = yaml.safe_load((share/'config/moe_cts_77k_controller.yaml').read_text())['rl_quadruped_controller']['ros__parameters']
assert moe['model_folder'] == 'moe_cts_77k'
assert moe['joints'] == flat['joints']
moe_policy_config = yaml.safe_load((share/'config/moe_cts_77k/config.yaml').read_text())
assert moe_policy_config['num_observations'] == 45
assert moe_policy_config['history_length'] == 10
assert moe_policy_config['history_layout'] == 'term_major'
assert moe_policy_config['action_scale'] == 0.25
assert moe_policy_config['ang_vel_scale'] == 0.25
assert moe_policy_config['dof_vel_scale'] == 0.05
policy = share/'config/himloco/himloco.pt'
lock = json.loads(Path('docs/GO2_UPSTREAM_LOCK.json').read_text())
assert hashlib.sha256(policy.read_bytes()).hexdigest() == lock['policy_files']['descriptions/unitree/go2_description/config/himloco/himloco.pt']
stairs_lock = json.loads(Path('docs/GO2_STAIRS_POLICY_LOCK.json').read_text())
for filename, digest in stairs_lock['files'].items():
    assert hashlib.sha256((share/'config/stairs_trot'/filename).read_bytes()).hexdigest() == digest
policy_lock = json.loads(Path('docs/GO2_POLICY_MODELS_LOCK.json').read_text())
for family, folder in (('flat', 'mjlab_flat'), ('rough', 'moe_cts_77k')):
    for filename, digest in policy_lock[family]['files'].items():
        model_file = share/'config'/folder/filename
        assert model_file.is_file(), model_file
        assert hashlib.sha256(model_file.read_bytes()).hexdigest() == digest
cts_lock = json.loads(Path('docs/GO2_CTS_POLICY_LOCK.json').read_text())
cts_policy = share/'config/go2_cts/go2_cts_150k.pt'
assert cts_policy.is_file()
assert hashlib.sha256(cts_policy.read_bytes()).hexdigest() == cts_lock['sha256']
cts_config = yaml.safe_load((share/'config/go2_cts/config.yaml').read_text())
assert cts_config['num_observations'] == 45
assert cts_config['commands_scale'] == cts_lock['contract']['command_scale']
legged_policy = share/'config/legged_gym/policy.pt'
assert hashlib.sha256(legged_policy.read_bytes()).hexdigest() == lock['policy_files']['descriptions/unitree/go2_description/config/legged_gym/policy.pt']
for mode in ('true', 'false'):
    xml = xacro.process_file(str(share/'xacro/robot.xacro'), mappings={'GAZEBO':mode, 'CLASSIC':'false'}).toxml()
    root = ET.fromstring(xml)
    links = {link.attrib['name'] for link in root.findall('link')}
    assert {'lidar', 'utlidar_lidar', 'utlidar_imu', 'imu_link'} <= links
    imu_joint = root.find("joint[@name='imu_joint']/origin")
    assert imu_joint is not None and imu_joint.attrib['xyz'] == '-0.02557 0 0.04232'
    livox_imu_joint = root.find("joint[@name='livox_imu_joint']/origin")
    assert livox_imu_joint is not None
    assert livox_imu_joint.attrib['xyz'] == '0.011 0.02329 -0.04412'
    joints = {j.attrib['name']:j for j in root.findall('joint') if j.attrib['type']=='revolute'}
    assert set(joints) == set(config['joints']) and len(joints)==12
    for pose in ('down_pos','stand_pos'):
        for name, value in zip(config['joints'],config[pose],strict=True):
            limit = joints[name].find('limit')
            assert float(limit.attrib['lower']) <= value <= float(limit.attrib['upper']), (pose,name,value)
    for mesh in root.findall('.//mesh'):
        assert Path(mesh.attrib['filename'].removeprefix('file://')).is_file()
    if mode == 'true':
        controlled = root.findall('ros2_control/joint')
        assert len(controlled)==12
        for joint in controlled:
            initial = joint.find("state_interface[@name='position']/param[@name='initial_value']")
            assert initial is not None
            assert float(initial.text) == config['down_pos'][config['joints'].index(joint.attrib['name'])]
        artifact_dir = Path(tempfile.gettempdir()) / 'legbot_offline_check'
        artifact_dir.mkdir(parents=True, exist_ok=True)
        urdf = artifact_dir / 'go2.urdf'
        urdf.write_text(xml)
        result = subprocess.run(['gz','sdf','-p',str(urdf)],check=True,capture_output=True,text=True)
        (artifact_dir / 'go2.sdf').write_text(result.stdout)
        (artifact_dir / 'sdf_parser.log').write_text(result.stderr)
        sdf = ET.fromstring(result.stdout)
        sensors = {s.attrib['name'] for s in sdf.findall('.//sensor')}
        assert {'navigation_lidar','livox_imu','imu_sensor'} <= sensors, sensors
        sdf_joints = {j.attrib['name'] for j in sdf.findall('.//joint')}
        assert set(joints) <= sdf_joints
world = Path(get_package_share_directory('legbot_bringup'))/'worlds/Building.sdf'
model_path = str(world.parent.parent/'models')
gz_environment = dict(os.environ)
gz_environment['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
    item for item in (model_path, gz_environment.get('GZ_SIM_RESOURCE_PATH', '')) if item)
gz_environment['SDF_PATH'] = os.pathsep.join(
    item for item in (model_path, gz_environment.get('SDF_PATH', '')) if item)
result = subprocess.run(['gz','sdf','-k',str(world)],check=True,capture_output=True,text=True,
                        env=gz_environment)
assert 'Valid' in result.stdout, result.stdout+result.stderr

flat_world = world.parent/'ScanFlatFastLio.sdf'
flat_root = ET.parse(flat_world).getroot()
models = {model.attrib['name']: model for model in flat_root.findall('.//model')}
assert models['flat_ground'].findtext('.//plane/size') == '40 24'
landmarks = {name: model for name, model in models.items()
             if name.startswith('localization_')}
assert len(landmarks) == 8
assert {'localization_north_slanted_panel', 'localization_south_ramp',
        'localization_east_stack'} <= set(landmarks)
expected_wall_poses = {
    'north_wall': '2 6.0 1.0 0 0 0',
    'south_wall': '2 -6.0 1.0 0 0 0',
    'west_wall': '-9.0 0 1.0 0 0 0',
    'east_wall': '13.0 0 1.0 0 0 0',
}
for name, pose in expected_wall_poses.items():
    assert models[name].findtext('pose') == pose
assert models['avoidance_block_1'].findtext('pose').startswith('-1.00 ')
result = subprocess.run(['gz', 'sdf', '-k', str(flat_world)], check=True,
                        capture_output=True, text=True, env=gz_environment)
assert 'Valid' in result.stdout, result.stdout + result.stderr
print('PASS GO2 URDF, policy provenance, Harmonic world and preserved lidar/IMU sensors')
