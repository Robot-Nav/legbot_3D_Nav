"""Ethernet GO2 EDU deployment; command output is disabled by default."""
import os
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    val = lambda n: LaunchConfiguration(n).perform(context)
    interface = val('network_interface')
    if not interface or interface == 'lo' or not os.path.isdir('/sys/class/net/'+interface):
        raise RuntimeError('Provide an existing Ethernet interface, for example network_interface:=enp131s0')
    mappings = {'GAZEBO': 'false', 'network_interface': interface, 'domain': '0', 'enable_commands': val('enable_commands')}
    mappings.update({key: val(key) for key in ('lidar_x','lidar_y','lidar_z','lidar_roll','lidar_pitch','lidar_yaw')})
    description = xacro.process_file(os.path.join(share('go2_description'), 'xacro/robot.xacro'), mappings=mappings).toxml()
    controller_files = {
        'himloco': 'robot_control.yaml',
        'legged_gym': 'legged_gym_controller.yaml',
        'stairs_trot': 'stairs_trot_controller.yaml',
        'robot_lab': 'robot_lab_controller.yaml',
        'go2_cts': 'go2_cts_controller.yaml',
        'mjlab_flat': 'mjlab_flat_controller.yaml',
        'moe_cts_77k': 'moe_cts_77k_controller.yaml',
    }
    controller_file = controller_files[val('policy_profile')]
    package_share = share('legbot_bringup')
    workspace_root = os.path.abspath(os.path.join(package_share, '..', '..', '..', '..'))
    diagnostic_overlay = os.path.join(
        workspace_root, '.deps', 'diagnostic_updater_4.2.7', 'opt', 'ros', 'jazzy', 'lib')
    library_path = os.environ.get('LD_LIBRARY_PATH', '')
    bundled_library_dirs = [
        os.path.join(workspace_root, 'third_party', 'libtorch', 'lib'),
        os.path.join(workspace_root, 'third_party', 'onnxruntime', 'lib'),
        diagnostic_overlay,
    ]
    for directory in reversed(bundled_library_dirs):
        if os.path.isdir(directory):
            library_path = directory + (os.pathsep + library_path if library_path else '')
    manager = Node(package='controller_manager', executable='ros2_control_node', output='screen',
                   parameters=[os.path.join(share('go2_description'),'config',controller_file), {'use_sim_time': False}],
                   remappings=[('~/robot_description','/go2/robot_description')])
    controllers = ['joint_state_broadcaster','imu_sensor_broadcaster']
    if val('enable_commands') == 'true':
        controllers.append('rl_quadruped_controller')
    actions = [SetEnvironmentVariable('LD_LIBRARY_PATH', library_path),
               Node(package='robot_state_publisher', executable='robot_state_publisher',
                    parameters=[{'robot_description': description, 'use_sim_time': False}],
                    remappings=[('robot_description', '/go2/robot_description')]),
               RegisterEventHandler(OnProcessStart(target_action=manager, on_start=[
                   Node(package='controller_manager', executable='spawner',
                        arguments=[c,'--controller-manager-timeout','120']) for c in controllers])), manager]
    if val('enable_commands') == 'true':
        actions.append(Node(package='legbot_bringup', executable='go2_command_adapter', parameters=[{
            'use_sim_time': False,
            'odom_topic': val('odom_topic'),
            'cloud_topic': val('cloud_topic'),
            'require_navigation_data': val('require_navigation_data') == 'true',
            'command_timeout': float(val('command_timeout')),
            'navigation_timeout': float(val('navigation_timeout')),
            'max_vx': float(val('max_vx')),
            'max_vy': float(val('max_vy')),
            'max_yaw_rate': float(val('max_yaw_rate')),
        }]))
    return actions


def generate_launch_description():
    args = [DeclareLaunchArgument('network_interface', description='Ethernet adapter connected to GO2 EDU'),
            DeclareLaunchArgument('enable_commands', default_value='false', choices=['false','true']),
            DeclareLaunchArgument('policy_profile', default_value='moe_cts_77k',
                                  choices=['himloco','legged_gym','stairs_trot','robot_lab','go2_cts',
                                           'mjlab_flat','moe_cts_77k']),
            DeclareLaunchArgument('odom_topic', default_value='/fast_lio/odometry_base'),
            DeclareLaunchArgument('cloud_topic', default_value='/fast_lio/cloud_registered'),
            DeclareLaunchArgument('require_navigation_data', default_value='true',
                                  choices=['true', 'false']),
            DeclareLaunchArgument('command_timeout', default_value='0.25'),
            DeclareLaunchArgument('navigation_timeout', default_value='0.50'),
            DeclareLaunchArgument('max_vx', default_value='0.20'),
            DeclareLaunchArgument('max_vy', default_value='0.08'),
            DeclareLaunchArgument('max_yaw_rate', default_value='0.30')]
    args += [DeclareLaunchArgument(key, default_value=value) for key,value in
             [('lidar_x','0.15'),('lidar_y','0.0'),('lidar_z','0.15'),('lidar_roll','0.0'),('lidar_pitch','0.0'),('lidar_yaw','0.0')]]
    return LaunchDescription(args+[OpaqueFunction(function=setup)])
