import sys
import argparse
import numpy as np

import rospy
from nav_msgs.msg import Path

from utils import *
from planner_wrapper import TomogramPlanner


from geometry_msgs.msg import Point
from visualization_msgs.msg import *
from geometry_msgs.msg import PointStamped
from interactive_markers.menu_handler import *
from interactive_markers.interactive_marker_server import *

sys.path.append('../')
from config import Config

parser = argparse.ArgumentParser()
parser.add_argument('--scene', type=str, default='Building', help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\']')
parser.add_argument('--tomogram', type=str, default='', help='Tomogram basename without .pickle')
parser.add_argument('--start', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
parser.add_argument('--goal', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

cfg = Config()


scene_defaults = {
    'Building': ('building2_9', [-5.5, 6.0, 0.5], [2.0, -3.0, 4.5]),
}
if args.scene not in scene_defaults and not (args.tomogram and args.start and args.goal):
    parser.error('unknown scene or missing --tomogram/--start/--goal for a custom map')

default_tomogram, default_start, default_goal = scene_defaults.get(
    args.scene, (args.tomogram, args.start, args.goal)
)
tomo_file = args.tomogram or default_tomogram
start_pos = np.asarray(args.start or default_start, dtype=np.float32)
end_pos = np.asarray(args.goal or default_goal, dtype=np.float32)
if not np.isfinite(start_pos).all() or not np.isfinite(end_pos).all():
    parser.error('--start and --goal must contain finite coordinates')

path_pub = rospy.Publisher("/pct_path", Path, latch=True, queue_size=1)
planner = TomogramPlanner(cfg)

# 新增全局变量
last_planned_start_pos = None
last_planned_end_pos = None
plan_timer = None
PLAN_INTERVAL = 0.5  # 规划检查间隔（秒）


def plan_callback(event):  # 关键修改：添加event参数接收TimerEvent
    """定时器回调函数：执行路径规划并发布"""
    global last_planned_end_pos, last_planned_start_pos,end_pos, start_pos, plan_timer

    # 检查位置是否发生变化（考虑浮点数精度）
    position_changed = True
    if last_planned_start_pos is not None and last_planned_end_pos is not None:
        if np.linalg.norm(end_pos - last_planned_end_pos) < 0.01 and \
              np.linalg.norm(start_pos - last_planned_start_pos) < 0.01 :
            position_changed = False
    
    # 只有位置变化时才执行规划
    if position_changed:
        try:
            traj_3d = planner.plan(start_pos, end_pos)
            if traj_3d is not None:
                path_pub.publish(traj2ros(traj_3d))
                print(f"规划并发布路径: statr_pos:{start_pos},end_pos:{end_pos}")
                last_planned_start_pos = start_pos.copy()  # 更新上次规划位置
                last_planned_end_pos = end_pos.copy()  # 更新上次规划位置
        except rospy.ROSInterruptException:
            print("路径发布失败")
    
    # 重新启动定时器（实现周期性检查）
    plan_timer = rospy.Timer(rospy.Duration(PLAN_INTERVAL), plan_callback, oneshot=True)


def processFeedback(feedback):
    """只负责更新end_pos，不直接执行规划"""
    global end_pos,start_pos
    p = feedback.pose.position
    # 更新目标位置
    if feedback.marker_name=="start_pos":
        start_pos = np.array([p.x, p.y, p.z-0.5], dtype=np.float32)
    elif feedback.marker_name=="end_pos":
        end_pos = np.array([p.x, p.y, p.z-0.5], dtype=np.float32)

def normalizeQuaternion( quaternion_msg ):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s


def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x =  0.4
    marker.scale.y =  0.4
    marker.scale.z =  0.4
    marker.color.r = 0
    marker.color.g = 0
    marker.color.b = 1
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def make6DofMarker(position,name,fixed=True,show_6dof = True):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.name = name
    int_marker.description = f"{name} 6-DOF Controller"
    int_marker.pose.position= Point(*position)
    makeBoxControl(int_marker)

    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        normalizeQuaternion(control.orientation)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        normalizeQuaternion(control.orientation)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
    server.insert(int_marker, processFeedback)

    server.applyChanges()



def pct_plan():
    planner.loadTomogram(tomo_file)

    make6DofMarker(start_pos,"start_pos", show_6dof=True)
    make6DofMarker(end_pos, "end_pos",show_6dof=True)
    
    # print("初始目标位置", end_pos)
    
    # 启动定时器（首次延迟PLAN_INTERVAL后执行）
    global plan_timer
    plan_timer = rospy.Timer(rospy.Duration(PLAN_INTERVAL), plan_callback, oneshot=True)
    return




if __name__ == '__main__':
    global server
    rospy.init_node("pct_planner", anonymous=True)
    server = InteractiveMarkerServer("basic_controls")
    pct_plan()
    rospy.spin()
