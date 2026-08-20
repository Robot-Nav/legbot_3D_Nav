#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@brief: 将 /scan (sensor_msgs/PointCloud) 变换到 odom 坐标系后发布为 PointCloud2
@Editor: CJH + 修改完善版
@Date: 2025-10-22 → 2025-11-22
"""

import tf
import rospy
import struct
import numpy as np
from threading import Lock

from sensor_msgs.msg import PointCloud, PointCloud2,PointField
from unitree_guide.msg import CustomMsg, CustomPoint
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry


ODOM_FRAME = "odom"
RAW_LIDAR_FRAME = "laser_livox"
ODOM_TOPIC = "/Odometry_gazebo"           
m_buf = Lock()
latest_odom = None
latest_odom_time = None


def _get_struct_fmt(pointcloud2):
    fmt = ''
    for field in pointcloud2.fields:
        if field.datatype == PointField.FLOAT32:
            fmt += 'f'
        elif field.datatype == PointField.UINT8:
            fmt += 'B'
        elif field.datatype == PointField.INT8:
            fmt += 'b'
        elif field.datatype == PointField.UINT16:
            fmt += 'H'
        elif field.datatype == PointField.INT16:
            fmt += 'h'
        elif field.datatype == PointField.UINT32:
            fmt += 'I'
        elif field.datatype == PointField.INT32:
            fmt += 'i'
        else:
            rospy.logwarn("Unsupported field type: %d", field.datatype)
    return fmt


def pointcloud2_to_custommsg(pointcloud2):
    custom_msg = CustomMsg()
    custom_msg.header = pointcloud2.header
    custom_msg.timebase = rospy.Time.now().to_nsec()
    custom_msg.point_num = pointcloud2.width
    custom_msg.lidar_id = 1  # Assuming lidar_id is 1
    custom_msg.rsvd = [0, 0, 0]  # Reserved fields

    # Parse PointCloud2 data
    fmt = _get_struct_fmt(pointcloud2)
    for i in range(0, len(pointcloud2.data), pointcloud2.point_step):
        point_data = pointcloud2.data[i:i+pointcloud2.point_step]
        x, y, z = struct.unpack(fmt, point_data)

        custom_point = CustomPoint()
        custom_point.offset_time = rospy.Time.now().to_nsec() - custom_msg.timebase
        custom_point.x = x
        custom_point.y = y
        custom_point.z = z
        # custom_point.reflectivity = int(intensity * 255)  # Scale intensity to 0-255
        custom_point.tag = 0  # Assuming no tag
        custom_point.line = 0  # Assuming no line number

        custom_msg.points.append(custom_point)

    return custom_msg

def rotate_pointcloud_y(points, theta):
    # theta = np.deg2rad(theta_deg)
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    R_y = np.array([
        [ cos_t, 0.0,  sin_t],
        [ 0.0,   1.0,  0.0 ],
        [-sin_t, 0.0,  cos_t]
    ])
    points_array = np.array(points, dtype=np.float32)
    rotated = (R_y @ points_array.T).T
    return rotated.tolist()

def odom_callback(odom_msg):
    global latest_odom, latest_odom_time
    with m_buf:
        latest_odom = odom_msg
        latest_odom_time = odom_msg.header.stamp

def quat_to_rot_matrix(q):
    """四元数 → 3x3 旋转矩阵 (numpy)"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])

def transform_points_to_odom(points_sensor, odom_msg):
    global tf_listener
    """
    将 sensor_frame 中的点云变换到 odom 坐标系
    """
    if odom_msg is None:
        return points_sensor

    try:
         # 获取base到laser_livox的变换
        (trans_base, rot_base) = tf_listener.lookupTransform('base', 'laser_livox', rospy.Time(0))
        rot_base_matrix = tf.transformations.quaternion_matrix(rot_base)[:3, :3]

        points_np = np.array(points_sensor, dtype=np.float32)
        if points_np.size == 0:
            return []
        points_base = (rot_base_matrix @ points_np.T).T + trans_base
        
        # 提取 odom → sensor_frame 的变换
        trans = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ])

        rot = quat_to_rot_matrix(odom_msg.pose.pose.orientation)

        # 先旋转，再平移： P_odom = R * P_sensor + t
        transformed = (rot @ points_base.T).T + trans
        return transformed.tolist()

    except Exception as e:
        rospy.logwarn("Exception in transform_points_to_odom: %s", str(e))
        # 如果TF变换失败，使用原来的方法
        trans = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ])
        rot = quat_to_rot_matrix(odom_msg.pose.pose.orientation)
        points_np = np.array(points_sensor, dtype=np.float32)
        if points_np.size == 0:
            return []
        transformed = (rot @ points_np.T).T + trans
        return transformed.tolist()


def filter_points_by_angle(points, min_angle_deg, max_angle_deg):
    """根据垂直角度过滤点云"""
    points_np = np.array(points, dtype=np.float32)
    if points_np.size == 0:
        return []
    
    # 计算每个点的垂直角度
    distances = np.linalg.norm(points_np[:, :2], axis=1)  # xy平面距离
    angles = np.arctan2(points_np[:, 2], distances)  # 垂直角度
    angles_deg = np.rad2deg(angles)
    
    # 角度过滤
    mask = (angles_deg >= min_angle_deg) & (angles_deg <= max_angle_deg)
    return points_np[mask].tolist()


def mmw_handler(mmw_cloud_msg):
    global latest_odom, pub_laser_cloud, pub_laser_livox, pub_laser_raw, laser_blind, min_angle, max_angle

    with m_buf:
        odom_now = latest_odom
        stamp = mmw_cloud_msg.header.stamp

    # Step 1: 提取原始点云 (更快的方式)
    raw_header = rospy.Header()
    raw_header.stamp = stamp
    raw_header.frame_id = RAW_LIDAR_FRAME
    x = np.fromiter((p.x for p in mmw_cloud_msg.points), dtype=np.float32)
    y = np.fromiter((p.y for p in mmw_cloud_msg.points), dtype=np.float32)
    z = np.fromiter((p.z for p in mmw_cloud_msg.points), dtype=np.float32)
    raw_points = np.column_stack((x, y, z)).tolist()

    if not raw_points:
        return

    # Step 2: 可选的固定 Y 轴旋转（比如安装角度补偿）
    rotated_points = rotate_pointcloud_y(raw_points, theta=0)  # 

    # Step 2.5: 角度过滤
    angle_filtered_points = filter_points_by_angle(rotated_points, min_angle, max_angle)

    # Step 3: 盲区过滤
    points_np = np.array(angle_filtered_points, dtype=np.float32)
    distances = np.linalg.norm(points_np, axis=1)
    filtered_points = points_np[distances >= laser_blind].tolist()

    # Step 3.5 转为 CustomMsg 并发布
    m_buf.acquire()
    cloud_msg_ = pc2.create_cloud_xyz32(raw_header, filtered_points)
    custom_msg = pointcloud2_to_custommsg(cloud_msg_)
    pub_laser_livox.publish(custom_msg)
    pub_laser_raw.publish(cloud_msg_)
    m_buf.release()

    # Step 4: 变换到 odom 坐标系
    transformed_points = transform_points_to_odom(filtered_points, odom_now)

    # Step 5: 创建 PointCloud2，frame_id = "odom"
    odom_header = rospy.Header()
    odom_header.stamp = stamp
    odom_header.frame_id = ODOM_FRAME
    cloud_msg = pc2.create_cloud_xyz32(odom_header, transformed_points)
    # 发布pocintcloud2消息
    pub_laser_cloud.publish(cloud_msg)

  
    


def main():
    global pub_laser_cloud, pub_laser_livox, pub_laser_raw, laser_blind, min_angle, max_angle, tf_listener


    rospy.init_node('pre_mmw_to_odom', anonymous=True)
    #监听雷达与底盘的安装角度，便于矫正雷达位置

    tf_listener = tf.TransformListener()

    laser_blind = rospy.get_param('~laser_blind', 0.2)  # 盲区半径
    rospy.loginfo(f"Blind range : {laser_blind} m")


    min_angle = rospy.get_param('~min_angle', 2.5)  # 默认下限-15度
    max_angle = rospy.get_param('~max_angle',60)   # 默认上限45度
    rospy.loginfo(f"Angle filter : {min_angle} ~ {max_angle} deg")

    # 订阅原始点云和里程计
    rospy.Subscriber('/scan', PointCloud, mmw_handler, queue_size=10)
    rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback, queue_size=10)

    pub_laser_livox = rospy.Publisher('/livox/lidar2', CustomMsg, queue_size=10)
    pub_laser_raw = rospy.Publisher('/livox/points_raw', PointCloud2, queue_size=10)
    pub_laser_cloud = rospy.Publisher("/livox/Pointcloud2", PointCloud2, queue_size=10)

    rospy.loginfo("=== Pointcloud2livox STARTED ===")
    rospy.loginfo(f"Raw lidar frame: {RAW_LIDAR_FRAME}")
    rospy.loginfo(f"Raw FAST-LIO topic: /livox/points_raw")
    rospy.loginfo(f"Odom topic : {ODOM_TOPIC}")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
