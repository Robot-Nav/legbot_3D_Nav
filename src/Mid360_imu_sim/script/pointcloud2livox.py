#! /usr/bin/python3

# 将PointCloud -> PointCloud2 -> livox_ros_drive2.CustomMsg

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from std_msgs.msg import Header
import struct
from threading import Lock

# 全局变量
pub = None
lidar_frame = "lidar_frame"  # 替换为实际的 LiDAR 帧 ID
m_buf = Lock()


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


def mmw_handler(mmw_cloud_msg):
    global pub_laser_cloud, lidar_frame, m_buf

    # 加锁
    m_buf.acquire()

    # 将 PointCloud 转换为 PointCloud2
    laser_cloud_msg = PointCloud2()
    laser_cloud_msg.header.stamp = mmw_cloud_msg.header.stamp
    laser_cloud_msg.header.frame_id = lidar_frame
    laser_cloud_msg = pc2.create_cloud_xyz32(laser_cloud_msg.header, [(p.x, p.y, p.z) for p in mmw_cloud_msg.points])
    # laser_cloud_msg 是 PointCloud2格式数据
    custom_msg = pointcloud2_to_custommsg(laser_cloud_msg)
    pub.publish(custom_msg)
    # 发布 PointCloud2 消息
    # pub_laser_cloud.publish(laser_cloud_msg)

    # 解锁
    m_buf.release()

def main():
    global pub
    # 初始化 ROS 节点
    rospy.init_node('pre_mmw', anonymous=True)

    # 添加一个rosparam确认是否为仿真环境
    sim_param = rospy.get_param('~enable_pcd2livox', False)  # 私有参数
    if not sim_param:
        rospy.loginfo("Pointcloud2livox is enabled!")

        # 订阅 PointCloud 话题
        sub_mmw_cloud = rospy.Subscriber('/scan', PointCloud, mmw_handler)
        
        # pub_laser_cloud = rospy.Publisher("livox/lidar", PointCloud2, queue_size=2000)
        pub = rospy.Publisher('/livox/lidar2', CustomMsg, queue_size=10)

        # 保持节点运行
        rospy.spin()
    rospy.loginfo("Pointcloud2livox is disabled!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
