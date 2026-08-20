#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
// #include "tf2_ros/transform_listener.h"
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>   // 将tf监听绑定到ROS回调函数


using namespace std;
ros::Publisher robotVelocity_BASE_frame_pub;
string robot_name = "a1";
nav_msgs::Odometry Odom;
double x=0, y=0, z=0, roll=0, pitch=0, yaw=-1.5707963;


void callback_Odometry(const nav_msgs::Odometry::ConstPtr &msg) {

    static tf::TransformBroadcaster bf2;
    tf::Transform transform_odom2map;
    tf2::Quaternion qtn;
    qtn.setRPY(roll, pitch, yaw);

    transform_odom2map.setRotation(tf::Quaternion(qtn.x(),
                                         qtn.y(),
                                         qtn.z(),
                                         qtn.w()));
    transform_odom2map.setOrigin(tf::Vector3(x,
                                    y,
                                    z));
    
    tf::Point pt_map(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    tf::Point pt_odom = transform_odom2map * pt_map;
    
    tf::Quaternion q_map(msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
    tf::Quaternion q_odom = transform_odom2map.getRotation() * q_map;


    tf::Vector3 linear_vel(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z
    );
    tf::Vector3 transformed_linear_vel = transform_odom2map * linear_vel;

    tf::Vector3 angular_vel(
        msg->twist.twist.angular.x,
        msg->twist.twist.angular.y,
        msg->twist.twist.angular.z
    );
    tf::Vector3 transformed_angular_vel = transform_odom2map * angular_vel;


    static tf::TransformBroadcaster bf;
    tf::Transform transform_odom2base;
    transform_odom2base.setRotation(q_odom);
    transform_odom2base.setOrigin(pt_odom);

    bf.sendTransform(tf::StampedTransform(transform_odom2base, msg->header.stamp, msg->header.frame_id, "body_transform"));

    Odom.header.stamp = msg->header.stamp;
    Odom.header.frame_id =msg->header.frame_id;
    // cout<<"msg->header.frame_id:"<<msg->header.frame_id<<endl;
    Odom.child_frame_id = "body_transform";
    // set the position
    
    Odom.pose.pose.position.x = pt_map.getX();
    Odom.pose.pose.position.y = pt_map.getY();
    Odom.pose.pose.position.z = pt_map.getZ();
    
    Odom.pose.pose.orientation.w = q_odom.getW() ;
    Odom.pose.pose.orientation.x = q_odom.getX()  ;
    Odom.pose.pose.orientation.y = q_odom.getY() ;
    Odom.pose.pose.orientation.z = q_odom.getZ() ;
    // set the velocity 
    Odom.twist.twist.linear.x = transformed_linear_vel.getX();
    Odom.twist.twist.linear.y = transformed_linear_vel.getY();
    Odom.twist.twist.linear.z = transformed_linear_vel.getZ();

    Odom.twist.twist.angular.x = transformed_angular_vel.getX();
    Odom.twist.twist.angular.y = transformed_angular_vel.getY();
    Odom.twist.twist.angular.z = transformed_angular_vel.getZ();

    robotVelocity_BASE_frame_pub.publish(Odom);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_transform");
    ros::NodeHandle nh("~");
    ros::NodeHandle node;
    ros::Subscriber Odometry_origin_sub;
    // tf::TransformListener tf_listener_;

    // if (argc != 7)   // x y z qx qy qz qw 
    // {
    //     ROS_ERROR("Usage: static_transform_publisher x y z yaw pitch roll");
    //     return -1;
    // }

    // x = atof(argv[1]);
    // y = atof(argv[2]);
    // z = atof(argv[3]);

    // double yaw   = -1.57;
    // double pitch = 0;
    // double roll  = 0;
    
    Odometry_origin_sub = node.subscribe<nav_msgs::Odometry>("/Odometry", 10, callback_Odometry);
    robotVelocity_BASE_frame_pub = node.advertise<nav_msgs::Odometry>("/Odometry_transform", 1);

    ros::spin();
    return 0;
}



