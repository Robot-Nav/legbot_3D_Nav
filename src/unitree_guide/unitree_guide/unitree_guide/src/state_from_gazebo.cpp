#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
// #include "tf2_ros/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>   // 将tf监听绑定到ROS回调函数


using namespace std;
ros::Publisher robotVelocity_BASE_frame_pub;
string robot_name = "a1";
nav_msgs::Odometry Odom;
double x=0, y=0, z=0, roll=0, pitch=0, yaw=0;
bool publish_zero_velocity = false;


void callback_BASE(const gazebo_msgs::LinkStates::ConstPtr &msg) {
    size_t index = 0;
    for (const auto &linkName : msg->name) {
        if (linkName == robot_name+"_gazebo::base")
            break;
        ++index;
    }
    if (index >= msg->name.size() || index >= msg->pose.size() || index >= msg->twist.size()) {
        ROS_ERROR_THROTTLE(1.0, "[state_from_gazebo] link %s_gazebo::base not found", robot_name.c_str());
        return;
    }

    // /gazebo/link_states can be emitted more than once during one simulation
    // clock tick.  Publishing the same TF stamp repeatedly makes tf2 report
    // TF_REPEATED_DATA and can flood the console when simulation time repeats.
    const ros::Time stamp = ros::Time::now();
    static ros::Time last_stamp;
    static bool have_last_stamp = false;
    if (have_last_stamp && stamp == last_stamp) {
        return;
    }
    last_stamp = stamp;
    have_last_stamp = true;

    //map到odom的tf变换
    static tf::TransformBroadcaster bf1;
    tf::Transform transform_odom2map;
    tf::Quaternion qtn;
    qtn.setRPY(roll, pitch, yaw);
    transform_odom2map.setRotation(tf::Quaternion(qtn.x(),
                                         qtn.y(),
                                         qtn.z(),
                                         qtn.w()));
    transform_odom2map.setOrigin(tf::Vector3(x,
                                    y,
                                    z));
    // 发布odom到map的tf关系
    bf1.sendTransform(tf::StampedTransform(transform_odom2map, stamp, "map", "odom"));
    
    //求变化矩阵的逆解，用于推算map到odom的关系，以便能得到base到map的关系，及
    tf::Transform transform_map2odom = transform_odom2map.inverse();

    tf::Point pt_map(msg->pose[index].position.x,msg->pose[index].position.y,msg->pose[index].position.z);
    tf::Point pt_odom = transform_map2odom * pt_map;
    
    tf::Quaternion q_map(msg->pose[index].orientation.x,
                        msg->pose[index].orientation.y,
                        msg->pose[index].orientation.z,
                        msg->pose[index].orientation.w);
    tf::Quaternion q_odom = transform_map2odom.getRotation() * q_map;

    // 转换为odom的速度关系
    tf::Vector3 linear_vel(
        msg->twist[index].linear.x,
        msg->twist[index].linear.y,
        msg->twist[index].linear.z);
    tf::Vector3 transformed_linear_vel = transform_map2odom.getBasis() * linear_vel;

    tf::Vector3 angular_vel(
        msg->twist[index].angular.x,
        msg->twist[index].angular.y,
        msg->twist[index].angular.z);
    tf::Vector3 transformed_angular_vel = transform_map2odom.getBasis() * angular_vel;
    
    //发布base到odom的tf变换
    static tf::TransformBroadcaster bf2;
    tf::Transform transform_odom2base;
    transform_odom2base.setRotation(q_odom);
    transform_odom2base.setOrigin(pt_odom);

    bf2.sendTransform(tf::StampedTransform(transform_odom2base, stamp, "odom", "base"));

    Odom.header.stamp = stamp;
    Odom.header.frame_id = "odom";
    Odom.child_frame_id = "base";

    // set the position
    Odom.pose.pose.position.x = pt_odom.x();
    Odom.pose.pose.position.y = pt_odom.y();
    Odom.pose.pose.position.z = pt_odom.z();

    Odom.pose.pose.orientation.w = q_odom.w();
    Odom.pose.pose.orientation.x = q_odom.x();
    Odom.pose.pose.orientation.y = q_odom.y();
    Odom.pose.pose.orientation.z = q_odom.z();


    // set the velocity
    Odom.twist.twist.linear.x = transformed_linear_vel.x();
    Odom.twist.twist.linear.y = transformed_linear_vel.y();
    Odom.twist.twist.linear.z = transformed_linear_vel.z();


    Odom.twist.twist.angular.x = transformed_angular_vel.x();
    Odom.twist.twist.angular.y = transformed_angular_vel.y();
    Odom.twist.twist.angular.z = transformed_angular_vel.z();

    if (publish_zero_velocity)
    {
        Odom.twist.twist.linear.x = 0.0;
        Odom.twist.twist.linear.y = 0.0;
        Odom.twist.twist.linear.z = 0.0;
        Odom.twist.twist.angular.x = 0.0;
        Odom.twist.twist.angular.y = 0.0;
        Odom.twist.twist.angular.z = 0.0;
    }


    robotVelocity_BASE_frame_pub.publish(Odom);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "state_from_gazebo");
    ros::NodeHandle nh("~");
    ros::NodeHandle node;
    ros::Subscriber tfState_BASE_sub;

    // tf::TransformListener tf_listener_;

    if (argc != 7)   // x y z qx qy qz qw 
    {
        ROS_ERROR("Usage: static_transform_publisher x y z yaw pitch roll");
        return -1;
    }

    x = atof(argv[1]);
    y = atof(argv[2]);
    z = atof(argv[3]);

    // Store the rotation in the values used by callback_BASE.  The previous
    // local declarations shadowed these globals, so non-zero launch angles
    // were silently ignored.
    yaw   = atof(argv[4]);
    pitch = atof(argv[5]);
    roll  = atof(argv[6]);
  
    nh.param<std::string>("robot_name", robot_name, string("a1"));
    nh.param<bool>("publish_zero_velocity", publish_zero_velocity, false);
    tfState_BASE_sub = node.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, callback_BASE);
    robotVelocity_BASE_frame_pub = node.advertise<nav_msgs::Odometry>("/Odometry_gazebo", 1);

    ros::spin();
    return 0;
}
