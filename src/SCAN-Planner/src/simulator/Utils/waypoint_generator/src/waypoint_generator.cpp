#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <string>

ros::Publisher waypoints_pub;
ros::Publisher waypoints_vis_pub;
nav_msgs::Odometry odom;
bool is_odom_ready = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  is_odom_ready = true;
  odom = *msg;
}

void publishWaypoints(const geometry_msgs::PoseStamped& goal) {
  nav_msgs::Path waypoints;
  waypoints.header.frame_id = "world";
  waypoints.header.stamp = ros::Time::now();
  waypoints.poses.push_back(goal);
  waypoints_pub.publish(waypoints);

  geometry_msgs::PoseArray waypoints_vis;
  waypoints_vis.header = waypoints.header;
  if (is_odom_ready) {
    waypoints_vis.poses.push_back(odom.pose.pose);
  }
  waypoints_vis.poses.push_back(goal.pose);
  waypoints_vis_pub.publish(waypoints_vis);
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (msg->pose.position.z <= -0.1) {
    ROS_WARN("[waypoint_generator] invalid goal.");
    return;
  }

  publishWaypoints(*msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_generator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string body_pose_topic;
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));

  ros::Subscriber odom_sub = nh.subscribe(body_pose_topic, 10, odomCallback);
  ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, goalCallback);
  waypoints_pub = pnh.advertise<nav_msgs::Path>("waypoints", 50);
  waypoints_vis_pub = pnh.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);

  ros::spin();
  return 0;
}
