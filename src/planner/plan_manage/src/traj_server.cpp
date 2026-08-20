/**
 * @file traj_server.cpp
 * @brief 轨迹服务器节点，用于接收和处理B样条轨迹，并生成速度命令
 */
#include "bspline_opt/uniform_bspline.h"            // 奥迪ometry消息类型  // 均匀B样条优化库
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int16.h>

#include <boost/bind.hpp>   // 将tf监听绑定到ROS回调函数
#include <tf/transform_listener.h>

ros::Publisher pos_vel_pub;
std::string odom_topic;

using ego_planner::UniformBspline;
using namespace std;

bool receive_traj_ = false;
bool have_odom_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
ros::Time last_odom_time_;

// yaw control
double last_yaw_=0, last_yaw_dot_=0;
double time_forward_;

geometry_msgs::Pose odom2map_Pose;
Eigen::Vector3d end_pt_,end_euler_; // goal state

ros::Publisher control_point_state_pub_;
nav_msgs::Odometry control_point_state;

std::pair<double, double> calculate_yaw(double dt);
double LimitSpeed(const double vel_input,const double upper,const double lower);
void odometryCallback(const nav_msgs::OdometryConstPtr &msg,tf::TransformListener* tf_listener_ptr);

// PID parameters
double kp_yaw_, ki_yaw_, kd_yaw_;
double kp_position_, max_command_speed_, odom_timeout_;
double yaw_error_integral_ = 0.0;
double last_yaw_error_ = 0.0;

//触发路径规划的回调函数
void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  if (!msg || msg->order < 1 || msg->pos_pts.size() <= static_cast<size_t>(msg->order) ||
      msg->knots.size() != msg->pos_pts.size() + msg->order + 1)
  {
    ROS_ERROR_THROTTLE(1.0, "[Traj server] Reject malformed B-spline message.");
    return;
  }

  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    if (!std::isfinite(msg->pos_pts[i].x) || !std::isfinite(msg->pos_pts[i].y) ||
        !std::isfinite(msg->pos_pts[i].z))
    {
      ROS_ERROR_THROTTLE(1.0, "[Traj server] Reject B-spline with non-finite control point.");
      return;
    }
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  start_time_ = msg->start_time;

  traj_.clear();
  traj_.push_back(pos_traj);


  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
  yaw_error_integral_ = 0.0;
  last_yaw_error_ = 0.0;
}


double LimitSpeed(const double vel_input,const double upper,const double lower)
{
  double vel_output;
  if(vel_input > upper) vel_output = upper;
  else if(vel_input < lower) vel_output = lower;
  else vel_output = vel_input;

  return vel_output;
}


void odometryCallback(const nav_msgs::OdometryConstPtr &msg,tf::TransformListener* tf_listener_ptr)
{
    try
    {
      // 则当前odom_pos_为base与map的关系
      tf::StampedTransform transform_odom2map;
      transform_odom2map.setIdentity();//将变换矩阵初始化为单位阵

      //将当前位姿从odom坐标系转换到map坐标，如果存在tf变换关系的话
      if (!tf_listener_ptr->waitForTransform("map", msg->header.frame_id, ros::Time(0), ros::Duration(0.2)))
      {
        ROS_WARN_THROTTLE(1.0, "[Traj server] Waiting for transform map <- %s.", msg->header.frame_id.c_str());
        return;
      }
      tf_listener_ptr->lookupTransform("map", msg->header.frame_id,
                                  ros::Time(0), transform_odom2map);
      // 位置变换    
      tf::Point pt_odom(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
      tf::Point pt_map = transform_odom2map * pt_odom;

      odom2map_Pose.position.x = pt_map.x();
      odom2map_Pose.position.y = pt_map.y();
      odom2map_Pose.position.z = pt_map.z();                 
      // 速度变换（线速度需要旋转到 map 坐标系）
      tf::Quaternion q_odom(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
      tf::Quaternion q_map = transform_odom2map.getRotation() * q_odom;
      odom2map_Pose.orientation.x = q_map.x();
      odom2map_Pose.orientation.y = q_map.y();
      odom2map_Pose.orientation.z = q_map.z();
      odom2map_Pose.orientation.w = q_map.w();
      have_odom_ = true;
      last_odom_time_ = ros::Time::now();
    }
    catch(const tf::TransformException& e)
    {
      ROS_WARN_THROTTLE(1.0, "[Traj server] Odometry transform failed: %s", e.what());
      return;
    }

}



std::pair<double, double> calculate_yaw(double dt)
{
  double yaw_diff=0;
  double min_close_distance=1.0;
  constexpr double PI = 3.1415926;
  //获取当前机器人偏航角，与当前位置
  double yaw_robot = tf::getYaw(odom2map_Pose.orientation);
  double yaw_control_point = tf::getYaw(control_point_state.pose.pose.orientation);

  Eigen::Vector3d current_pos=Eigen::Vector3d(odom2map_Pose.position.x,odom2map_Pose.position.y,odom2map_Pose.position.z);

  //当位置接近于目标，使用终点的偏航角
  yaw_diff=((end_pt_ - current_pos).norm()<min_close_distance? end_euler_(2):yaw_control_point) - yaw_robot;

  // cout<<"distance to goal:"<< (end_pt_ - current_pos).norm()<<endl;
  // cout<<"end_euler_(2)"<< end_euler_(2)*180/3.14 <<endl;
  while (yaw_diff > PI)  yaw_diff -= 2 * PI;
  while (yaw_diff < -PI) yaw_diff += 2 * PI;

  // PID控制计算
  dt = std::max(1e-3, std::min(dt, 0.2));
  yaw_error_integral_ += yaw_diff * dt;
  yaw_error_integral_ = LimitSpeed(yaw_error_integral_, 1.5, -1.5);
  double yaw_error_derivative = (yaw_diff - last_yaw_error_) / dt;
  double yaw_rate = kp_yaw_ * yaw_diff + 
                   ki_yaw_ * yaw_error_integral_ + 
                   kd_yaw_ * yaw_error_derivative;
  
  last_yaw_error_ = yaw_diff;
  yaw_rate = LimitSpeed(yaw_rate, 1.0, -1.0);//除以 时间差 得到 角速度

  //计算角速度速度
  // cout<<"angular PID veloctiy"<< "  " <<yaw_rate*180/3.14<<endl;
  // cout<<"angular diff"<< "  " <<yaw_diff*180/3.14<<endl;
  // cout<<"angular veloctiy"<< "  " <<yaw_diff*180/3.14/ (t_cur+1e-6)<<endl;
  // cout << "future_yaw  " << yaw_control_point*180/3.14 << " current_yaw " <<yaw_robot*180/3.14 << endl;

  return {yaw_diff,yaw_rate};// 角度差值  角速度
}

void waypointCallback(const nav_msgs::PathConstPtr &msg)
{
  if (!msg || msg->poses.empty())
    return;
  if (msg->poses[0].pose.position.z < -0.1)
    return;
  end_pt_ << msg->poses.back().pose.position.x, msg->poses.back().pose.position.y, msg->poses.back().pose.position.z; //twilight: goal height
  Eigen::Quaterniond end_qtn_ = Eigen::Quaterniond(
    msg->poses.back().pose.orientation.w,  // 实部 w
    msg->poses.back().pose.orientation.x,  // 虚部 x
    msg->poses.back().pose.orientation.y,  // 虚部 y
    msg->poses.back().pose.orientation.z   // 虚部 z
  );
  end_euler_ =end_qtn_.matrix().eulerAngles(0,1,2);
}


void cmdCallback(const ros::TimerEvent &e)
{
  geometry_msgs::Twist robotVelocity_BASE_frame;

  ros::Time time_now = ros::Time::now();
  if (!receive_traj_ || !have_odom_ || (time_now - last_odom_time_).toSec() > odom_timeout_)
  {
    ROS_WARN_THROTTLE(1.0, "[Traj server] No valid trajectory/odometry; publishing zero command.");
    pos_vel_pub.publish(robotVelocity_BASE_frame);
    return;
  }

  double t_cur = (time_now - start_time_).toSec();
  // cout<<"t_cur: "<<t_cur<<endl;
  //向前预测时间段
  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);
  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    /*** calculate yaw ***/
    
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  const double control_dt = (time_now - time_last).toSec();
  time_last = time_now;

  // ------------ estimation q from traj ------------- // 
  control_point_state.header.frame_id = "map";
  static double yaw_estimate_from_traj = 0;
  // vy/vx 得到预测偏航角
  if(vel.norm()>0.1)//设置一个关于矢量速度的低通滤波，在突然暂停的情况下保持yaw的旋转角
  yaw_estimate_from_traj=std::atan2(vel(1),vel(0)+1e-6);
  
  //构建一个欧拉角的（3，1）变量
  Eigen::Vector3d eulerAngle(yaw_estimate_from_traj,0,0);
  //利用tf的四元数类，转欧拉角为四元数
  tf::Quaternion qtn;
  qtn.setRPY(eulerAngle[2],eulerAngle[1],eulerAngle[0]);// 设置偏航值，俯仰值，翻滚值，单位是弧度
  // cout << "yaw_estimate_from_traj: " << yaw_estimate_from_traj*180/PI << endl;
  control_point_state.pose.pose.position.x = pos(0);
  control_point_state.pose.pose.position.y = pos(1);
  control_point_state.pose.pose.position.z = pos(2);
  control_point_state.pose.pose.orientation.x = qtn.getX();
  control_point_state.pose.pose.orientation.y = qtn.getY();
  control_point_state.pose.pose.orientation.z = qtn.getZ();
  control_point_state.pose.pose.orientation.w = qtn.getW();
  //可视化转向角度
  control_point_state_pub_.publish(control_point_state);
  yaw_yawdot = calculate_yaw(control_dt);
  // ------------ trans vel ------------- // 

  geometry_msgs::Twist velWorld;
  
  velWorld.angular.x = 0;
  velWorld.angular.y = 0;
  velWorld.angular.z = yaw_yawdot.second;

  if (abs(yaw_yawdot.first)<0.785)
  {
    const Eigen::Vector3d current_pos(odom2map_Pose.position.x,
                                      odom2map_Pose.position.y,
                                      odom2map_Pose.position.z);
    Eigen::Vector3d corrected_vel = vel + kp_position_ * (pos - current_pos);
    corrected_vel(2) = 0.0;
    const double speed_xy = corrected_vel.head<2>().norm();
    if (max_command_speed_ > 0.0 && speed_xy > max_command_speed_)
      corrected_vel.head<2>() *= max_command_speed_ / speed_xy;
    velWorld.linear.x = corrected_vel(0);
    velWorld.linear.y = corrected_vel(1);
    velWorld.linear.z = 0;
  }else{
    velWorld.linear.x = 0;
    velWorld.linear.y = 0;
    velWorld.linear.z = 0;
  }

  // transform velocity to base
  Eigen::Matrix<double, 3, 1> baseLinearVelWorldCur;
  Eigen::Matrix<double, 3, 1> baseLinearVelBodyCur;
  Eigen::Quaterniond baseOriWorldCur;
  Eigen::Matrix<double, 3, 1> baseAngularVelWorldCur;
  Eigen::Matrix<double, 3, 1> baseAngularVelBodyCur;
  
  // cout << "position: " << basePosWorldCur[0] << "  " << basePosWorldCur[1] << "  " << basePosWorldCur[2] <<endl;
  baseOriWorldCur.w() = odom2map_Pose.orientation.w;
  baseOriWorldCur.x() = odom2map_Pose.orientation.x;
  baseOriWorldCur.y() = odom2map_Pose.orientation.y;
  baseOriWorldCur.z() = odom2map_Pose.orientation.z;

  baseLinearVelWorldCur[0]  = velWorld.linear.x;
  baseLinearVelWorldCur[1]  = velWorld.linear.y;
  baseLinearVelWorldCur[2]  = velWorld.linear.z;
  baseAngularVelWorldCur[0] = velWorld.angular.x;
  baseAngularVelWorldCur[1] = velWorld.angular.y;
  baseAngularVelWorldCur[2] = velWorld.angular.z;

  baseLinearVelBodyCur = baseOriWorldCur.toRotationMatrix().transpose() * baseLinearVelWorldCur;
  baseAngularVelBodyCur = baseOriWorldCur.toRotationMatrix().transpose() * baseAngularVelWorldCur;

  // write to cmd vel
  robotVelocity_BASE_frame.linear.x = baseLinearVelBodyCur[0];
  robotVelocity_BASE_frame.linear.y = baseLinearVelBodyCur[1];
  robotVelocity_BASE_frame.linear.z = 0;

  robotVelocity_BASE_frame.angular.x = 0;
  robotVelocity_BASE_frame.angular.y = 0;
  robotVelocity_BASE_frame.angular.z = baseAngularVelBodyCur[2];

  // cout << "current_pos: " << odom2map_Pose.position.x<<"   "<<odom2map_Pose.position.y <<endl;
  // cout << "future_pos: " << control_point_state.pose.pose.position.x<<"   "<<control_point_state.pose.pose.position.y <<endl;
  // cout << "velocity: " << robotVelocity_BASE_frame.linear.x << "  " << robotVelocity_BASE_frame.angular.y <<endl;
  // cout << "yaw_velocity: " << robotVelocity_BASE_frame.angular.z<<endl;

  pos_vel_pub.publish(robotVelocity_BASE_frame);

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;// public
  ros::NodeHandle nh("~");// private

  tf::TransformListener tf_listener_;

  nh.param("traj_server/kp_yaw", kp_yaw_, 1.5);
  nh.param("traj_server/ki_yaw", ki_yaw_, 0.0);
  nh.param("traj_server/kd_yaw", kd_yaw_, 0.0);
  nh.param("traj_server/kp_position", kp_position_, 0.8);
  nh.param("traj_server/max_command_speed", max_command_speed_, 0.8);
  nh.param("traj_server/odom_timeout", odom_timeout_, 0.5);

  nh.param<std::string>("odometry_topic", odom_topic, "default_string");
  std::string global_path_topic;
  nh.param<std::string>("global_path_topic", global_path_topic, "/pct_path");
  nh.param("traj_server/time_forward", time_forward_, -1.0);

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);

  ros::Subscriber odom_sub = node.subscribe<nav_msgs::Odometry>(odom_topic, 10, boost::bind(odometryCallback, _1, &tf_listener_));

  pos_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.1), cmdCallback);

  ros::Subscriber waypoint_sub_ = node.subscribe(global_path_topic, 1, waypointCallback);

  control_point_state_pub_ = node.advertise<nav_msgs::Odometry>("/control_point_state_", 10);

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
