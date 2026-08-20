#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#include "bspline_opt/uniform_bspline.h"
#include "scan_planner/Bspline.h"

namespace
{
using scan_planner::UniformBspline;

ros::Publisher cmd_vel_pub;
ros::Publisher execution_frozen_pub;
ros::Subscriber bspline_sub;
ros::Subscriber odom_sub;
ros::Timer cmd_timer;

bool receive_traj = false;
bool have_odom = false;
ros::Time last_odom_time;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
double last_nearest_time = 0.0;
int traj_id = 0;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;

double time_forward;
double heading_error_threshold;
double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
double finish_dist;
double finish_dist_z;
double tracking_z_weight;
double odom_timeout;
double slope_up_threshold;
double slope_down_threshold;
double stair_speed_scale;
double min_stair_speed;
std::string body_pose_topic;

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
{
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-6)
    return value;
  return value / norm * max_norm;
}

void loadParams(const ros::NodeHandle &nh)
{
  nh.param("time_forward", time_forward, 0.55);
  nh.param("heading_error_threshold", heading_error_threshold, 0.80);
  nh.param("kp_pos", kp_pos, 0.90);
  nh.param("kp_yaw", kp_yaw, 1.20);
  nh.param("max_vx", max_vx, 0.50);
  nh.param("max_vy", max_vy, 0.25);
  nh.param("max_vyaw", max_vyaw, 1.00);
  nh.param("finish_dist", finish_dist, 0.25);
  nh.param("finish_dist_z", finish_dist_z, 0.20);
  // A local B-spline spans only one short, monotonic route segment.  Its
  // nearest-point tracker must not inherit the large Z weight used by the
  // global multi-floor projection: PCT reference height is intentionally
  // above the measured base pose and would pin tracking at t=0 on stairs.
  nh.param("tracking_z_weight", tracking_z_weight, 0.0);
  nh.param("odom_timeout", odom_timeout, 0.5);
  nh.param("slope_up_threshold", slope_up_threshold, 0.30);
  nh.param("slope_down_threshold", slope_down_threshold, -0.30);
  nh.param("stair_speed_scale", stair_speed_scale, 1.00);
  nh.param("min_stair_speed", min_stair_speed, 0.30);
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/Odometry_gazebo"));

  max_vyaw = std::min(std::max(max_vyaw, 0.0), 1.0);
}

double findNearestTime(UniformBspline &pos_traj, const Eigen::Vector3d &pos,
                       double t_min, double t_max)
{
  t_min = std::max(t_min, 0.0);
  t_max = std::min(t_max, pos_traj.getTimeSum());

  double best_t = t_min;
  double best_dist = std::numeric_limits<double>::max();
  const double step = 0.01;

  for (double t = t_min; t <= t_max + 1e-6; t += step)
  {
    const double tc = std::min(t, t_max);
    const Eigen::Vector3d p = pos_traj.evaluateDeBoorT(tc);
    const double dist_xy = (p.head<2>() - pos.head<2>()).squaredNorm();
    const double dist_z = p.z() - pos.z();
    const double dist = dist_xy + tracking_z_weight * dist_z * dist_z;
    if (dist < best_dist)
    {
      best_dist = dist;
      best_t = tc;
    }
  }

  return best_t;
}

double estimateDesiredYaw(double t_near, double t_look, const Eigen::Vector3d &pos_near,
                          const Eigen::Vector3d &pos_look)
{
  const Eigen::Vector3d vel = traj[1].evaluateDeBoorT(t_look);
  if (vel.head<2>().norm() > 0.08)
    return std::atan2(vel(1), vel(0));

  const Eigen::Vector3d dir = pos_look - pos_near;
  if (dir.head<2>().norm() > 1e-4)
    return std::atan2(dir(1), dir(0));

  return odom_yaw;
}

void publishStop(double vyaw = 0.0)
{
  geometry_msgs::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  cmd_vel_pub.publish(cmd);
}

void publishExecutionFrozen(bool frozen)
{
  std_msgs::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
{
  if (!msg || msg->order <= 0 || msg->pos_pts.size() <= static_cast<size_t>(msg->order) ||
      msg->knots.size() != msg->pos_pts.size() + msg->order + 1)
  {
    ROS_ERROR_THROTTLE(1.0, "[a1_cmd_adapter] Reject malformed B-spline message.");
    return;
  }

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    if (!std::isfinite(msg->pos_pts[i].x) || !std::isfinite(msg->pos_pts[i].y) ||
        !std::isfinite(msg->pos_pts[i].z))
    {
      ROS_ERROR_THROTTLE(1.0, "[a1_cmd_adapter] Reject non-finite B-spline control point.");
      return;
    }
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = traj[0].getTimeSum();
  traj_id = msg->traj_id;
  receive_traj = true;
  last_nearest_time = 0.0;

  ROS_INFO("[a1_cmd_adapter] receive traj %d duration %.3fs", traj_id, traj_duration);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = tf::getYaw(msg->pose.pose.orientation);
  have_odom = true;
  last_odom_time = ros::Time::now();
}

void cmdCallback(const ros::TimerEvent &)
{
  if (!receive_traj || !have_odom ||
      (ros::Time::now() - last_odom_time).toSec() > odom_timeout)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  const double t_near = findNearestTime(traj[0], odom_pos, last_nearest_time, traj_duration);
  last_nearest_time = std::max(last_nearest_time, t_near);
  const double t_look = std::min(traj_duration, t_near + time_forward);
  const Eigen::Vector3d pos_near = traj[0].evaluateDeBoorT(t_near);
  const Eigen::Vector3d pos_look = traj[0].evaluateDeBoorT(t_look);
  const Eigen::Vector3d vel_look = traj[1].evaluateDeBoorT(t_look);

  const double yaw_des = estimateDesiredYaw(t_near, t_look, pos_near, pos_look);
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);
  const double vyaw_cmd = clamp(kp_yaw * yaw_err, -max_vyaw, max_vyaw);

  const bool near_traj_end = t_near >= traj_duration - 0.05;
  const Eigen::Vector3d end_pt = traj[0].evaluateDeBoorT(traj_duration);
  const bool reached_end =
      (end_pt.head<2>() - odom_pos.head<2>()).norm() <= finish_dist &&
      std::abs(end_pt.z() - odom_pos.z()) <= finish_dist_z;

  if (near_traj_end && reached_end)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  if (std::abs(yaw_err) > heading_error_threshold)
  {
    publishExecutionFrozen(true);
    publishStop(vyaw_cmd);
    return;
  }

  publishExecutionFrozen(false);

  Eigen::Vector2d pos_err = pos_look.head<2>() - odom_pos.head<2>();
  Eigen::Vector2d vel_ff = vel_look.head<2>();
  Eigen::Vector2d vel_world = clampNorm(vel_ff + kp_pos * pos_err, max_vx);

  const double ds = (pos_look.head<2>() - pos_near.head<2>()).norm();
  if (ds > 1e-4)
  {
    const double slope = (pos_look.z() - pos_near.z()) / ds;
    const bool on_stairs = slope > slope_up_threshold || slope < slope_down_threshold;
    if (on_stairs)
    {
      vel_world *= stair_speed_scale;
      // The learned locomotion controller needs enough commanded speed to
      // step onto a riser.  A very short/replanned B-spline can otherwise
      // decay to a small non-zero command: the robot stays against the first
      // riser forever while the planner believes it is still executing.
      const double stair_speed = vel_world.norm();
      if (min_stair_speed > 0.0 && stair_speed > 0.02 && stair_speed < min_stair_speed)
        vel_world *= min_stair_speed / stair_speed;
    }
  }

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;
  cmd_vel_pub.publish(cmd);
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "a1_cmd_adapter");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  loadParams(nh);

  bspline_sub = node.subscribe("/scan/planning/bspline", 10, bsplineCallback,
                               ros::TransportHints().tcpNoDelay());
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback,
                            ros::TransportHints().tcpNoDelay());
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
  execution_frozen_pub = node.advertise<std_msgs::Bool>("/scan/planning/execution_frozen", 10);
  cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  ROS_INFO("[a1_cmd_adapter] ready. bspline=/scan/planning/bspline odom=%s", body_pose_topic.c_str());

  ros::spin();

  cmd_timer.stop();
  bspline_sub.shutdown();
  odom_sub.shutdown();
  cmd_vel_pub.shutdown();
  execution_frozen_pub.shutdown();
  return 0;
}
