#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <legbot_runtime/runtime.hpp>
#include <std_msgs/msg/bool.hpp>
#include <legbot_runtime/transforms.hpp>

#include "bspline_opt/uniform_bspline.h"
#include <scan_planner/msg/bspline.hpp>

namespace
{
using scan_planner::UniformBspline;

legbot::Publisher cmd_vel_pub;
legbot::Publisher execution_frozen_pub;
legbot::Publisher goal_stop_ack_pub;
legbot::Subscriber bspline_sub;
legbot::Subscriber odom_sub;
legbot::Subscriber goal_sub;
legbot::Timer cmd_timer;

bool receive_traj = false;
bool have_odom = false;
legbot::Time last_odom_time;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
double last_nearest_time = 0.0;
int traj_id = 0;
legbot::Time latest_goal_time;
legbot::Time latest_traj_start;
bool have_goal_time = false;
bool have_goal_position = false;
bool goal_stop_latched = false;
bool goal_ack_sent = false;
geometry_msgs::msg::PoseStamped latest_goal_msg;
Eigen::Vector3d latest_goal_pos = Eigen::Vector3d::Zero();

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;

double time_forward;
double heading_error_threshold;
double rotate_exit_ratio;
double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
double finish_dist;
double finish_dist_z;
bool use_goal_z;
double tracking_z_weight;
bool steer_with_position_error;
double odom_timeout;
double slope_up_threshold;
double slope_down_threshold;
double stair_speed_scale;
double min_stair_speed;
double min_walk_speed;
double goal_slowdown_distance;
double turn_slowdown_angle;
double min_turn_speed_scale;
double cmd_accel_limit;
double cmd_yaw_accel_limit;
std::string body_pose_topic;
geometry_msgs::msg::Twist last_motion_cmd;
legbot::Time last_motion_cmd_time;
bool rotate_only = false;
int rotate_direction = 0;

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

void loadParams(const legbot::NodeHandle &nh)
{
  nh.param("time_forward", time_forward, 0.55);
  nh.param("heading_error_threshold", heading_error_threshold, 1.20);
  nh.param("rotate_exit_ratio", rotate_exit_ratio, 0.75);
  nh.param("kp_pos", kp_pos, 0.90);
  nh.param("kp_yaw", kp_yaw, 1.20);
  nh.param("max_vx", max_vx, 0.50);
  nh.param("max_vy", max_vy, 0.25);
  nh.param("max_vyaw", max_vyaw, 1.00);
  nh.param("finish_dist", finish_dist, 0.25);
  nh.param("finish_dist_z", finish_dist_z, 0.20);
  nh.param("use_goal_z", use_goal_z, false);
  // A local B-spline spans only one short, monotonic route segment.  Its
  // nearest-point tracker must not inherit the large Z weight used by the
  // global multi-floor projection: PCT reference height is intentionally
  // above the measured base pose and would pin tracking at t=0 on stairs.
  nh.param("tracking_z_weight", tracking_z_weight, 0.0);
  nh.param("steer_with_position_error", steer_with_position_error, false);
  nh.param("odom_timeout", odom_timeout, 0.5);
  nh.param("slope_up_threshold", slope_up_threshold, 0.30);
  nh.param("slope_down_threshold", slope_down_threshold, -0.30);
  nh.param("stair_speed_scale", stair_speed_scale, 1.00);
  nh.param("min_stair_speed", min_stair_speed, 0.30);
  nh.param("min_walk_speed", min_walk_speed, 0.0);
  nh.param("goal_slowdown_distance", goal_slowdown_distance, 0.80);
  nh.param("turn_slowdown_angle", turn_slowdown_angle, 0.30);
  nh.param("min_turn_speed_scale", min_turn_speed_scale, 0.30);
  nh.param("cmd_accel_limit", cmd_accel_limit, 0.75);
  nh.param("cmd_yaw_accel_limit", cmd_yaw_accel_limit, 2.00);
  legbot::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/Odometry_gazebo"));

  max_vyaw = std::min(std::max(max_vyaw, 0.0), 1.0);
  max_vx = std::min(std::max(max_vx, 0.0), 0.75);
  heading_error_threshold = clamp(heading_error_threshold, 0.10, M_PI);
  min_turn_speed_scale = clamp(min_turn_speed_scale, 0.0, 1.0);
  rotate_exit_ratio = clamp(rotate_exit_ratio, 0.0, 1.0);

  // This adapter commands a holonomic GO2 interface (linear.x + linear.y).
  // Pointing body yaw at the cross-track correction turns a lateral path
  // correction into a large arc and becomes singular at the final point.
  // Keep accepting the legacy launch argument for command-line compatibility,
  // but do not let an old copied command reactivate that unstable behaviour.
  if (steer_with_position_error)
  {
    ROS_WARN("[scan_go2_cmd_adapter] steer_with_position_error=true is incompatible with "
             "holonomic GO2 tracking; forcing spline-tangent steering.");
    steer_with_position_error = false;
  }
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
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  cmd_vel_pub.publish(cmd);
  last_motion_cmd = cmd;
  last_motion_cmd_time = legbot::Time::now();
}

double approach(double current, double target, double maximum_delta)
{
  return current + clamp(target - current, -maximum_delta, maximum_delta);
}

void publishSmooth(const geometry_msgs::msg::Twist &target)
{
  const legbot::Time now = legbot::Time::now();
  double dt = last_motion_cmd_time.isZero() ? 0.01 : (now - last_motion_cmd_time).toSec();
  dt = clamp(dt, 0.001, 0.05);
  geometry_msgs::msg::Twist cmd = target;
  const double linear_delta = std::max(0.0, cmd_accel_limit) * dt;
  const double yaw_delta = std::max(0.0, cmd_yaw_accel_limit) * dt;
  cmd.linear.x = approach(last_motion_cmd.linear.x, target.linear.x, linear_delta);
  cmd.linear.y = approach(last_motion_cmd.linear.y, target.linear.y, linear_delta);
  cmd.angular.z = approach(last_motion_cmd.angular.z, target.angular.z, yaw_delta);
  cmd_vel_pub.publish(cmd);
  last_motion_cmd = cmd;
  last_motion_cmd_time = now;
}

void publishExecutionFrozen(bool frozen)
{
  std_msgs::msg::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
{
  if (!msg)
    return;

  latest_goal_time = legbot::Time(msg->header.stamp);
  if (latest_goal_time.isZero())
    latest_goal_time = legbot::Time::now();
  have_goal_time = true;
  latest_goal_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  have_goal_position = latest_goal_pos.allFinite();
  goal_stop_latched = false;
  goal_ack_sent = false;
  latest_goal_msg = *msg;
  // A new point can be almost exactly behind the robot. Do not carry the
  // preceding goal's rotate hysteresis/direction into that U-turn.
  rotate_only = false;
  rotate_direction = 0;

  // A queued or RViz goal starts a new point-to-point mission.  Never keep
  // driving the preceding B-spline while SCAN is computing (or rejecting)
  // the replacement trajectory.  This also makes an occupied/invalid click
  // fail safe: the quadruped holds position until a fresh B-spline arrives.
  if (latest_traj_start.isZero() || latest_traj_start < latest_goal_time)
  {
    receive_traj = false;
    last_nearest_time = 0.0;
    publishExecutionFrozen(false);
    publishStop();
    ROS_INFO("[scan_go2_cmd_adapter] new goal received; previous trajectory stopped.");
  }
}

void bsplineCallback(const scan_planner::msg::Bspline::ConstSharedPtr &msg)
{
  if (!msg || msg->order <= 0 || msg->pos_pts.size() <= static_cast<size_t>(msg->order) ||
      msg->knots.size() != msg->pos_pts.size() + msg->order + 1)
  {
    ROS_ERROR_THROTTLE(1.0, "[scan_go2_cmd_adapter] Reject malformed B-spline message.");
    return;
  }

  const legbot::Time candidate_start(msg->start_time);
  if (have_goal_time && !candidate_start.isZero() && candidate_start < latest_goal_time)
  {
    ROS_WARN("[scan_go2_cmd_adapter] Ignore trajectory older than the latest goal.");
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
      ROS_ERROR_THROTTLE(1.0, "[scan_go2_cmd_adapter] Reject non-finite B-spline control point.");
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
  latest_traj_start = candidate_start;
  receive_traj = true;
  last_nearest_time = 0.0;

  ROS_INFO("[scan_go2_cmd_adapter] receive traj %d duration %.3fs", traj_id, traj_duration);
}

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
  const auto &p = msg->pose.pose.position;
  const auto &q = msg->pose.pose.orientation;
  const double q_norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q_norm) || q_norm < 0.5 || q_norm > 1.5 ||
      std::max({std::abs(p.x), std::abs(p.y), std::abs(p.z)}) > 10000.0)
  {
    ROS_WARN_THROTTLE(1.0, "[scan_go2_cmd_adapter] Reject invalid odometry sample.");
    return;
  }
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = legbot_tf::getYaw(msg->pose.pose.orientation);
  have_odom = true;
  last_odom_time = legbot::Time::now();
}

void cmdCallback(const legbot::TimerEvent &)
{
  if (!have_odom ||
      (legbot::Time::now() - last_odom_time).toSec() > odom_timeout)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  // The planner publishes short, repeatedly replaced local splines.  Their
  // endpoint/time is not a reliable indication that the mission goal has
  // been reached.  Latch the stop against the actual /scan/goal instead, so
  // locomotion inertia cannot carry the robot across the tolerance and make
  // the position controller command an equally large reverse step forever.
  double goal_xy_distance = std::numeric_limits<double>::infinity();
  if (have_goal_position)
  {
    goal_xy_distance = (latest_goal_pos.head<2>() - odom_pos.head<2>()).norm();
    // XY decides arrival. Optional Z is a broad same-floor gate, not a
    // precision height target.
    const bool inside_goal =
        goal_xy_distance <= finish_dist &&
        (!use_goal_z || std::abs(latest_goal_pos.z() - odom_pos.z()) <= finish_dist_z);
    if (inside_goal && !goal_stop_latched)
      ROS_INFO("[scan_go2_cmd_adapter] final goal reached; latching zero velocity until the next goal.");
    goal_stop_latched = goal_stop_latched || inside_goal;
    if (goal_stop_latched)
    {
      publishExecutionFrozen(false);
      publishStop();
      // A zero cmd_vel is not mission completion: the RL controller remains in
      // walking mode until the waypoint coordinator hears an explicit arrival.
      // Echo the exact goal stamp so an old acknowledgment cannot finish a
      // newer RViz or queued goal.
      if (!goal_ack_sent)
      {
        geometry_msgs::msg::PoseStamped ack = latest_goal_msg;
        ack.pose.position.x = odom_pos.x();
        ack.pose.position.y = odom_pos.y();
        ack.pose.position.z = odom_pos.z();
        goal_stop_ack_pub.publish(ack);
        goal_ack_sent = true;
      }
      return;
    }
  }

  // A goal clicked inside the arrival circle needs no new B-spline, but it
  // still needs the stop acknowledgment above so the RL mode can end.
  if (!receive_traj)
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

  const bool near_traj_end = t_near >= traj_duration - 0.05;
  const Eigen::Vector3d end_pt = traj[0].evaluateDeBoorT(traj_duration);
  const bool reached_end =
      (end_pt.head<2>() - odom_pos.head<2>()).norm() <= finish_dist &&
      (!use_goal_z || std::abs(end_pt.z() - odom_pos.z()) <= finish_dist_z);

  if (near_traj_end && reached_end)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

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
      const double effective_min_stair_speed = std::min(min_stair_speed, max_vx);
      if (effective_min_stair_speed > 0.0 && stair_speed > 0.02 &&
          stair_speed < effective_min_stair_speed)
        vel_world *= effective_min_stair_speed / stair_speed;
    }
  }

  // Some velocity policies intentionally hold a static pose for very small
  // commands. A rest-to-rest B-spline also starts with a very small non-zero
  // feed-forward velocity, so the nearest-point tracker can deadlock at t=0:
  // the policy does not step, and the requested speed never ramps up. Apply a
  // configurable launch speed while the endpoint remains outside its finish
  // tolerance. The normal reached_end branch above still publishes zero.
  const double effective_min_walk_speed = std::min(std::max(min_walk_speed, 0.0), max_vx);
  const double walk_speed = vel_world.norm();
  const bool terminal_approach =
      have_goal_position && goal_xy_distance <= std::max(finish_dist, goal_slowdown_distance);
  const bool needs_minimum_speed =
      !reached_end && !terminal_approach && effective_min_walk_speed > 0.0 &&
      walk_speed > 0.005 && walk_speed < effective_min_walk_speed;
  if (needs_minimum_speed)
    vel_world *= effective_min_walk_speed / walk_speed;

  // Stair minimum-speed handling must never defeat the caller's total speed
  // limit.  Previously a 0.10 m/s experiment could produce a 0.25 m/s
  // lateral command after the minimum-speed rescale.
  vel_world = clampNorm(vel_world, max_vx);

  // A short local spline can still contain feed-forward speed at the mission
  // endpoint.  Bound it explicitly by the remaining goal distance so GO2
  // does not coast through the finish circle and then reverse.  This also
  // makes the result independent of the locomotion policy's small-command
  // dead zone because the goal latch above owns the final zero command.
  double terminal_scale = 1.0;
  if (terminal_approach && goal_slowdown_distance > finish_dist + 1e-3)
  {
    terminal_scale = clamp(
        (goal_xy_distance - finish_dist) /
            (goal_slowdown_distance - finish_dist),
        0.0, 1.0);
    vel_world = clampNorm(vel_world, max_vx * terminal_scale);
  }

  double yaw_des = estimateDesiredYaw(t_near, t_look, pos_near, pos_look);
  // Position-error steering is useful only during coarse tracking.  Near the
  // final point its direction flips by pi as soon as the robot crosses the
  // target, producing the observed left/right oscillation.  Always retain
  // the stable spline-tangent heading in the terminal approach zone.
  if (steer_with_position_error && !terminal_approach && vel_world.norm() > 0.02)
    yaw_des = std::atan2(vel_world(1), vel_world(0));
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);
  const double terminal_yaw_scale = terminal_approach ? terminal_scale : 1.0;

  const double abs_yaw_error = std::abs(yaw_err);
  if (terminal_approach)
  {
    rotate_only = false;
    rotate_direction = 0;
  }
  else if (!rotate_only && abs_yaw_error > heading_error_threshold)
  {
    rotate_only = true;
    // At a nearly 180-degree turn, quaternion/odometry noise can move the
    // wrapped error repeatedly between +pi and -pi. Latch one direction for
    // this rotate-in-place phase so the command cannot alternate left/right.
    rotate_direction = yaw_err >= 0.0 ? 1 : -1;
    ROS_INFO("[scan_go2_cmd_adapter] Enter rotate-in-place: yaw_error=%.2f rad, direction=%s.",
             yaw_err, rotate_direction > 0 ? "CCW" : "CW");
  }
  else if (rotate_only && abs_yaw_error < heading_error_threshold * rotate_exit_ratio)
  {
    rotate_only = false;
    rotate_direction = 0;
    ROS_INFO("[scan_go2_cmd_adapter] Exit rotate-in-place: yaw_error=%.2f rad.", yaw_err);
  }

  const double controlled_yaw_error =
      rotate_only && rotate_direction != 0
          ? static_cast<double>(rotate_direction) * abs_yaw_error
          : yaw_err;
  const double vyaw_cmd = clamp(
      kp_yaw * controlled_yaw_error * terminal_yaw_scale, -max_vyaw, max_vyaw);

  // Separate enter/exit thresholds prevent the controller from alternating
  // between rotate-in-place and translation at the same angular boundary.
  if (rotate_only)
  {
    publishExecutionFrozen(true);
    // Ramp angular velocity just like translation. The old direct publish
    // jumped between 0 and max yaw rate, which made GO2's learned gait look
    // stuck/jerky at waypoint reversals.
    geometry_msgs::msg::Twist rotate_cmd;
    rotate_cmd.angular.z = vyaw_cmd;
    publishSmooth(rotate_cmd);
    ROS_INFO_THROTTLE(1.0,
                      "[scan_go2_cmd_adapter] Rotating: current=%.2f desired=%.2f "
                      "error=%.2f cmd=%.2f rad/s.",
                      odom_yaw, yaw_des, yaw_err, vyaw_cmd);
    return;
  }

  publishExecutionFrozen(false);

  // A quadruped loses stability when a large translational command is mixed
  // with a sharp heading correction. Reduce translation continuously before
  // the rotate-in-place threshold rather than switching abruptly at it.
  if (heading_error_threshold > turn_slowdown_angle &&
      abs_yaw_error > turn_slowdown_angle)
  {
    const double ratio = clamp(
        (heading_error_threshold - abs_yaw_error) /
            (heading_error_threshold - turn_slowdown_angle),
        0.0, 1.0);
    const double scale = min_turn_speed_scale + (1.0 - min_turn_speed_scale) * ratio;
    vel_world *= scale;
  }

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;
  publishSmooth(cmd);
}
} // namespace

int main(int argc, char **argv)
{
  legbot::init(argc, argv, "scan_go2_cmd_adapter");
  legbot::NodeHandle node;
  legbot::NodeHandle nh("~");

  loadParams(nh);

  bspline_sub = node.subscribe("/scan/planning/bspline", 10, bsplineCallback,
                               legbot::TransportHints().tcpNoDelay());
  goal_sub = node.subscribe("/scan/goal", 10, goalCallback,
                            legbot::TransportHints().tcpNoDelay());
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback,
                            legbot::TransportHints().tcpNoDelay());
  cmd_vel_pub = node.advertise<geometry_msgs::msg::Twist>("/cmd_vel", 20);
  execution_frozen_pub = node.advertise<std_msgs::msg::Bool>("/scan/planning/execution_frozen", 10);
  goal_stop_ack_pub = node.advertise<geometry_msgs::msg::PoseStamped>("/scan/goal_stop_ack", 10);
  cmd_timer = node.createTimer(legbot::Duration(0.01), cmdCallback);

  ROS_INFO("[scan_go2_cmd_adapter] ready. bspline=/scan/planning/bspline odom=%s",
           body_pose_topic.c_str());

  legbot::spin();

  cmd_timer.stop();
  bspline_sub.shutdown();
  goal_sub.shutdown();
  odom_sub.shutdown();
  cmd_vel_pub.shutdown();
  execution_frozen_pub.shutdown();
  goal_stop_ack_pub.shutdown();
  return 0;
}
