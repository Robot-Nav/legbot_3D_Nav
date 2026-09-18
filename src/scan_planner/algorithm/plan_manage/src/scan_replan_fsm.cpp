
#include <plan_manage/scan_replan_fsm.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace scan_planner
{

  void SCANReplanFSM::init(legbot::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    reference_progress_time_ = 0.0;
    last_freeze_update_time_ = legbot::Time::now();

    /*  fsm param  */
    nh.param("fsm/navi_mode", navi_mode_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/finish_dist", finish_dist_, 0.35);
    nh.param("fsm/finish_dist_z", finish_dist_z_, 0.25);
    nh.param("fsm/planning_horizon", planning_horizon_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/max_replan_fail_count", max_replan_fail_count_, 1000);
    nh.param("fsm/replan_retry_delay", replan_retry_delay_, 0.5);
    nh.param("grid_map/obstacles_inflation_z_up", self_inflation_z_up_, 0.0);
    nh.param("grid_map/obstacles_inflation_z_down", self_inflation_z_down_, 0.0);
    nh.param("grid_map/double_cylinder_radius", self_double_cylinder_radius_, 0.0);
    nh.param("grid_map/double_cylinder_offset", self_double_cylinder_offset_, 0.0);
    nh.param("fsm/z_projection_weight", z_projection_weight_, 8.0);
    nh.param("fsm/align_reference_height_to_odom", align_reference_height_to_odom_, true);
    nh.param("fsm/manual_goal_use_message_z", manual_goal_use_message_z_, false);
    nh.param("fsm/manual_goal_visual_standing_height",
             manual_goal_visual_standing_height_, 0.30);
    nh.param("fsm/odom_twist_in_body_frame", odom_twist_in_body_frame_, true);
    nh.param("fsm/max_reference_height_alignment", max_reference_height_alignment_, 0.8);
    nh.param("grid_map/frame_id", self_inflation_frame_id_, std::string("world"));

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      nh.param("fsm/waypoint_num", waypoint_num_, -1);

      if (waypoint_num_ <= 0)
      {
        ROS_ERROR("[SCANReplanFSM] navi_mode=2 requires keypoints_yaml with fsm/waypoint_num and fsm/waypoint{i}_{x,y,z}.");
        legbot::shutdown();
        return;
      }
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        nh.param("fsm/waypoint" + to_string(i) + "_x", preset_waypoints_[i](0), -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_y", preset_waypoints_[i](1), -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_z", preset_waypoints_[i](2), -1.0);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(legbot::Duration(0.01), &SCANReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(legbot::Duration(0.05), &SCANReplanFSM::checkCollisionCallback, this);

    std::string body_pose_topic;
    legbot::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
    odom_sub_ = nh.subscribe(body_pose_topic, 1, &SCANReplanFSM::odometryCallback, this);
    go2_execution_frozen_sub_ = nh.subscribe("/planning/go2_execution_frozen", 10, &SCANReplanFSM::go2ExecutionFrozenCallback, this);

    bspline_pub_ = nh.advertise<scan_planner::msg::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<scan_planner::msg::DataDisp>("/planning/data_display", 100);
    self_inflation_pub_ = nh.advertise<visualization_msgs::msg::Marker>("self_inflation", 10, true);

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &SCANReplanFSM::rvizGoalCallback, this);
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      legbot::Duration(1.0).sleep();
      while (legbot::ok() && !have_odom_)
        legbot::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = nh.subscribe("/initial_path", 1, &SCANReplanFSM::pathCallback, this);
    else
      cout << "Wrong navi_mode_ value! navi_mode_=" << navi_mode_ << endl;
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      legbot::Duration(0.001).sleep();
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory to first preset waypoint!");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      ROS_WARN("[SCANReplanFSM] Ignore RViz goal before receiving initial body pose.");
      return;
    }

    nav_msgs::msg::Path::SharedPtr path(new nav_msgs::msg::Path);
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[waypointCallback] Empty waypoint message, ignore.");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;
    // Each RViz/queue click creates a fresh point-to-point global reference.
    // Progress is a time coordinate on that reference and cannot be reused
    // from the preceding waypoint.  A stale value samples past the start of
    // the new polynomial and can select a local target behind the robot.
    reference_progress_time_ = 0.0;

    bool success = false;
    // The first odometry sample can arrive while the quadruped is still in
    // its crouch/stand transition.  Reusing that cached height later places
    // a clicked ground-plane goal below the walking base and can make the
    // endpoint look occupied.  Keep the clicked XY, but use the current base
    // height unless the caller explicitly supplies Z.
    double goal_z = odom_pos_(2);
    if (manual_goal_use_message_z_ && std::isfinite(msg->poses[0].pose.position.z))
      goal_z = msg->poses[0].pose.position.z;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, goal_z;
    manual_planning_z_ = goal_z;
    manual_goal_initial_z_ = goal_z;
    manual_clicked_xy_ << msg->poses[0].pose.position.x,
                          msg->poses[0].pose.position.y;
    ROS_INFO("[SCAN manual goal] target=[%.2f, %.2f, %.2f], message_z=%s.",
             end_pt_(0), end_pt_(1), end_pt_(2), manual_goal_use_message_z_ ? "enabled" : "disabled");
    // A queued/manual waypoint is released only after the preceding point has
    // settled.  FAST-LIO's instantaneous twist on a walking quadruped can
    // contain large footfall spikes even when net body motion is near zero;
    // using that spike as a polynomial boundary condition creates severe
    // overshoot for a short next segment.  Start every fresh manual segment
    // from rest and use measured/planned velocity only for in-flight replans.
    success = planner_manager_->planGlobalTraj(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      success = checkGlobalTargetOccupancy();
    }

    double visual_ground_z = odom_pos_.z() - manual_goal_visual_standing_height_;
    visualization_->displayManualGroundGoal(msg->poses[0].pose.position.x,
                                            msg->poses[0].pose.position.y,
                                            visual_ground_z);
    ROS_INFO("[SCAN manual goal] XY click estimated floor marker z=%.2f; "
             "body planning target z=%.2f.",
             visual_ground_z, end_pt_.z());
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      replan_fail_count_ = 0;
      need_hover_stop_ = false;
      flag_escape_emergency_ = true;
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ != INIT)
      {
        // A fresh manual goal is also the recovery command after a failed
        // replan.  Previously goals received in REPLAN_TRAJ or
        // EMERGENCY_STOP updated end_pt_ but left the FSM trapped in its old
        // state, so every later RViz click was ignored by execution.
        have_new_target_ = true;
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.size() < 2)
    {
      ROS_WARN("[planGlobalTrajByWaypoints] Reference path requires at least two points.");
      return false;
    }

    end_pt_ = waypoints.back();
    std::vector<Eigen::Vector3d> reference_waypoints(waypoints.begin() + 1, waypoints.end());

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      legbot::Duration(0.001).sleep();
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        waypoints.front(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        reference_waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("Unable to generate global trajectory from waypoints!");
      return false;
    }

    if (!checkGlobalTargetOccupancy())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      ROS_WARN("[navi_mode=%d] No active waypoint to plan.", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("[navi_mode=%d] Unable to generate trajectory to waypoint %d.", navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!checkGlobalTargetOccupancy())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    ROS_INFO("[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f].",
             navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::advanceReferencePath()
  {
    if (referenceGoalReached())
    {
      have_target_ = false;
      trigger_ = false;
      changeFSMExecState(WAIT_TARGET, "FSM");
      return true;
    }

    if (planFromCurrentTraj())
    {
      replan_fail_count_ = 0;
      changeFSMExecState(EXEC_TRAJ, "FSM");
      return true;
    }

    replan_fail_count_++;
    changeFSMExecState(REPLAN_TRAJ, "FSM");
    return false;
  }

  bool SCANReplanFSM::referenceGoalReached() const
  {
    return (end_pt_.head<2>() - odom_pos_.head<2>()).norm() <= finish_dist_ &&
           goalFloorMatches();
  }

  bool SCANReplanFSM::goalFloorMatches() const
  {
    // Reaching a waypoint is a horizontal navigation decision. Z is only a
    // coarse floor discriminator for explicit 3D/manual goals and stored
    // multi-floor routes. A normal RViz 2D goal is projected to the current
    // floor and must not fail because its message carries z=0 or FAST-LIO's
    // arbitrary odom origin drifts vertically.
    const bool floor_aware =
        navi_mode_ != NAVI_MODE::MANUAL_TARGET || manual_goal_use_message_z_;
    return !floor_aware ||
           std::abs(end_pt_.z() - odom_pos_.z()) <= finish_dist_z_;
  }

  bool SCANReplanFSM::checkGlobalTargetOccupancy()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    // The requested endpoint is mission state, not a disposable planner
    // sample.  Replacing end_pt_ with a backward free point made the FSM
    // announce success there while the GO2 adapter still waited for the
    // original /scan/goal.  That left the learned walking policy active with
    // zero cmd_vel and produced both premature stops and post-arrival sway.
    // getLocalTarget() already selects a collision-free point along this
    // reference. Preserve the real endpoint and let subsequent scans/replans
    // retry it; completion remains tied to measured distance to end_pt_.
    ROS_WARN_THROTTLE(1.0,
                      "[global target] Requested target [%.2f, %.2f, %.2f] is currently "
                      "occupied. Keeping it as the mission goal; the local target will "
                      "stop at a free point and retry as the map clears.",
                      end_pt_(0), end_pt_(1), end_pt_(2));
    return true;
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[pathCallback] Received empty /initial_path, ignore.");
      return;
    }

    if (!have_odom_)
    {
      pending_path_ = msg;
      ROS_WARN_THROTTLE(1.0, "[pathCallback] No odometry yet, cannot plan global trajectory.");
      return;
    }

    std::vector<Eigen::Vector3d> raw_waypoints;
    raw_waypoints.reserve(msg->poses.size());
    for (const auto &pose_stamped : msg->poses)
      raw_waypoints.emplace_back(pose_stamped.pose.position.x,
                                 pose_stamped.pose.position.y,
                                 pose_stamped.pose.position.z);

    // PCT/tomogram Z can use a terrain/reference convention offset from the
    // measured robot-base Z.  SCAN uses Z to disambiguate overlapping floors,
    // so leaving that constant bias in place makes progress lag and eventually
    // sends the local target behind the robot on stairs.  Calibrate the bias
    // once at the route start while preserving every relative height change.
    if (align_reference_height_to_odom_ && !raw_waypoints.empty())
    {
      const double z_bias = odom_pos_.z() - raw_waypoints.front().z();
      if (std::abs(z_bias) <= std::max(0.0, max_reference_height_alignment_))
      {
        for (auto &waypoint : raw_waypoints)
          waypoint.z() += z_bias;
        ROS_INFO("[SCAN] Align reference Z to odom by %.3f m (first path %.3f -> %.3f).",
                 z_bias, raw_waypoints.front().z() - z_bias, raw_waypoints.front().z());
      }
      else
      {
        ROS_WARN("[SCAN] Reference Z bias %.3f m exceeds limit %.3f m; keep original heights.",
                 z_bias, max_reference_height_alignment_);
      }
    }

    end_pt_ = raw_waypoints.back();
    reference_progress_time_ = 0.0;

    // Height convention alignment above is data-driven.  Do not add another
    // hard-coded body-height offset here.
    constexpr double min_dist = 0.50;
    constexpr double duplicate_eps = 1e-3;
    constexpr double corner_angle = 20.0 * M_PI / 180.0;
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(raw_waypoints.size() + 1);
    waypoints.push_back(odom_pos_);
    Eigen::Vector3d last_selected = odom_pos_;

    for (size_t i = 0; i < raw_waypoints.size(); ++i)
    {
      const bool terminal = i + 1 == raw_waypoints.size();
      bool is_corner = false;
      if (i > 0 && i + 1 < raw_waypoints.size())
      {
        const Eigen::Vector3d incoming = raw_waypoints[i] - raw_waypoints[i - 1];
        const Eigen::Vector3d outgoing = raw_waypoints[i + 1] - raw_waypoints[i];
        if (incoming.norm() > duplicate_eps && outgoing.norm() > duplicate_eps)
        {
          const double cosine = std::max(-1.0, std::min(1.0, incoming.normalized().dot(outgoing.normalized())));
          is_corner = std::acos(cosine) > corner_angle;
        }
      }

      const double distance = (raw_waypoints[i] - last_selected).norm();
      if (distance > duplicate_eps && (distance >= min_dist || is_corner || terminal))
      {
        waypoints.push_back(raw_waypoints[i]);
        last_selected = raw_waypoints[i];
      }
    }

    if (waypoints.size() < 2)
    {
      ROS_INFO("[SCAN] Reference-path goal is already reached.");
      have_target_ = false;
      trigger_ = false;
      return;
    }

    trigger_ = true;

    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      ROS_INFO("==========================================\n");
    }
    else
    {
      ROS_ERROR("❌ Unable to generate global trajectory!");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    const auto &v = msg->twist.twist.linear;
    const double q_norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    const bool valid =
        std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        std::isfinite(q_norm) && q_norm >= 0.5 && q_norm <= 1.5 &&
        std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
        std::max({std::abs(p.x), std::abs(p.y), std::abs(p.z)}) <= 10000.0 &&
        std::hypot(v.x, v.y, v.z) <= 20.0;
    if (!valid)
    {
      ROS_WARN_THROTTLE(1.0, "[SCANReplanFSM] Reject invalid odometry sample.");
      return;
    }
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      ROS_INFO("[SCANReplanFSM] Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;
    if (odom_orient_.norm() > 1e-6)
      odom_orient_.normalize();
    else
      odom_orient_.setIdentity();

    const Eigen::Vector3d reported_velocity(msg->twist.twist.linear.x,
                                            msg->twist.twist.linear.y,
                                            msg->twist.twist.linear.z);
    // nav_msgs/Odometry defines twist in child_frame_id.  SCAN's polynomial
    // boundary state is expressed in the odom/world frame, so rotate the
    // FAST-LIO base-frame velocity before using it for global and local plans.
    odom_vel_ = odom_twist_in_body_frame_
                    ? odom_orient_.toRotationMatrix() * reported_velocity
                    : reported_velocity;

    //odom_acc_ = estimateAcc( msg );

    have_odom_ = true;
      if (pending_path_) { auto pending = pending_path_; pending_path_.reset(); pathCallback(pending); }
    publishSelfInflationMarker();
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const legbot::Time now = legbot::Time::now();
    double dt = (now - last_freeze_update_time_).toSec();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.toSec() > 1e-5)
      info->start_time_ += legbot::Duration(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = legbot::Time::now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = legbot::Duration(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_.publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_.publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback(const legbot::TimerEvent &e)
  {
    updateLocalTrajTimeFreeze();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if (!isWaypointSequenceMode() && referenceGoalReached())
      {
        callEmergencyStop(odom_pos_);
        replan_fail_count_ = 0;
        have_target_ = false;
        trigger_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM_GOAL_REACHED");
        break;
      }

      if (isWaypointSequenceMode() && referenceGoalReached())
      {
        if (current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (!planNextWaypoint())
            replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM_WAYPOINT_REACHED");
        }
        else
        {
          callEmergencyStop(odom_pos_);
          active_waypoints_.clear();
          current_wp_ = 0;
          replan_fail_count_ = 0;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM_GOAL_REACHED");
        }
        break;
      }

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      legbot::Time time_now = legbot::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (!isWaypointSequenceMode() && referenceGoalReached())
      {
        callEmergencyStop(odom_pos_);
        replan_fail_count_ = 0;
        have_target_ = false;
        trigger_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM_GOAL_REACHED");
        break;
      }

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_.head<2>() - odom_pos_.head<2>()).norm() < 0.5 &&
          goalFloorMatches())
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 >= (int)active_waypoints_.size() &&
          referenceGoalReached())
      {
        callEmergencyStop(odom_pos_);
        active_waypoints_.clear();
        current_wp_ = 0;
        replan_fail_count_ = 0;
        have_target_ = false;
        trigger_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM_GOAL_REACHED");
        break;
      }

      // Upstream SCAN treats expiry of the planned trajectory time as mission
      // completion.  That is reasonable for its time-synchronised aerial
      // controller, but not for GO2: locomotion lag means the measured robot
      // pose can still be outside the waypoint tolerance.  Keep the target
      // and replan from the measured/current state until the XY arrival checks
      // above succeed.  Otherwise the FSM drops into WAIT_TARGET while the
      // adapter continues tracking the last spline and oscillates around it.
      if (t_cur > info->duration_ - 1e-2)
      {
        if (planFromCurrentTraj())
        {
          replan_fail_count_ = 0;
          changeFSMExecState(EXEC_TRAJ, "FSM_TRAJ_EXPIRED_NOT_REACHED");
          return;
        }

        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM_TRAJ_EXPIRED_NOT_REACHED");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1 &&
            legbot::Time::now() >= replan_retry_not_before_)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          ROS_INFO("Exiting EMERGENCY_STOP. Switching to WAIT_TARGET. Need a new target point.");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = legbot::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      ROS_WARN("Replan failed %d times. Stop, keep the current target, and retry after %.2fs.",
               replan_fail_count_, replan_retry_delay_);
      replan_fail_count_ = 0;
      // A GO2 can briefly enter the inflated footprint because of locomotion
      // lag or a stale near-field voxel.  Dropping the mission here leaves it
      // permanently stuck in WAIT_TARGET.  Hold still so ray clearing can
      // update the map, then retry the same target with a bounded backoff.
      need_hover_stop_ = false;
      replan_retry_not_before_ = legbot::Time::now() +
                                 legbot::Duration(std::max(0.0, replan_retry_delay_));
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    legbot::Time time_now = legbot::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      // Replan from measured state.  Starting from the old planned state can
      // accumulate tracking error and eventually detach the local trajectory
      // from the robot on stairs.
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // The learned locomotion controller can briefly move faster than its
      // command during a stair impact.  Passing that measured overspeed into
      // a planner configured for a lower maximum makes every candidate
      // dynamically infeasible.  Preserve its direction while projecting the
      // boundary state back into the planner's admissible velocity set.
      const double max_start_speed = std::max(0.0, planner_manager_->pp_.max_vel_);
      const double measured_speed = start_vel_.norm();
      if (max_start_speed > 0.0 && measured_speed > max_start_speed)
      {
        start_vel_ *= max_start_speed / measured_speed;
        ROS_WARN_THROTTLE(1.0,
                          "Clamp measured replan speed from %.3f to %.3f m/s.",
                          measured_speed, max_start_speed);
      }

      bool success = callReboundReplan(false, false);
      if (!success)
      {
        success = callReboundReplan(true, false);
        if (!success)
        {
          success = callReboundReplan(true, true);
          if (!success)
            return false;
        }
      }

      return true;
    }

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    // The global trajectory is a geometric reference, not the command that
    // enforces continuity.  Feeding the outgoing velocity/acceleration of the
    // previous avoidance spline into this reference bends it farther away
    // from the direct route on every replan.  Keep the refreshed reference
    // rest-to-rest (and therefore straight for a manual point-to-point goal),
    // while reboundReplan() below still receives start_vel_/start_acc_ and
    // produces a dynamically continuous local B-spline.
    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(),
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      ROS_ERROR("[navi_mode=%d] Unable to refresh global trajectory from odom to current target.", navi_mode_);
      return false;
    }

    if (!checkGlobalTargetOccupancy())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    if (have_new_target_ && navi_mode_ == NAVI_MODE::MANUAL_TARGET)
    {
      start_vel_.setZero();
      return;
    }

    LocalTrajData *info = &planner_manager_->local_data_;
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      return;
    if (info->start_time_.toSec() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (legbot::Time::now() - info->start_time_).toSec();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback(const legbot::TimerEvent &e)
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ != EXEC_TRAJ || !have_odom_ || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = std::max(0.0, (legbot::Time::now() - info->start_time_).toSec());
    for (double t = t_cur;; t += time_step)
    {
      const double sample_t = std::min(t, info->duration_);
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(sample_t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(
          std::min(sample_t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (sample_t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", sample_t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
      if (sample_t >= info->duration_)
        break;
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    // Do not hand an occupied target to A*/B-spline optimization. If the
    // reference has no free point in the local window, keep the robot stopped
    // and retry after the occupancy map changes.
    if (planner_manager_->grid_map_->getInflateOccupancy(
            local_target_pt_, estimateYawFromSegment(odom_pos_, local_target_pt_)) != 0)
    {
      ROS_WARN_THROTTLE(1.0, "No collision-free local target; defer replan until the map clears.");
      return false;
    }

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    if (plan_success)
      have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void SCANReplanFSM::getLocalTarget()
  {
    const bool manual_xy_only = navi_mode_ == NAVI_MODE::MANUAL_TARGET &&
                                !manual_goal_use_message_z_;
    if (manual_xy_only)
    {
      // A 2-D click is an XY destination, not a fixed-altitude drone goal.
      // Follow the measured walking-base height slowly enough to suppress
      // footfall noise, instead of keeping the height sampled at click time.
      const double next_z = manual_planning_z_ +
          std::clamp(0.25 * (odom_pos_.z() - manual_planning_z_), -0.05, 0.05);
      if (std::abs(odom_pos_.z() - manual_goal_initial_z_) > 0.40)
        ROS_WARN_THROTTLE(5.0,
                          "[SCAN manual goal] Base Z changed by %.2f m since this 2-D click. "
                          "Local planning stays XY-projected; check FAST-LIO on flat ground.",
                          odom_pos_.z() - manual_goal_initial_z_);
      if (std::abs(next_z - end_pt_.z()) > 0.02)
      {
        end_pt_.z() = next_z;
        visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1),
                                          0.3, 0);
        double visual_floor_z = next_z - manual_goal_visual_standing_height_;
        visualization_->displayManualGroundGoal(manual_clicked_xy_.x(),
                                                manual_clicked_xy_.y(), visual_floor_z);
      }
      manual_planning_z_ = next_z;
    }
    auto projectedReference = [&](Eigen::Vector3d point) {
      if (manual_xy_only)
        point.z() = manual_planning_z_;
      return point;
    };
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double max_acc = planner_manager_->pp_.max_acc_;
    const double duration = planner_manager_->global_data_.global_duration_;
    double t_step = max_vel > 1e-6 ? planning_horizon_ / 20.0 / max_vel : 0.01;
    t_step = std::max(t_step, 0.01);

    double t_proj = reference_progress_time_;
    double min_dist_to_start = 9999.0;
    const double search_start = std::max(0.0, reference_progress_time_ - 0.2);
    for (double t = search_start; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = projectedReference(
          planner_manager_->global_data_.getPosition(t));
      const double dist_xy = (pos_t.head<2>() - start_pt_.head<2>()).norm();
      const double dist_z = std::abs(pos_t.z() - start_pt_.z());
      const double dist_to_start = dist_xy +
          (manual_xy_only ? 0.0 : z_projection_weight_ * dist_z);
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        t_proj = t;
      }
    }
    t_proj = std::max(reference_progress_time_, t_proj);
    reference_progress_time_ = t_proj;
    double target_t = duration;
    double total_dist = 0.0;
    bool target_found = false;
    Eigen::Vector3d prev_pos = projectedReference(
        planner_manager_->global_data_.getPosition(t_proj));
    local_target_pt_ = projectedReference(end_pt_);

    for (double t = t_proj; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = projectedReference(
          planner_manager_->global_data_.getPosition(t));
      total_dist += manual_xy_only
          ? (pos_t.head<2>() - prev_pos.head<2>()).norm()
          : (pos_t - prev_pos).norm();
      if (total_dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        target_found = true;
        break;
      }
      prev_pos = pos_t;
    }
    planner_manager_->global_data_.last_progress_time_ = target_found ? target_t : duration;

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    // A point immediately behind an obstacle can have a free centre voxel but
    // still be an unusable B-spline endpoint.  A cubic spline needs several
    // terminal control points after the collision segment; without that room,
    // rebound/A* cannot obtain a free out point and repeatedly emits an
    // emergency-stop trajectory.  Require a short, free approach band behind
    // the candidate target.  Inflation already contains the GO2 footprint, so
    // this is endpoint support rather than another body-radius inflation.
    const double terminal_clearance = std::max(
        2.0 * planner_manager_->pp_.ctrl_pt_dist,
        3.0 * planner_manager_->grid_map_->getResolution());
    auto targetHasTerminalClearance = [&](double candidate_t,
                                          const Eigen::Vector3d &candidate_pt) {
      if (targetOccupancy(candidate_pt) != 0)
        return false;

      double checked_distance = 0.0;
      Eigen::Vector3d previous_pt = candidate_pt;
      for (double t = candidate_t - t_step;
           t >= std::max(0.0, t_proj) - 1e-6 && checked_distance < terminal_clearance;
           t -= t_step)
      {
        const double sample_t = std::max(std::max(0.0, t_proj), t);
        const Eigen::Vector3d sample_pt = projectedReference(
            planner_manager_->global_data_.getPosition(sample_t));
        if (targetOccupancy(sample_pt) != 0)
          return false;

        checked_distance += manual_xy_only
            ? (previous_pt.head<2>() - sample_pt.head<2>()).norm()
            : (previous_pt - sample_pt).norm();
        previous_pt = sample_pt;
      }
      return true;
    };

    if (!targetHasTerminalClearance(target_t, local_target_pt_))
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      // Prefer a point farther along the reference. This keeps the obstacle
      // between start and endpoint so SCAN can generate a real avoidance path.
      // Interleaving forward/backward candidates used to select a point just
      // before the obstacle and made progress stall at the next replan.
      for (double t_forward = target_t;; t_forward += t_step)
      {
        const double sample_t = std::min(
            t_forward, planner_manager_->global_data_.global_duration_);
        Eigen::Vector3d pt = projectedReference(
            planner_manager_->global_data_.getPosition(sample_t));
        if (targetHasTerminalClearance(sample_t, pt))
        {
          local_target_pt_ = pt;
          adjusted_t = sample_t;
          found_free_target = true;
          break;
        }
        if (sample_t >= planner_manager_->global_data_.global_duration_ - 1e-6)
          break;
      }

      // If the remaining route ends inside an obstacle, fall back to a safe
      // point before it. Keep the real mission endpoint unchanged so a later
      // map update or a replacement goal can resume normally.
      if (!found_free_target)
      {
        for (double t_backward = target_t - t_step;; t_backward -= t_step)
        {
          const double sample_t = std::max(std::max(0.0, t_proj), t_backward);
          Eigen::Vector3d pt = projectedReference(
              planner_manager_->global_data_.getPosition(sample_t));
          if (targetHasTerminalClearance(sample_t, pt))
          {
            local_target_pt_ = pt;
            adjusted_t = sample_t;
            found_free_target = true;
            break;
          }
          if (sample_t <= std::max(0.0, t_proj) + 1e-6)
            break;
        }
      }

      if (found_free_target)
      {
        ROS_WARN_THROTTLE(1.0,
                          "Local target lacks terminal clearance; adjusted it to a supported free point.");
        target_t = adjusted_t;
      }
      else
      {
        ROS_WARN_THROTTLE(1.0,
                          "Local target lacks terminal clearance and no supported free target was found.");
      }
    }

    const double stop_dist = max_acc > 1e-6 ? (max_vel * max_vel) / (2.0 * max_acc) : 0.0;
    if ((end_pt_ - local_target_pt_).norm() < stop_dist)
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      if (manual_xy_only)
        local_target_vel_.z() = 0.0;
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;
      // cout << "AA" << endl;
    }

    // Keep the moving planning-horizon target separate from the current
    // reference waypoint, and visualize the corresponding horizontal yaw.
    visualization_->displayLocalTarget(local_target_pt_);
    visualization_->displayHeadingVector(odom_pos_, local_target_pt_);
  }

} // namespace scan_planner
