#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    have_filtered_odom_vel_ = false;
    flag_escape_emergency_ = true;
    reference_progress_time_ = 0.0;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/odom_velocity_alpha", odom_velocity_alpha_, 0.25);
    nh.param("fsm/max_odom_speed", max_odom_speed_, 1.2);
    nh.param("fsm/odom_height_offset", odom_height_offset_, 0.0);
    odom_velocity_alpha_ = std::max(0.0, std::min(1.0, odom_velocity_alpha_));

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    nh.param<std::string>("fsm/odometry_topic", odom_topic, "/Odometry");

    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.2), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.1), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe(odom_topic, 1, &EGOReplanFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);


    switch (target_type_)
    {
    //手动给予坐标点摸索
    case TARGET_TYPE::MANUAL_TARGET:
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
      break;
    case TARGET_TYPE::PRESET_TARGET:
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      cout << "planGlobalTrajbyGivenWps  in init" << endl;
      planGlobalTrajbyGivenWps();
      break;
    case TARGET_TYPE::REFENCE_PATH:
      //读取path话题生成全局轨迹
      path_sub_= nh.subscribe("/pct_path",1,&EGOReplanFSM::pathCallback,this);
      break;
    default:
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
      break;
    }
     
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      // visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      // A reference path is a complete navigation request.  Without this
      // trigger, the FSM remains in INIT even after receiving /pct_path.
      trigger_ = true;
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }
  

  void EGOReplanFSM::pathCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses.empty())
    {
      ROS_WARN("[EGO] Received empty path from move_base.");
      return;
    }

    if (!have_odom_)
    {
      ROS_WARN("[EGO] No odometry yet, cannot plan global trajectory.");
      return;
    }
    std::vector<Eigen::Vector3d> raw_waypoints;
    raw_waypoints.reserve(msg->poses.size());
    for (const auto &pose_stamped : msg->poses)
      raw_waypoints.emplace_back(pose_stamped.pose.position.x,
                                 pose_stamped.pose.position.y,
                                 pose_stamped.pose.position.z);

    end_pt_ = raw_waypoints.back();
    reference_progress_time_ = 0.0;

    // Remove duplicate/over-dense points while retaining real corners and the
    // exact terminal point.  Duplicate start points otherwise create a zero
    // duration segment in the global minimum-snap trajectory.
    constexpr double min_dist = 0.8;
    constexpr double duplicate_eps = 1e-3;
    constexpr double corner_angle = 20.0 * M_PI / 180.0;
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(raw_waypoints.size());
    Eigen::Vector3d last_selected = odom_pos_;
    for (size_t i = 0; i < raw_waypoints.size(); ++i)
    {
      const bool terminal = i + 1 == raw_waypoints.size();
      bool corner = false;
      if (i > 0 && i + 1 < raw_waypoints.size())
      {
        const Eigen::Vector3d in = raw_waypoints[i] - raw_waypoints[i - 1];
        const Eigen::Vector3d out = raw_waypoints[i + 1] - raw_waypoints[i];
        if (in.norm() > duplicate_eps && out.norm() > duplicate_eps)
        {
          const double cosine = std::max(-1.0, std::min(1.0, in.normalized().dot(out.normalized())));
          corner = std::acos(cosine) > corner_angle;
        }
      }

      const double distance = (raw_waypoints[i] - last_selected).norm();
      if (distance > duplicate_eps && (distance >= min_dist || corner || terminal))
      {
        waypoints.push_back(raw_waypoints[i]);
        last_selected = raw_waypoints[i];
      }
    }

    if (waypoints.empty())
    {
      ROS_INFO("[EGO] Reference-path goal is already reached.");
      have_target_ = false;
      trigger_ = false;
      return;
    }
    // === 2. 计划轨迹 ===
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      trigger_ = true;
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == INIT || exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }
      visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!msg || msg->poses.empty())
      return;
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, odom_pos_(2); //twilight: goal height
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    std::cout<<"run_flag"<<success<<std::endl;

    // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

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
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ){

        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }
              

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
  
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();

    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }



  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    Eigen::Vector3d position(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z + odom_height_offset_);
    Eigen::Vector3d velocity(msg->twist.twist.linear.x,
                             msg->twist.twist.linear.y,
                             0.0);
    try
    {
      tf::StampedTransform transform_odom2map;
      transform_odom2map.setIdentity();
      if (!msg->header.frame_id.empty() && msg->header.frame_id != "map")
      {
        if (!tf_listener_.waitForTransform("map", msg->header.frame_id, ros::Time(0), ros::Duration(0.2)))
        {
          ROS_WARN_THROTTLE(1.0, "[EGO] Waiting for transform %s -> map", msg->header.frame_id.c_str());
          return;
        }
        tf_listener_.lookupTransform("map", msg->header.frame_id, ros::Time(0), transform_odom2map);
      }

      tf::Point pt_odom(position(0), position(1), position(2));
      tf::Point pt_map = transform_odom2map * pt_odom;
      tf::Vector3 vel_odom(velocity(0), velocity(1), velocity(2));
      tf::Vector3 vel_map_tf = transform_odom2map.getBasis() * vel_odom;
      Eigen::Vector3d vel_map(vel_map_tf.x(), vel_map_tf.y(), 0.0);

      if (!std::isfinite(pt_map.x()) || !std::isfinite(pt_map.y()) || !std::isfinite(pt_map.z()) ||
          !vel_map.allFinite())
      {
        ROS_WARN_THROTTLE(1.0, "[EGO] Ignore non-finite odometry sample.");
        return;
      }

      if (!have_filtered_odom_vel_ ||
          (!last_odom_stamp_.isZero() && (msg->header.stamp - last_odom_stamp_).toSec() > 0.5))
      {
        odom_vel_ = vel_map;
        have_filtered_odom_vel_ = true;
      }
      else
      {
        odom_vel_ = odom_velocity_alpha_ * vel_map + (1.0 - odom_velocity_alpha_) * odom_vel_;
      }
      if (max_odom_speed_ > 0.0 && odom_vel_.norm() > max_odom_speed_)
        odom_vel_ = odom_vel_.normalized() * max_odom_speed_;

      odom_pos_ = Eigen::Vector3d(pt_map.x(), pt_map.y(), pt_map.z());
      last_odom_stamp_ = msg->header.stamp;
      have_odom_ = true;

    }
    catch(const tf::TransformException& e)
    {
      ROS_WARN_THROTTLE(1.0, "[EGO] Odometry transform failed: %s", e.what());
      return;
    }
    
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    if (new_state == EMERGENCY_STOP && pre_s != int(EMERGENCY_STOP))
      emergency_start_time_ = ros::Time::now();
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    /*定时器回调，定时查看当前的exec状态，以判断是否规划或是停止*/
    // cout << "execFSMCallback" << endl;
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 10)
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
      // cout << "change state to INIT" <<endl;

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
      cout << " case GEN_NEW_TRAJ:" <<endl;
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (!success && !flag_random_poly_init)
      {
        ROS_WARN("[EGO] Deterministic initial guess failed; retry once with a randomized intermediate point.");
        success = callReboundReplan(true, true);
      }

      if (success)
      {

        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        flag_escape_emergency_ = true;
        changeFSMExecState(EMERGENCY_STOP, "PLAN_FAIL");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      cout << "!!!REPLAN_TRAJ" <<endl;
      if (planFromCurrentTraj())
      {
        // planFromCurrentTraj()
        // cout << "change state to EXEC_TRAJ in '!!!REPLAN_TRAJ'" <<endl;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        flag_escape_emergency_ = true;
        changeFSMExecState(EMERGENCY_STOP, "REPLAN_FAIL");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      cout << "Executing... "<< endl;
      Eigen::Vector3d pos = odom_pos_;

      if ((end_pt_ - pos).norm() < 0.2)
      {
        cout << " near end position" << endl;
        cout << "### change state to WAIT_TARGET in '!!!EXEC_TRAJ'" << endl;
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else
      {
        cout << "### change state to REPLAN_TRAJ in '!!!EXEC_TRAJ'" << endl;

        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_)
      {
        callEmergencyStop(odom_pos_);
        flag_escape_emergency_ = false;
      }
      else if (have_target_ && odom_vel_.norm() < 0.1 &&
               (ros::Time::now() - emergency_start_time_).toSec() >= 0.5)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "EMERGENCY_RECOVERY");
      }

      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {
    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

  // 到目标点才停止replan
    cout << " final plan success="<< endl;
    return true;
;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;

    info->start_pos_ = odom_pos_;

    auto map = planner_manager_->grid_map_;

    if (exec_state_ != EXEC_TRAJ || !have_odom_ || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_total = info->duration_ ;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_total && t >= t_total) 
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        flag_escape_emergency_ = true;
        changeFSMExecState(EMERGENCY_STOP, "COLLISION");
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    cout<<"current pos:"<<start_pt_.transpose()<<endl;
    cout<<"future target:"<<local_target_pt_.transpose()<<endl;

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false; //FIXME

    cout<<"plan_success: "<<plan_success;

    if (plan_success)
    {
      auto info = &planner_manager_->local_data_;
      /* publish traj */
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();//get_control_points();会向前得到t_cur时间段的轨迹
      bspline.pos_pts.reserve(pos_pts.cols());
      
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
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
      // visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }
    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;
    info->start_pos_ = odom_pos_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
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

/**
 * @brief 获取局部目标点的函数
 * 该函数用于在全局路径上选择一个合适的局部目标点，用于局部路径规划
 */
void EGOReplanFSM::getLocalTarget()
{
  // ========== 1. 初始化+安全校验 ==========
  double max_vel = planner_manager_->pp_.max_vel_;
  double max_acc = planner_manager_->pp_.max_acc_;
  
  // 时间步长（安全兜底，避免max_vel为0）
  double t_step = (max_vel > 1e-6) ? (planning_horizen_ / 20 / max_vel) : 0.01;
  t_step = std::max(t_step, 0.01); // 步长不小于10ms，保证精度

  double total_dist = 0.0;        // 从start_pt_开始的累计轨迹路程
  bool target_found = false;      // 是否找到满足条件的目标点
  Eigen::Vector3d local_target = end_pt_; // 默认目标点为全局终点
  double t_proj = 0.0;            // start_pt_在全局轨迹上的投影时间
  double min_dist_to_start = std::numeric_limits<double>::max();

  // ========== 2. 第一步：找到start_pt_在全局轨迹上的投影点（核心） ==========
  // 遍历全局轨迹，找到离start_pt_最近的点（投影点）
  double t;
  const double search_start = std::max(0.0, reference_progress_time_ - 0.2);
  for (t = search_start; t < planner_manager_->global_data_.global_duration_; t += t_step)
  {
    Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
    double dist_to_start = (pos_t - start_pt_).norm();
    
    // 更新投影点（最近点）
    if (dist_to_start < min_dist_to_start)
    {
      min_dist_to_start = dist_to_start;
      t_proj = t;
    }
  }
  t_proj = std::max(reference_progress_time_, t_proj);
  reference_progress_time_ = t_proj;
  ROS_DEBUG("start_pt_投影点时间t_proj=%.3f，投影点到start_pt_距离=%.3f m", 
            t_proj, min_dist_to_start);

  // ========== 3. 第二步：从投影点开始累计路程，寻找局部目标点 ==========
  Eigen::Vector3d prev_pos = planner_manager_->global_data_.getPosition(t_proj); // 投影点位置
  for (t = t_proj; t < planner_manager_->global_data_.global_duration_; t += t_step)
  {
    Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
    
    // 计算当前轨迹段的路程（相邻点间距），累加到总路程
    double seg_dist = (pos_t - prev_pos).norm();
    total_dist += seg_dist;


    // ========== 核心条件：累计路程≥规划范围，确定局部目标点 ==========
    if (total_dist >= planning_horizen_)
    {
      local_target = pos_t;
      target_found = true;
      // 更新进度时间为当前目标点的时间（避免紊乱）
      planner_manager_->global_data_.last_progress_time_ = t;
      break;
    }

    prev_pos = pos_t; // 更新上一个轨迹点
  }

  // ========== 4. 边界处理：未找到目标点（遍历到轨迹末尾） ==========
  if (!target_found)
  {
    local_target = end_pt_;
    planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
  }
  local_target_pt_ = local_target; // 赋值给成员变量

  // ========== 5. 目标速度计算（兼容动力学约束） ==========
  double stop_dist = max_acc > 1e-6 ? (max_vel * max_vel) / (2 * max_acc) : 0.0;
  if ((end_pt_ - local_target_pt_).norm() < stop_dist)
  {
    local_target_vel_ = Eigen::Vector3d::Zero(); // 接近终点，速度归零
  }
  else
  {
    // 取目标点对应时间的速度（安全兜底：不超过轨迹总时长）
    double vel_t = std::min(t, planner_manager_->global_data_.global_duration_);
    local_target_vel_ = planner_manager_->global_data_.getVelocity(vel_t);
    // 速度上限约束
    if (local_target_vel_.norm() > max_vel)
    {
      local_target_vel_ = local_target_vel_.normalized() * max_vel;
    }
  }

}

} // namespace ego_planner
