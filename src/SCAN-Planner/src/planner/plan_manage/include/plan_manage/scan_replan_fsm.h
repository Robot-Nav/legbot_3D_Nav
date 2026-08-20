#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner/Bspline.h>
#include <scan_planner/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner::DataDisp data_disp_;

    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double finish_dist_, finish_dist_z_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double emergency_time_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double z_projection_weight_;
    bool align_reference_height_to_odom_;
    double max_reference_height_alignment_;
    double reference_progress_time_;
    std::string self_inflation_frame_id_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    ros::Time last_freeze_update_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber goal_sub_, odom_sub_, path_sub_, go2_execution_frozen_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_, self_inflation_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool advanceReferencePath();
    bool referenceGoalReached() const;
    bool adjustGlobalTargetIfOccupied();
    void getLocalTarget();
    void finishProcess();
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    void pathCallback(const nav_msgs::PathConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::BoolConstPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
