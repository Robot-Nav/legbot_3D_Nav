#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline_opt/uniform_bspline.h>
#include <iostream>
#include <traj_utils/polynomial_traj.h>
#include <legbot_runtime/runtime.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdlib.h>

using std::vector;
namespace scan_planner
{
  class PlanningVisualization
  {
  private:
    std::string frame_id_;
    legbot::NodeHandle node;

    legbot::Publisher goal_point_pub;
    legbot::Publisher manual_ground_goal_pub;
    legbot::Publisher global_list_pub;
    legbot::Publisher init_list_pub;
    legbot::Publisher optimal_list_pub;
    legbot::Publisher a_star_list_pub;
    legbot::Publisher local_target_pub;
    legbot::Publisher heading_vector_pub;

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(legbot::NodeHandle &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(legbot::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id);
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayManualGroundGoal(double x, double y, double ground_z);
    void displayLocalTarget(const Eigen::Vector3d &local_target);
    void displayHeadingVector(const Eigen::Vector3d &origin, const Eigen::Vector3d &target);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayOptimalTraj(UniformBspline position_traj, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(legbot::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    // void displayIntermediateState(legbot::Publisher& intermediate_pub, scan_planner::msg::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(legbot::Publisher& guide_vector_pub, scan_planner::msg::BsplineOptimizer::Ptr optimizer);
  };
} // namespace scan_planner
#endif
