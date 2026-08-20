#include <traj_utils/planning_visualization.h>
#include <cmath>
#include <limits>
#include <std_msgs/ColorRGBA.h>

using std::cout;
using std::endl;
namespace scan_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;
    nh.param("grid_map/frame_id", frame_id_, std::string("map"));

    goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list", 2);
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);
    local_target_pub = nh.advertise<visualization_msgs::Marker>("local_target", 2, true);
    heading_vector_pub = nh.advertise<visualization_msgs::Marker>("heading_vector", 2, true);
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = frame_id_;
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    pub.publish(sphere);
    pub.publish(line_strip);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = frame_id_;
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = frame_id_;
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = frame_id_;
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub.publish(sphere);
  }

  void PlanningVisualization::displayLocalTarget(const Eigen::Vector3d &local_target)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = frame_id_;
    sphere.header.stamp = ros::Time::now();
    sphere.ns = "scan_local_target";
    sphere.id = 0;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.orientation.w = 1.0;
    sphere.pose.position.x = local_target.x();
    sphere.pose.position.y = local_target.y();
    sphere.pose.position.z = local_target.z();
    sphere.scale.x = 0.24;
    sphere.scale.y = 0.24;
    sphere.scale.z = 0.24;
    sphere.color.r = 1.0;
    sphere.color.g = 0.35;
    sphere.color.b = 0.0;
    sphere.color.a = 1.0;
    local_target_pub.publish(sphere);
  }

  void PlanningVisualization::displayHeadingVector(const Eigen::Vector3d &origin, const Eigen::Vector3d &target)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = frame_id_;
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "scan_heading_vector";
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;

    const Eigen::Vector2d direction = (target - origin).head<2>();
    if (direction.norm() < 1e-6)
    {
      arrow.action = visualization_msgs::Marker::DELETE;
      heading_vector_pub.publish(arrow);
      return;
    }

    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.06;
    arrow.scale.y = 0.14;
    arrow.scale.z = 0.20;
    arrow.color.r = 1.0;
    arrow.color.g = 0.85;
    arrow.color.b = 0.05;
    arrow.color.a = 1.0;

    constexpr double kHeadingLength = 0.8;
    const Eigen::Vector2d unit_direction = direction.normalized();
    geometry_msgs::Point start;
    start.x = origin.x();
    start.y = origin.y();
    start.z = origin.z() + 0.12;
    geometry_msgs::Point end = start;
    end.x += kHeadingLength * unit_direction.x();
    end.y += kHeadingLength * unit_direction.y();
    arrow.points.push_back(start);
    arrow.points.push_back(end);
    heading_vector_pub.publish(arrow);
  }

  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (global_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1, 0, 0, 1);
    displayMarkerList(optimal_list_pub, list, 0.15, color, id);
  }

  void PlanningVisualization::displayOptimalTraj(UniformBspline position_traj, int id)
  {
    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    const double duration = position_traj.getTimeSum();
    if (duration < 1e-6)
    {
      return;
    }

    UniformBspline velocity_traj = position_traj.getDerivative();
    const int sample_num = std::max(2, static_cast<int>(std::ceil(duration / 0.1)) + 1);

    std::vector<Eigen::Vector3d> points;
    std::vector<double> speeds;
    points.reserve(sample_num);
    speeds.reserve(sample_num);

    double min_speed = std::numeric_limits<double>::max();
    double max_speed = 0.0;
    for (int i = 0; i < sample_num; ++i)
    {
      const double t = duration * static_cast<double>(i) / static_cast<double>(sample_num - 1);
      Eigen::Vector3d pos = position_traj.evaluateDeBoorT(t);
      const double speed = velocity_traj.evaluateDeBoorT(t).norm();

      points.push_back(pos);
      speeds.push_back(speed);
      min_speed = std::min(min_speed, speed);
      max_speed = std::max(max_speed, speed);
    }

    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = frame_id_;
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;
    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08;
    line_strip.scale.x = 0.08;

    const double speed_range = std::max(1e-6, max_speed - min_speed);
    for (size_t i = 0; i < points.size(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = points[i](0);
      pt.y = points[i](1);
      pt.z = points[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);

      const double ratio = std::min(1.0, std::max(0.0, (speeds[i] - min_speed) / speed_range));
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      color.r = 1.0;
      color.g = 1.0 - ratio;
      color.b = 0.0;
      sphere.colors.push_back(color);
      line_strip.colors.push_back(color);
    }

    optimal_list_pub.publish(sphere);
    optimal_list_pub.publish(line_strip);
  }

  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    if (a_star_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star paths different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    // for ( int i=0; i<10; i++ )
    // {
    //   //Eigen::Vector4d color(1,1,0,0);
    //   displayMarkerList(a_star_list_pub, list, scale, color, id+i);
    // }

    for (auto block : a_star_paths)
    {
      list.clear();
      for (auto pt : block)
      {
        list.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }

  void PlanningVisualization::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;
    // clear
    pub.publish(array);

    generateArrowDisplayArray(array, list, scale, color, id);

    pub.publish(array);
  }

  // PlanningVisualization::
} // namespace scan_planner
