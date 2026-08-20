#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/scan_replan_fsm.h>

using namespace scan_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_planner_node");
  ros::NodeHandle nh("~");

  SCANReplanFSM scan_replan;

  scan_replan.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
