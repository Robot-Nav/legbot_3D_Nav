#include <legbot_runtime/runtime.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <plan_manage/scan_replan_fsm.h>

using namespace scan_planner;

int main(int argc, char **argv)
{
  legbot::init(argc, argv, "scan_planner_node");
  legbot::NodeHandle nh("~");

  SCANReplanFSM scan_replan;

  scan_replan.init(nh);

  legbot::Duration(1.0).sleep();
  legbot::spin();

  return 0;
}
