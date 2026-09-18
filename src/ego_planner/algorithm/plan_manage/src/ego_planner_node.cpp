#include <legbot_runtime/runtime.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  legbot::init(argc, argv, "ego_planner_node");
  legbot::NodeHandle nh("~");


  EGOReplanFSM rebo_replan;

  rebo_replan.init(nh);

  legbot::Duration(1.0).sleep();
  legbot::spin();

  return 0;
}
