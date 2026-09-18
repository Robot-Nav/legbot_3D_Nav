#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("go2_mode_manager");
  const auto interface = node->declare_parameter<std::string>("network_interface", "");
  const auto action = node->declare_parameter<std::string>("action", "check");
  const auto confirm_action = node->declare_parameter<bool>("confirm_action", false);
  const auto stand_down_first = node->declare_parameter<bool>("stand_down_first", true);
  const auto select_mode = node->declare_parameter<std::string>("select_mode", "normal");
  if (interface.empty() || interface == "lo") {
    RCLCPP_ERROR(node->get_logger(), "Provide the Ethernet network_interface connected to GO2");
    rclcpp::shutdown();
    return 2;
  }
  if (action != "check" && action != "release" && action != "select") {
    RCLCPP_ERROR(node->get_logger(), "action must be check, release, or select");
    rclcpp::shutdown();
    return 2;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, interface);
  unitree::robot::b2::MotionSwitcherClient motion;
  motion.SetTimeout(5.0f);
  motion.Init();
  std::string form;
  std::string mode;
  int result = motion.CheckMode(form, mode);
  if (result != 0) {
    RCLCPP_ERROR(node->get_logger(), "GO2 CheckMode failed with code %d", result);
    rclcpp::shutdown();
    return 3;
  }
  RCLCPP_INFO(
    node->get_logger(), "GO2 high-level mode: %s (form=%s)",
    mode.empty() ? "released / low-level ready" : mode.c_str(), form.c_str());

  if (action == "release" && !mode.empty()) {
    if (!confirm_action) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Refusing to release GO2 high-level control without confirm_action:=true");
      rclcpp::shutdown();
      return 4;
    }
    if (stand_down_first) {
      unitree::robot::go2::SportClient sport;
      sport.SetTimeout(5.0f);
      sport.Init();
      result = sport.StandDown();
      if (result != 0) {
        RCLCPP_ERROR(node->get_logger(), "GO2 StandDown failed with code %d", result);
        rclcpp::shutdown();
        return 5;
      }
      RCLCPP_INFO(node->get_logger(), "GO2 StandDown accepted; waiting before release");
      std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    result = motion.ReleaseMode();
    if (result != 0) {
      RCLCPP_ERROR(node->get_logger(), "GO2 ReleaseMode failed with code %d", result);
      rclcpp::shutdown();
      return 6;
    }
    for (int attempt = 0; attempt < 20; ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      result = motion.CheckMode(form, mode);
      if (result == 0 && mode.empty()) break;
    }
    if (result != 0 || !mode.empty()) {
      RCLCPP_ERROR(node->get_logger(), "GO2 did not enter released low-level mode");
      rclcpp::shutdown();
      return 7;
    }
    RCLCPP_INFO(node->get_logger(), "GO2 high-level controller released; low-level control is ready");
  }
  if (action == "select") {
    if (!confirm_action) {
      RCLCPP_ERROR(
        node->get_logger(), "Refusing to select GO2 mode without confirm_action:=true");
      rclcpp::shutdown();
      return 8;
    }
    result = motion.SelectMode(select_mode);
    if (result != 0) {
      RCLCPP_ERROR(
        node->get_logger(), "GO2 SelectMode(%s) failed with code %d", select_mode.c_str(), result);
      rclcpp::shutdown();
      return 9;
    }
    RCLCPP_INFO(
      node->get_logger(), "GO2 selected high-level mode '%s'; the remote controller is available again",
      select_mode.c_str());
  }
  rclcpp::shutdown();
  return 0;
}
