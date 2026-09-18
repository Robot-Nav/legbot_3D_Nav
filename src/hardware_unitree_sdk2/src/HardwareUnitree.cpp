//
// Created by biao on 24-9-9.
//

#include "hardware_unitree_sdk2/HardwareUnitree.h"
#include "crc32.h"
#include <cmath>
#include <algorithm>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;
using hardware_interface::return_type;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitree::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    joint_torque_command_.assign(12, 0);
    joint_position_command_.assign(12, 0);
    joint_velocities_command_.assign(12, 0);
    joint_kp_command_.assign(12, 0);
    joint_kd_command_.assign(12, 0);

    joint_position_.assign(12, 0);
    joint_velocities_.assign(12, 0);
    joint_effort_.assign(12, 0);

    imu_states_.assign(10, 0);
    foot_force_.assign(4, 0);
    high_states_.assign(6, 0);

    for (const auto& joint : info_.joints)
    {
        for (const auto& interface : joint.state_interfaces)
        {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }


    if (const auto network_interface_param = info.hardware_parameters.find("network_interface"); network_interface_param
        != info.hardware_parameters.end())
    {
        network_interface_ = network_interface_param->second;
    }
    if (const auto domain_param = info.hardware_parameters.find("domain"); domain_param != info.hardware_parameters.
        end())
    {
        domain_ = std::stoi(domain_param->second);
    }
    if (const auto show_foot_force_param = info.hardware_parameters.find("show_foot_force"); show_foot_force_param !=
        info.hardware_parameters.end())
    {
        show_foot_force_ = show_foot_force_param->second == "true";
    }

    const auto command_enable = info.hardware_parameters.find("enable_commands");
    enable_commands_ = command_enable != info.hardware_parameters.end() && command_enable->second == "true";
    if (network_interface_.empty() || network_interface_ == "lo" || domain_ != 0) {
        RCLCPP_ERROR(get_logger(), "GO2 EDU requires an explicit Ethernet interface and SDK domain 0");
        return CallbackReturn::ERROR;
    }
    const std::vector<std::string> expected = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint",
        "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    if (info.joints.size() != expected.size()) return CallbackReturn::ERROR;
    for (size_t i=0; i<expected.size(); ++i)
        if (info.joints[i].name != expected[i]) return CallbackReturn::ERROR;
    RCLCPP_INFO(get_logger(), " network_interface: %s, domain: %d", network_interface_.c_str(), domain_);
    ChannelFactory::Instance()->Init(domain_, network_interface_);

    if (enable_commands_) {
        unitree::robot::b2::MotionSwitcherClient motion;
        motion.SetTimeout(5.0f); motion.Init();
        std::string form, mode;
        if (motion.CheckMode(form, mode) != 0 || !mode.empty()) {
            RCLCPP_ERROR(get_logger(), "GO2 must be in low-level mode before enabling commands; release sport mode explicitly first");
            return CallbackReturn::ERROR;
        }
    }
    low_cmd_publisher_ =
        std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(
            TOPIC_LOWCMD);
    low_cmd_publisher_->InitChannel();

    lows_tate_subscriber_ =
        std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(
            TOPIC_LOWSTATE);
    lows_tate_subscriber_->InitChannel(
        [this](auto&& PH1)
        {
            lowStateMessageHandle(std::forward<decltype(PH1)>(PH1));
        },
        1);
    initLowCmd();

    high_state_subscriber_ =
        std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
            TOPIC_HIGHSTATE);
    high_state_subscriber_->InitChannel(
        [this](auto&& PH1)
        {
            highStateMessageHandle(std::forward<decltype(PH1)>(PH1));
        },
        1);


    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HardwareUnitree::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
    }

    // export imu sensor state interface
    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
        state_interfaces.emplace_back(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]);
    }

    // export foot force sensor state interface
    if (info_.sensors.size() > 1)
    {
        for (uint i = 0; i < info_.sensors[1].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &foot_force_[i]);
        }
    }

    // export odometer state interface
    if (info_.sensors.size() > 2)
    {
        // export high state interface
        for (uint i = 0; i < info_.sensors[2].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[2].name, info_.sensors[2].state_interfaces[i].name, &high_states_[i]);
        }
    }


    return
        state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareUnitree::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
        ind++;
    }
    return command_interfaces;
}

return_type HardwareUnitree::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    // joint states
    for (int i(0); i < 12; ++i)
    {
        joint_position_[i] = low_state_.motor_state()[i].q();
        joint_velocities_[i] = low_state_.motor_state()[i].dq();
        joint_effort_[i] = low_state_.motor_state()[i].tau_est();
    }

    // imu states
    imu_states_[0] = low_state_.imu_state().quaternion()[0]; // w
    imu_states_[1] = low_state_.imu_state().quaternion()[1]; // x
    imu_states_[2] = low_state_.imu_state().quaternion()[2]; // y
    imu_states_[3] = low_state_.imu_state().quaternion()[3]; // z
    imu_states_[4] = low_state_.imu_state().gyroscope()[0];
    imu_states_[5] = low_state_.imu_state().gyroscope()[1];
    imu_states_[6] = low_state_.imu_state().gyroscope()[2];
    imu_states_[7] = low_state_.imu_state().accelerometer()[0];
    imu_states_[8] = low_state_.imu_state().accelerometer()[1];
    imu_states_[9] = low_state_.imu_state().accelerometer()[2];

    // contact states
    foot_force_[0] = low_state_.foot_force()[0];
    foot_force_[1] = low_state_.foot_force()[1];
    foot_force_[2] = low_state_.foot_force()[2];
    foot_force_[3] = low_state_.foot_force()[3];

    if (show_foot_force_)
    {
        RCLCPP_INFO(get_logger(), "foot_force(): %f, %f, %f, %f", foot_force_[0], foot_force_[1], foot_force_[2],
                    foot_force_[3]);
    }

    // high states
    high_states_[0] = high_state_.position()[0];
    high_states_[1] = high_state_.position()[1];
    high_states_[2] = high_state_.position()[2];
    high_states_[3] = high_state_.velocity()[0];
    high_states_[4] = high_state_.velocity()[1];
    high_states_[5] = high_state_.velocity()[2];

    // RCLCPP_INFO(get_logger(), "high state: %f %f %f %f %f %f", high_states_[0], high_states_[1], high_states_[2],
    //             high_states_[3], high_states_[4], high_states_[5]);

    return return_type::OK;
}

return_type HardwareUnitree::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!enable_commands_) return return_type::OK;
    if (std::chrono::steady_clock::now() - received_ > std::chrono::milliseconds(100))
        return return_type::ERROR;
    for (int i=0; i<12; ++i) {
        if (!std::isfinite(joint_position_command_[i]) || !std::isfinite(joint_velocities_command_[i]) ||
            !std::isfinite(joint_kp_command_[i]) || !std::isfinite(joint_kd_command_[i]) ||
            !std::isfinite(joint_torque_command_[i])) return return_type::ERROR;
    }
    for (int i=0; i<12; ++i) {
        const auto it = info_.limits.find(info_.joints[i].name);
        if (it == info_.limits.end() || !it->second.has_effort_limits) return return_type::ERROR;
        const auto &limit = it->second;
        if (limit.has_position_limits)
            joint_position_command_[i] = std::clamp(joint_position_command_[i], limit.min_position, limit.max_position);
        const double kp = joint_kp_command_[i];
        const double kd = joint_kd_command_[i];
        if (kp < 0 || kd < 0) return return_type::ERROR;
        const double tau = kp * (joint_position_command_[i]-low_state_.motor_state()[i].q()) +
            kd * (joint_velocities_command_[i]-low_state_.motor_state()[i].dq()) + joint_torque_command_[i];
        const double scale = std::abs(tau) > limit.max_effort ? limit.max_effort/std::abs(tau) : 1.0;
        joint_kp_command_[i] *= scale; joint_kd_command_[i] *= scale; joint_torque_command_[i] *= scale;
    }
    // send command
    for (int i(0); i < 12; ++i)
    {
        low_cmd_.motor_cmd()[i].mode() = 0x01;
        low_cmd_.motor_cmd()[i].q() = static_cast<float>(joint_position_command_[i]);
        low_cmd_.motor_cmd()[i].dq() = static_cast<float>(joint_velocities_command_[i]);
        low_cmd_.motor_cmd()[i].kp() = static_cast<float>(joint_kp_command_[i]);
        low_cmd_.motor_cmd()[i].kd() = static_cast<float>(joint_kd_command_[i]);
        low_cmd_.motor_cmd()[i].tau() = static_cast<float>(joint_torque_command_[i]);
    }

    low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_),
                                (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    low_cmd_publisher_->Write(low_cmd_);
    return return_type::OK;
}

void HardwareUnitree::initLowCmd()
{
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd_.motor_cmd()[i].mode() =
            0x01; // motor switch to servo (PMSM) mode
        low_cmd_.motor_cmd()[i].q() = 0;
        low_cmd_.motor_cmd()[i].kp() = 0;
        low_cmd_.motor_cmd()[i].dq() = 0;
        low_cmd_.motor_cmd()[i].kd() = 0;
        low_cmd_.motor_cmd()[i].tau() = 0;
    }
}

void HardwareUnitree::lowStateMessageHandle(const void* messages)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    received_ = std::chrono::steady_clock::now();
    low_state_ = *static_cast<const unitree_go::msg::dds_::LowState_*>(messages);
}

void HardwareUnitree::highStateMessageHandle(const void* messages)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    high_state_ = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(messages);
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    HardwareUnitree, hardware_interface::SystemInterface)
