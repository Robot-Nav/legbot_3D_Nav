/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>
#include "ros/ros.h"
#include <vector>

class IOInterface{
public:
IOInterface(){}
~IOInterface(){delete cmdPanel;}
virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
void zeroCmdPanel(){cmdPanel->setZero();}
void setPassive(){cmdPanel->setPassive();}
std::array<double, 3> _base_w_pos = {0.0, 0.0, 0.0};
std::array<double, 4> _base_w_ori = {0.0, 0.0, 0.0, 0.1};
std::array<double, 3> _base_w_linear_vel = {0.0, 0.0, 0.0};
std::array<double, 3> _base_w_angular_vel = {0.0, 0.0, 0.0};
std::array<double, 3> _base_t_pos = {0.0, 0.0, 0.0};
std::array<double, 4> _base_t_ori = {0.0, 0.0, 0.0, 0.1};
std::array<double, 3> _base_t_linear_vel = {0.0, 0.0, 0.0};
std::array<double, 3> _base_t_angular_vel = {0.0, 0.0, 0.0};
std::array<double, 3> _FL_foot_pos = {0.0, 0.0, 0.0};
std::array<double, 3> _FL_foot_vel = {0.0, 0.0, 0.0};
std::array<double, 3> _FR_foot_pos = {0.0, 0.0, 0.0};
std::array<double, 3> _FR_foot_vel = {0.0, 0.0, 0.0};
std::array<double, 3> _RL_foot_pos = {0.0, 0.0, 0.0};
std::array<double, 3> _RL_foot_vel = {0.0, 0.0, 0.0};
std::array<double, 3> _RR_foot_pos = {0.0, 0.0, 0.0};
std::array<double, 3> _RR_foot_vel = {0.0, 0.0, 0.0};
std::vector<float> axes;
std::vector<int> buttons;
uint32_t current_time = 0;
protected:
CmdPanel *cmdPanel;
};

#endif  //IOINTERFACE_H