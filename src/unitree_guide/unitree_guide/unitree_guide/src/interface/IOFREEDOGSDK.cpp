/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "interface/IOFREEDOGSDK.h"

IOFREEDOGSDK::IOFREEDOGSDK()
{
 udp_ = std::make_shared<FDSC::UnitreeConnection>("LOW_WIRED_DEFAULTS");
 udp_->startRecv();
 std::vector<uint8_t> cmdBytes = low_cmd.buildCmd(false);
 udp_->send(cmdBytes);
 std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

IOFREEDOGSDK::~IOFREEDOGSDK(){

}

void IOFREEDOGSDK::recvState() {
    std::vector<std::vector<uint8_t>> dataAll;
    udp_->getData(dataAll);
    if (!dataAll.empty()) {
        std::vector<uint8_t> data = dataAll.at(dataAll.size() - 1);
        low_state.parseData(data);
    }
    // std::cout << "Motor q";
    for(int i(0); i < 12; ++i){
        // std::cout << low_state.motorState_free_dog[i].q << " ";
    }
    // std::cout << std::endl;

    // std::cout << "Motor dq";
    for(int i(0); i < 12; ++i){
        // std::cout << low_state.motorState_free_dog[i].dq << " ";
    }
    // std::cout << std::endl;

    // std::cout << "quaternion";
    for(int i(0); i < 4; ++i){
        // std::cout << low_state.imu_quaternion[i] << " ";
    }
    // std::cout << std::endl;

    // std::cout << "gyroscope";
    for(int i(0); i < 3; ++i){
        // std::cout << low_state.imu_gyroscope[i] << " ";
    }
    // std::cout << std::endl;
    // std::cout << std::endl;
    // state->imu.quaternion[3] = lowState_.imu_quaternion[3];
}

void IOFREEDOGSDK::setCmd(int joint_i,std::vector<double> joint) {
    cmdArr.setMotorCmd(joint_i, FDSC::MotorModeLow::Servo, joint);
}

void IOFREEDOGSDK::sendCmd() {
    low_cmd.motorCmd_free_dog = cmdArr;
    std::vector<uint8_t> cmdBytes = low_cmd.buildCmd(false);
    udp_->send(cmdBytes);
}

void IOFREEDOGSDK::sendRecv() {
  recvState();
  sendCmd();
}

RotMat IOFREEDOGSDK::getRotMat() {
    Quat quat;
    quat << low_state.imu_quaternion[0], low_state.imu_quaternion[1], low_state.imu_quaternion[2], low_state.imu_quaternion[3];
    return quatToRotMat(quat);
}
