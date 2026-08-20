/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOFREEDOGSDK_H
#define IOFREEDOGSDK_H

#include "ros/ros.h"
#include "free_dog_sdk_cpp/common.hpp"
#include "free_dog_sdk_cpp/complex.hpp"
#include "free_dog_sdk_cpp/lowCmd.hpp"
#include "free_dog_sdk_cpp/lowState.hpp"
#include "free_dog_sdk_cpp/unitreeConnectBoost.hpp"
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct UnitreeMotorData {
 double pos_, vel_, tau_;                 // state
 double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct UnitreeImuData {
 double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
 double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
 double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
 double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
 double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
 double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class IOFREEDOGSDK{
public:
 IOFREEDOGSDK();
 ~IOFREEDOGSDK();

 void sendRecv();
 void setCmd(int joint_i,std::vector<double> joint);
 void sendCmd();
 void recvState();
 RotMat getRotMat();

 std::shared_ptr<FDSC::UnitreeConnection> udp_;
 FDSC::lowCmd low_cmd;
 FDSC::lowState low_state;
 FDSC::MotorCmdArray cmdArr;
 UnitreeMotorData jointData_[12]{};  // NOLINT(modernize-avoid-c-arrays)
 UnitreeImuData imuData_{};
 bool contactState_[4]{};  // NOLINT(modernize-avoid-c-arrays)

 int powerLimit_{};
 int contactThreshold_{};
 float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                              -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};
};


#endif  // IOSDK_H