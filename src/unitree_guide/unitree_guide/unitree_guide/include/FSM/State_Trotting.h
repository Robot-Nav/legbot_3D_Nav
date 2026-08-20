/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef TROTTING_H
#define TROTTING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include <torch/torch.h>
#include <fstream>

class State_Trotting : public FSMState{
public:
    State_Trotting(CtrlComponents *ctrlComp);
    ~State_Trotting();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx, double vy, double wz);

    void refresh_amp_obs();
    void save_amp_obs_thread();
    void open_amp_save_file();
    void close_amp_save_file();
    torch::Tensor quat_rotate_inverse(const torch::Tensor& q, const torch::Tensor& v);
    std::ofstream outfile;
private:
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    void calcBalanceKp();
    bool checkStepOrNot();

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

    // Rob State
    Vec3  _posBody, _velBody;
    double _yaw, _dYaw;
    Vec34 _posFeetGlobal, _velFeetGlobal;
    Vec34 _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec12 _q;

    // Robot command
    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    double _yawCmd, _dYawCmd;
    double _dYawCmdPast;
    Vec3 _wCmdGlobal;
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34 _posFeet2BGoal, _velFeet2BGoal;
    RotMat _Rd;
    Vec3 _ddPcd, _dWbd;
    Vec34 _forceFeetGlobal, _forceFeetBody;
    Vec34 _qGoal, _qdGoal;
    Vec12 _tau;

    // Control Parameters
    double _gaitHeight;
    Vec3 _posError, _velError;
    Mat3 _Kpp, _Kdp, _Kdw;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;
    Vec2 _vxLim, _vyLim, _wyawLim;
    Vec4 *_phase;
    VecInt4 *_contact;

    // Calculate average value
    AvgCov *_avg_posError = new AvgCov(3, "_posError", true, 1000, 1000, 1);
    AvgCov *_avg_angError = new AvgCov(3, "_angError", true, 1000, 1000, 1000);

    enum THREAD {
        STOP,
        RUNNING,
        OVER,
    };
    int _last_cmd = 0;
    float _startPos[12];
    uint32_t _duration = 4.3e6;   //us

    int reindex[12] = {3,4,5,0,1,2,9,10,11,6,7,8};
    std::array<float, 3> base_w_linear_vel = {0.0, 0.0, 0.0};
    std::array<float, 3> base_w_angular_vel = {0.0, 0.0, 0.0};

    std::array<float, 3> base_w_pos = {0.0, 0.0, 0.0};
    std::array<float, 4> base_w_orientation = {1.0, 0.0, 0.0, 0.0};
    std::array<float, 12> joint_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 12> foot_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 3> base_linear_vel = {0.0, 0.0, 0.0};
    std::array<float, 3> base_angular_vel = {0.0, 0.0, 0.0};
    std::array<float, 12> joint_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 12> foot_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<float> motion_data;
    uint32_t dofPosSwitBeginTime = 0.0;//关节位置切换开始时间
    float motion_time = 0.0;
    std::thread* amp_obs_thread;
    uint8_t ampthreadRunning=false;
    float amp_duration = 0.020;
};

#endif  // TROTTING_H