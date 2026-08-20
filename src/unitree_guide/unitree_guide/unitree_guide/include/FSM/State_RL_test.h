 /**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef STATE_RL_TEST_H
#define STATE_RL_TEST_H

#include "FSM/FSMState.h"
#include <fstream>  // 包含文件流的头文件
#include <thread>
#include <string>
#include <torch/torch.h>
#include <torch/script.h>
#define HISTORY_LEN 5
class State_RL : public FSMState{

public:
    State_RL(CtrlComponents *ctrlComp);
    ~State_RL(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
    void refresh_rl_obs();
    void refresh_rl_obs_real_robot();
    void refresh_amp_obs();
    void infer_thread_callback();
    void save_amp_obs_thread();
    void open_amp_save_file();
    void close_amp_save_file();
    void load_policy();
    void printSegments(const torch::Tensor& tensor, int segment_size, const std::vector<int>& sub_sizes);
    void printTensorHorizontal(const torch::Tensor& tensor, const std::string& name);
    torch::Tensor quat_rotate_inverse(const torch::Tensor& q, const torch::Tensor& v);
    std::ofstream outfile;
private:
    int debug = false;
    at::string model_path;
    torch::DeviceType device;
    torch::jit::script::Module model;
    enum Color {
        STOP,
        RUNNING,
        OVER,
    };
    int _cnt = 0;
    int _last_cmd = 0;
    float _startPos[12];
    uint32_t _duration = 1e6;   //us
    float _percent = 0;       //%
    float _targetPos_map[5][12] = {{0.0,0.9,-1.8,0.0,0.9,-1.8,0.0,0.9,-1.8,0.0,0.9,-1.8},
                                    {0.5,0.9,-1.8,0.2,0.5,-1.5,0.5,0.9,-1.8,0.2,0.5,-1.5},
                                    {-0.2,0.5,-1.5,-0.5,0.9,-1.8,-0.2,0.5,-1.5,-0.5,0.9,-1.8},
                                    {0.0,0.6,-1.5,0.0,0.6,-1.5,0.0,0.9,-1.8,0.0,0.9,-1.8},
                                    {0.0,0.9,-1.8,0.0,0.9,-1.8,0.0,0.6,-1.5,0.0,0.6,-1.5}
    };
    std::string angle_names[5] = {"stand","right","left","back","front"};
    int reindex[12] = {3,4,5,0,1,2,9,10,11,6,7,8};
    int reindex_orien[4] = {4,1,2,3};
    RotMat _B2G_RotMat, _G2B_RotMat;
    std::array<float, 3> command = {0.0, 0.0, 0.0};
    std::array<float, 12> last_actions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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
    Vec3 gravity = {0.0, 0.0, -1.0};
    // auto opts = torch::TensorOptions();
    torch::Tensor gravity_tensor = torch::tensor({0.0f, 0.0f, -1.0f});
    torch::Tensor base_ang_vel_tensor = torch::zeros({3});  // 角速度
    torch::Tensor projected_gravity_tensor = torch::zeros({3});  // 投影重力
    torch::Tensor commands_tensor = torch::zeros({3});  // 控制命令
    torch::Tensor dof_pos_tensor = torch::zeros({12});  // 自由度位置
    torch::Tensor default_dof_pos_tensor = torch::tensor({-0.15f, 0.55f, -1.5f,0.15f, 0.55f, -1.5f,-0.15f, 0.7f, -1.5f,0.15f, 0.7f, -1.5f});  // 默认自由度位置
    torch::Tensor dof_vel_tensor = torch::zeros({12});  // 自由度速度
    torch::Tensor actions_tensor = torch::zeros({12});  // 动作
    torch::Tensor actions_tensor_scaled = torch::zeros({12});  // 缩放后的动作
    torch::Tensor obs_scales_lin_vel = torch::tensor({2.0f, 2.0f, 2.0f});  // 线性速度缩放
    torch::Tensor obs_scales_ang_vel = torch::tensor({0.25f, 0.25f, 0.25f});  // 角速度缩放
    torch::Tensor obs_scales_dof_pos = torch::ones({12});  // 自由度位置缩放
    torch::Tensor obs_scales_dof_vel = torch::ones({12})* 0.05;  // 自由度速度缩放
    torch::Tensor commands_scale = torch::tensor({2.0f, 2.0f, 0.25f});  // 命令缩放
    torch::Tensor obs_tensor = torch::zeros({45});
    torch::Tensor obs_history_tensor = torch::zeros({HISTORY_LEN, 45});
    long long dofPosSwitBeginTime = 0.0;//关节位置切换开始时间
    float motion_time = 0.0;
    std::thread* infer_thread = nullptr;
    std::thread* amp_obs_thread = nullptr;
    uint8_t infer_thread_runnning=false;
    uint8_t ampthreadRunning=false;
    float infer_duration = 0.02;
    float amp_duration = 0.005;
};

#endif
