/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <cstdlib>
#include <ros/package.h>
#include "FSM/State_RL_test.h"

State_RL::State_RL(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::RL, "RL")
{
    load_policy();
    gravity(0,0) = 0.0;
    gravity(1,0) = 0.0;
    gravity(2,0) = -0.98;
    //在构造函数中初始化，订阅
    this->Sub_=nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1000,boost::bind(&FSMState::cmdVelCallback,this,_1));

}


void State_RL::enter(){
     // if (real == false){
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
            _startPos[i] = _lowState->motorState[i].q;
            _lowCmd->motorCmd[i].mode = 10;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 80;
            _lowCmd->motorCmd[i].Kd = 1;
            _lowCmd->motorCmd[i].tau = 0;
        }
        for(int i=0; i<4; i++){
             if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
                 _lowCmd->setSimStanceGain(i);
             }
             else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
                 _lowCmd->setRealStanceGain(i);
             }
             _lowCmd->setZeroDq(i);
             _lowCmd->setZeroTau(i);
        }
    // }
    // else if(real == true)
    // {
        for(int i=0; i<12; i++){
            float c_joint = _ctrlComp->ioInterFreeDog->low_state.motorState_free_dog[i].q;
            std::vector<double> joint{c_joint, 0, 0, 80, 1};
            _ctrlComp->ioInterFreeDog->setCmd(i,joint);
        }
    // }
    for (int i = 0; i < HISTORY_LEN; i++)
    {
        refresh_rl_obs();
    }
    infer_thread = new std::thread(&State_RL::infer_thread_callback,this);
    infer_thread_runnning = State_RL::RUNNING;
    if (debug == true){
        amp_obs_thread = new std::thread(&State_RL::save_amp_obs_thread,this);
        ampthreadRunning = State_RL::RUNNING;
    }
}

void State_RL::run(){
}

void State_RL::exit(){
    _percent = 0;
    if (debug && amp_obs_thread != nullptr) {
        ampthreadRunning = State_RL::STOP;
        if (amp_obs_thread->joinable()) {
            amp_obs_thread->join();
        }
        delete amp_obs_thread;
        amp_obs_thread = nullptr;
    }
    infer_thread_runnning = State_RL::STOP;
    if (infer_thread != nullptr && infer_thread->joinable()) {
        infer_thread->join();
    }
    delete infer_thread;
    infer_thread = nullptr;
    std::cout << "amp_obs_thread退出!" << std::endl;
    if (outfile.is_open()) {
        outfile.close();
        std::cout << "文件关闭成功!" << std::endl;
    }
}

FSMStateName State_RL::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        if (_last_cmd==static_cast<int>(UserCommand::RL))
        {
            _cnt = (_cnt+1)%(sizeof(_targetPos_map) / sizeof(_targetPos_map[0]));
            if (real == false){
                for(int i=0; i<12; i++){
                    _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
                    _startPos[i] = _lowState->motorState[i].q;
                }
            }
            else if(real == true){
                for(int i=0; i<12; i++){
                    _startPos[i] = _ctrlComp->ioInterFreeDog->low_state.motorState_free_dog[i].q;
                }
            }
            _percent = 0;
            std::cout << "cnt: " << _cnt << std::endl;
            // open_amp_save_file();
            dofPosSwitBeginTime = getTime();
        }
        _last_cmd = static_cast<int>(_lowState->userCmd);
        return FSMStateName::RL;
    }
    else{
        _last_cmd = static_cast<int>(_lowState->userCmd);
        return FSMStateName::RL;
    }
}

void State_RL::infer_thread_callback()
{
    while(infer_thread_runnning == State_RL::RUNNING)
    {
        long long _start_time = getTime();
        // std::cout << "_start_time" << _start_time << std::endl;
        refresh_rl_obs();
        torch::Tensor flattened_obs = obs_history_tensor.view({1, HISTORY_LEN * 45});
        if (debug == true)
        {
            const std::vector<int> sub_sizes = {3, 3, 3, 12, 12, 12};
            int segment_size = 45;
            // std::cout << "printSegments" << std::endl;
            // printSegments(flattened_obs.squeeze(), segment_size, sub_sizes);
        }
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(flattened_obs);
        // std::cout << "flattened_obs: " << flattened_obs << std::endl;
        actions_tensor = model.get_method("act_inference")(inputs).toTensor().to(torch::kCPU).squeeze();
        if (debug==true){
            torch::Tensor input_tensor = torch::arange(1, 226).view({1, 225}).to(torch::kFloat32).to(device); // 注意范围是 [start, end)
            std::vector<torch::jit::IValue> test;
            test.push_back(input_tensor);
            torch::Tensor output = model.get_method("act_inference")(test).toTensor().to(torch::kCPU).squeeze();
            // printTensorHorizontal(output, "same_net_work_test");
        }
        actions_tensor_scaled = actions_tensor.clone() * 0.25;
        std::vector<float> actions(actions_tensor_scaled.data_ptr<float>(),
                           actions_tensor_scaled.data_ptr<float>() + actions_tensor_scaled.numel());
        if (debug == true) std::cout << "actions[reindex[j]]  + default_dof_pos" << std::endl;
        for(int i=0; i<12; i++){
            if (real == false)
            {
                _lowCmd->motorCmd[i].q = actions[reindex[i]]  + default_dof_pos_tensor[reindex[i]].item<float>();
                _lowCmd->motorCmd[i].Kp = 80;
                _lowCmd->motorCmd[i].Kd = 1;
            }
            else if (real == true)
            {
                // float t_joint = actions[reindex[i]]  + default_dof_pos_tensor[reindex[i]].item<float>();
                // std::vector<double> joint{t_joint, 0, 0, 80, 1};
                // _ctrlComp->ioInterFreeDog->setCmd(i,joint);
            }
            if (debug == true) std::cout << actions[reindex[i]]  + default_dof_pos_tensor[reindex[i]].item<float>() << " ";
        }
        if (debug == true)
            std::cout << std::endl;
        // std::cout << "actions_tensor: " << actions_tensor << std::endl;
        wait(_start_time, (long long)(infer_duration * 1000000));
    }
    infer_thread_runnning = State_RL::OVER;
}

void State_RL::save_amp_obs_thread()
{
    while(ampthreadRunning == State_RL::RUNNING)
    {
        long long _start_time = getTime();
        if ((getTime() - dofPosSwitBeginTime)<_duration) {
            _percent = (float)(getTime() - dofPosSwitBeginTime)/_duration;
            _percent = _percent > 1 ? 1 : _percent;
            std::cout << "_percent" << _percent << std::endl;
            // if (real == false){
                std::cout << "_lowCmd->motorCmd ";
                for(int j=0; j<12; j++){
                    std::cout << _targetPos_map[_cnt][reindex[j]] << " ";
                    _lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos_map[_cnt][reindex[j]];
                }
                std::cout << _lowCmd->motorCmd << std::endl;
                std::cout << std::endl;
            // }
            // else if (real == true){
                std::cout << "target_joint";
                for(int j=0; j<12; j++){
                    std::cout << _targetPos_map[_cnt][j] << " ";
                    float t_joint = (1 - _percent)*_startPos[j] + _percent*_targetPos_map[_cnt][reindex[j]];
                    std::vector<double> joint{t_joint, 0, 0, 80, 1};
                    _ctrlComp->ioInterFreeDog->setCmd(j,joint);
                }
                std::cout << std::endl;
            // }
            if ((float)(getTime() - dofPosSwitBeginTime)>(float)_duration*0.95)
                close_amp_save_file();
        }
        if (outfile.is_open())
        {
            std::cout << "save data" << std::endl;
            refresh_amp_obs();
        }
        wait(_start_time, (long long)(infer_duration * 1000000));
    }
    ampthreadRunning = State_RL::OVER;
}



void State_RL::refresh_rl_obs(){
    auto opts = torch::TensorOptions().dtype(torch::kFloat32);
    //gazebo simulation mode
    if (real == false)
    {
        for (int i=0; i<4; i++) {
            base_w_orientation[i] = _ctrlComp->ioInter->_base_w_ori[i];
        }
        for (int i=0; i<3; i++) {
            base_w_angular_vel[i] = _ctrlComp->ioInter->_base_w_angular_vel[i];
        }
        torch::Tensor orientation_tensor = torch::from_blob(base_w_orientation.data(), {int64_t(base_w_orientation.size())}, opts).unsqueeze(0).clone();
        torch::Tensor w_angular_vel_tensor = torch::from_blob(base_w_angular_vel.data(), {int64_t(base_w_angular_vel.size())}, opts).unsqueeze(0).clone();
        base_ang_vel_tensor = quat_rotate_inverse(orientation_tensor, w_angular_vel_tensor).squeeze().clone();
        projected_gravity_tensor = quat_rotate_inverse(orientation_tensor, gravity_tensor.unsqueeze(0)).squeeze().clone();
        
        //订阅cmd_vel
        // this->Sub_=nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1000,boost::bind(&FSMState::cmdVelCallback,this,_1));

        // commands_tensor[0] = _ctrlComp->ioInter->axes[1];
        // commands_tensor[1] = _ctrlComp->ioInter->axes[0];
        // commands_tensor[2] = _ctrlComp->ioInter->axes[3]*3.14;

        commands_tensor[0] = this->current_cmd_vel_.linear_x;
        commands_tensor[1] = this->current_cmd_vel_.linear_y;
        commands_tensor[2] = this->current_cmd_vel_.angular_z;


        // std::cout << _ctrlComp->ioInter->axes << std::endl;
        // std::cout << "commands_tensor: " << commands_tensor << std::endl;
        for(int i=0; i<12; i++){
            joint_pos[i] = _lowState->motorState[reindex[i]].q;
        }
        dof_pos_tensor = torch::from_blob(joint_pos.data(), {int64_t(joint_pos.size())}, opts).clone();
        // printTensorHorizontal(dof_pos_tensor,"dof_pos_tensor");
        for(int i=0; i<12; i++){
            joint_vel[i] = _lowState->motorState[reindex[i]].dq;
        }
        dof_vel_tensor = torch::from_blob(joint_vel.data(), {int64_t(joint_vel.size())}, opts).clone();
        obs_tensor = torch::cat({
            base_ang_vel_tensor * obs_scales_ang_vel,
            projected_gravity_tensor,
            commands_tensor * commands_scale,
            (dof_pos_tensor - default_dof_pos_tensor) * obs_scales_dof_pos,
            dof_vel_tensor * obs_scales_dof_vel,
            actions_tensor
        }, -1).to(device);
        obs_history_tensor = torch::cat({
            obs_history_tensor.slice(0, 1, HISTORY_LEN).to(device),  // 删除最早的一步
            obs_tensor.unsqueeze(0)  // 将当前 obs_tensor 插入到历史中
        }, 0);  // 按行（第0维）拼接
    }
    else if (real == true)
    {
        _B2G_RotMat = _ctrlComp->ioInterFreeDog->getRotMat();
        _G2B_RotMat = _B2G_RotMat.transpose();
        Vec3 projected_gravity = _G2B_RotMat*gravity;
        projected_gravity_tensor = torch::tensor({projected_gravity(0,0), projected_gravity(1,0), projected_gravity(2,0)});
         for (int i=0; i<3; i++) {
            base_ang_vel_tensor[i] = _ctrlComp->ioInterFreeDog->low_state.imu_gyroscope[i];
        }
        commands_tensor[0] = _ctrlComp->ioInter->axes[1];
        commands_tensor[1] = _ctrlComp->ioInter->axes[0];
        commands_tensor[2] = _ctrlComp->ioInter->axes[3]*3.14;
        for(int i=0; i<12; i++){
            joint_pos[i] = _ctrlComp->ioInterFreeDog->low_state.motorState_free_dog[reindex[i]].q;
        }
        dof_pos_tensor = torch::from_blob(joint_pos.data(), {int64_t(joint_pos.size())}, opts).clone();
        for(int i=0; i<12; i++){
            joint_vel[i] = _ctrlComp->ioInterFreeDog->low_state.motorState_free_dog[reindex[i]].dq;
        }
        dof_vel_tensor = torch::from_blob(joint_vel.data(), {int64_t(joint_vel.size())}, opts).clone();
        obs_tensor = torch::cat({
            base_ang_vel_tensor * obs_scales_ang_vel,
            projected_gravity_tensor,
            commands_tensor * commands_scale,
            (dof_pos_tensor - default_dof_pos_tensor) * obs_scales_dof_pos,
            dof_vel_tensor * obs_scales_dof_vel,
            actions_tensor
        }, -1).to(device);
        obs_history_tensor = torch::cat({
            obs_history_tensor.slice(0, 1, HISTORY_LEN).to(device),  // 删除最早的一步
            obs_tensor.unsqueeze(0)  // 将当前 obs_tensor 插入到历史中
        }, 0);  // 按行（第0维）拼接
    }
}


void State_RL::refresh_rl_obs_real_robot(){

}

void State_RL::refresh_amp_obs(){
    auto opts = torch::TensorOptions().dtype(torch::kFloat32);
    motion_time = static_cast<float>(getRosTime() - dofPosSwitBeginTime)/1e6;
    outfile << "motion_time: " << motion_time << std::endl;
    outfile << "base_w_pos: ";
    for (int i=0; i<3; i++) {
        base_w_pos[i] = _ctrlComp->ioInter->_base_w_pos[i];
        outfile << base_w_pos[i] << " ";
    }
    outfile << std::endl;

    outfile << "base_ori: ";
    for (int i=0; i<4; i++) {
        base_w_orientation[i] = _ctrlComp->ioInter->_base_w_ori[i];
        outfile << base_w_orientation[i] << " ";
    }
    outfile << std::endl;

    outfile << "dof_pos: ";
    for(int i=0; i<12; i++){
        joint_pos[i] = _lowState->motorState[reindex[i]].q;
        outfile << joint_pos[i] << " ";
    }
    outfile << std::endl;

    outfile << "foot_pos: ";
    for (int i=0; i<3; i++){
        foot_pos[0*3+i] = _ctrlComp->ioInter->_FL_foot_pos[i];
        foot_vel[0*3+i] = _ctrlComp->ioInter->_FL_foot_vel[i];
    }
    for (int i=0; i<3; i++){
        foot_pos[1*3+i] = _ctrlComp->ioInter->_FR_foot_pos[i];
        foot_vel[1*3+i] = _ctrlComp->ioInter->_FR_foot_vel[i];
    }
    for (int i=0; i<3; i++){
        foot_pos[2*3+i] = _ctrlComp->ioInter->_RL_foot_pos[i];
        foot_vel[2*3+i] = _ctrlComp->ioInter->_RL_foot_vel[i];
    }
    for (int i=0; i<3; i++){
        foot_pos[3*3+i] = _ctrlComp->ioInter->_RR_foot_pos[i];
        foot_vel[3*3+i] = _ctrlComp->ioInter->_RR_foot_vel[i];
    }
    for (  int  i=0;i<12;i++ )
    {
        outfile << foot_pos[i] << " ";
    }
    outfile << std::endl;

    outfile << "base_w_linear_vel: ";
    for (int i=0; i<3; i++) {
        base_w_linear_vel[i] = _ctrlComp->ioInter->_base_w_linear_vel[i];
    }
    torch::Tensor orientation_tensor = torch::from_blob(base_w_orientation.data(), {int64_t(base_w_orientation.size())}, opts).unsqueeze(0).clone();
    torch::Tensor w_linear_vel_tensor = torch::from_blob(base_w_linear_vel.data(), {int64_t(base_w_linear_vel.size())}, opts).unsqueeze(0).clone();
    torch::Tensor result = quat_rotate_inverse(orientation_tensor, w_linear_vel_tensor).squeeze().clone();
    for (int i = 0; i < 3; ++i) {
        base_linear_vel[i] = result[i].item<float>();
        // std::cout << base_linear_vel[i] << " ";
        outfile << base_linear_vel[i] << " ";
    }
    outfile << std::endl;

    outfile << "base_w_angular_vel: ";
    for (int i=0; i<3; i++) {
        base_w_angular_vel[i] = _ctrlComp->ioInter->_base_w_angular_vel[i];
    }
    torch::Tensor w_angular_vel_tensor = torch::from_blob(base_w_angular_vel.data(), {int64_t(base_w_angular_vel.size())}, opts).unsqueeze(0).clone();
    result = quat_rotate_inverse(orientation_tensor, w_angular_vel_tensor).squeeze().clone();
    for (int i = 0; i < 3; ++i) {
        base_angular_vel[i] = result[i].item<float>();
        // std::cout << base_angular_vel[i] << " ";
        outfile << base_angular_vel[i] << " ";
    }
    outfile << std::endl;

    outfile << "dof_vel: ";
    for(int i=0; i<12; i++){
        joint_vel[i] = _lowState->motorState[reindex[i]].dq;
        outfile << joint_vel[i] << " ";
    }
    outfile << std::endl;

    outfile << "foot_vel";
    for (  int  i=0;i<12;i++ )
    {
        outfile << foot_vel[i] << " ";
    }
    outfile << std::endl;
    outfile << std::endl;
    outfile << std::endl;
}

void State_RL::open_amp_save_file()
{
    // 打开文件输出流
    // 获取当前系统时间
    std::time_t cTime = std::time(nullptr);
    std::tm* currentTm = std::localtime(&cTime);
    // 构建文件名，格式为 systime + 年-月-日.txt
    std::ostringstream fileNameStream;
    fileNameStream << "/home/chy/log/gazebo/" << angle_names[_cnt];
    std::string fileName = fileNameStream.str();
    // 以追加模式打开文件
    outfile = std::ofstream(fileName, std::ios::out | std::ios::app);
    if (!outfile) {
        std::cerr << "无法打开文件!" << std::endl;
    } else {
        // std::cout << "文件打开成功!" << std::endl;
    }
}

void State_RL::close_amp_save_file()
{
    if (outfile.is_open()) {
        outfile.close();
        // std::cout << "文件关闭保存成功!" << std::endl;
    }
}

torch::Tensor State_RL::quat_rotate_inverse(const torch::Tensor& q, const torch::Tensor& v) {
    // Ensure q and v are of the correct shape: (batch_size, 4) for quaternions and (batch_size, 3) for vectors
    auto shape = q.sizes();
    // std::cout << "shape: " << shape << std::endl;
    auto q_w = q.index({torch::indexing::Slice(), 3});  // last column is the w component
    // std::cout << "q_w: " << q_w << std::endl;
    auto q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});  // first three columns are the vector part
    // std::cout << "q_vec: " << q_vec << std::endl;
    // a = v * (2.0 * q_w^2 - 1.0).unsqueeze(-1)
    auto a = v * (2.0 * q_w.pow(2) - 1.0).unsqueeze(-1);
    // std::cout << "a: " << a << std::endl;
    // b = cross(q_vec, v) * q_w.unsqueeze(-1) * 2.0
    auto b = torch::cross(q_vec, v, /*dim=*/-1) * q_w.unsqueeze(-1) * 2.0;
    // std::cout << "b: " << b << std::endl;
    // c = q_vec * torch::bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    auto q_vec_reshaped = q_vec.view({shape[0], 1, 3});
    // std::cout << "q_vec_reshaped: " << q_vec_reshaped << std::endl;
    auto v_reshaped = v.view({shape[0], 3, 1});
    // std::cout << "v_reshaped: " << v_reshaped << std::endl;
    auto c = q_vec * torch::bmm(q_vec_reshaped, v_reshaped).squeeze(-1) * 2.0;
    // std::cout << "c: " << c << std::endl;
    // Return a - b + c
    // std::cout << "a - b + c: " << a - b + c << std::endl;
    return a - b + c;
}

void State_RL::load_policy()
{
    if (!nh.getParam("rl_policy_path", model_path)) {
        const char *policy_env = std::getenv("LEGBOT_RL_POLICY");
        if (policy_env != nullptr && policy_env[0] != '\0') {
            model_path = policy_env;
        } else {
            const std::string package_path = ros::package::getPath("unitree_guide");
            model_path = package_path + "/../../logs/policy_act_inference_stair.pt";
        }
    }
    std::cout << model_path << std::endl;
    // load model from check point
    std::cout << "cuda::is_available():" << torch::cuda::is_available() << std::endl;
    device= torch::kCPU;
    if (torch::cuda::is_available()){
        device = torch::kCUDA;
    }
    model = torch::jit::load(model_path);
    std::cout << "load model is successed!" << std::endl;
    model.to(device);
    std::cout << "load model to device!" << std::endl;
    model.eval();
}

void State_RL::printSegments(const torch::Tensor& tensor, int segment_size, const std::vector<int>& sub_sizes) {
    int num_segments = tensor.size(0) / segment_size;
    std::cout << "num_segments" << num_segments << tensor.size(0) << segment_size << std::endl;
    for (int seg = 0; seg < num_segments; ++seg) {
        auto segment = tensor.slice(0, seg * segment_size, (seg + 1) * segment_size);
        std::cout << "Segment " << seg + 1 << ":\n";

        int start = 0;
        for (size_t i = 0; i < sub_sizes.size(); ++i) {
            int size = sub_sizes[i];
            auto sub_segment = segment.slice(0, start, start + size);  // 按列（第1维）分割
            std::cout << "  Sub-segment " << i + 1 << " (" << size << " elements): ";
            std::string output_str = "  Sub-segment " + std::to_string(i + 1);
            printTensorHorizontal(sub_segment, output_str);
            start += size;
        }
    }
}

// 横排打印函数
void State_RL::printTensorHorizontal(const torch::Tensor& tensor, const std::string& name) {
    std::cout << name << " (" << tensor.sizes() << "): [ ";
    auto tensor_cpu = tensor.to(torch::kCPU);  // 确保张量在 CPU 上
    auto accessor = tensor_cpu.accessor<float, 1>();  // 假设是一维张量

    for (int i = 0; i < tensor.size(0); ++i) {
        std::cout << accessor[i] << " ";
    }
    std::cout << "]\n";
}
