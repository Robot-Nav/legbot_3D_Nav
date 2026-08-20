/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include <iomanip>

State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl){
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;

#ifdef ROBOT_TYPE_Go1
    _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780; 
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

    _vxLim << -0.8, 0.8;
    _vyLim << -0.6, 0.6;
    _wyawLim << -1.0, 1.0;

}

State_Trotting::~State_Trotting(){
    delete _gait;
}

void State_Trotting::enter(){
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();

    amp_obs_thread = new std::thread(&State_Trotting::save_amp_obs_thread,this);
    ampthreadRunning = State_Trotting::RUNNING;
}

void State_Trotting::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
    ampthreadRunning = State_Trotting::STOP;
    amp_obs_thread->join();
    std::cout << "amp_obs_thread退出!" << std::endl;
    if (outfile.is_open()) {
        outfile.close();
        std::cout << "文件关闭成功!" << std::endl;
    }
}

FSMStateName State_Trotting::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        if (_last_cmd==static_cast<int>(UserCommand::START))
        {
            open_amp_save_file();
            dofPosSwitBeginTime = getTime();
        }
        _last_cmd = static_cast<int>(_lowState->userCmd);
        return FSMStateName::TROTTING;
    }
    else{
        _last_cmd = static_cast<int>(_lowState->userCmd);
        return FSMStateName::TROTTING;
    }
}

void State_Trotting::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();

    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();
    calcQQd();

    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }

}

bool State_Trotting::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20) ){
        return true;
    }else{
        return false;
    }
}

void State_Trotting::setHighCmd(double vx, double vy, double wz){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0; 
    _dYawCmd = wz;
}

void State_Trotting::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void State_Trotting::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
    // std::cout << "_vCmdBody(0): " << _vCmdBody(0) << std::endl;
    // std::cout << "_vCmdBody(1): " << _vCmdBody(1) << std::endl;
    // std::cout << "_dYawCmd: " << _dYawCmd << std::endl;
}

void State_Trotting::calcTau(){
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}

void State_Trotting::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }
    
    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}


void State_Trotting::save_amp_obs_thread()
{
    while(ampthreadRunning == State_Trotting::RUNNING)
    {
        if (_ctrlComp->ioInter->buttons[4] == 1 && !outfile.is_open()){
            open_amp_save_file();
            dofPosSwitBeginTime = getRosTime();
        }
        if (_ctrlComp->ioInter->buttons[5] == 1 && outfile.is_open()){
            close_amp_save_file();
        }
        long long _start_time = getRosTime();
        if (outfile.is_open() && (getRosTime() - dofPosSwitBeginTime) < _duration)
        {
            std::cout << "save data" << std::endl;
            refresh_amp_obs();
        }
        rosAbsoluteWait(_start_time, (long long)(amp_duration * 1000000));
    }
    ampthreadRunning = State_Trotting::OVER;
}

void State_Trotting::refresh_amp_obs(){
    auto opts = torch::TensorOptions().dtype(torch::kFloat32);
    motion_time = static_cast<float>(getRosTime() - dofPosSwitBeginTime)/1e6;
    // outfile << "motion_time: " << motion_time << std::endl;
    // outfile << "base_w_pos: ";
    for (int i=0; i<3; i++) {
        base_w_pos[i] = _ctrlComp->ioInter->_base_w_pos[i];
        // outfile << base_w_pos[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "base_ori: ";
    for (int i=0; i<4; i++) {
        base_w_orientation[i] = _ctrlComp->ioInter->_base_w_ori[i];
        // outfile << base_w_orientation[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "dof_pos: ";
    for(int i=0; i<12; i++){
        joint_pos[i] = _lowState->motorState[reindex[i]].q;
        // outfile << joint_pos[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "foot_pos: ";
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
        // outfile << foot_pos[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "base_w_linear_vel: ";
    for (int i=0; i<3; i++) {
        base_w_linear_vel[i] = _ctrlComp->ioInter->_base_w_linear_vel[i];
    }
    torch::Tensor orientation_tensor = torch::from_blob(base_w_orientation.data(), {int64_t(base_w_orientation.size())}, opts).unsqueeze(0).clone();
    torch::Tensor w_linear_vel_tensor = torch::from_blob(base_w_linear_vel.data(), {int64_t(base_w_linear_vel.size())}, opts).unsqueeze(0).clone();
    torch::Tensor result = quat_rotate_inverse(orientation_tensor, w_linear_vel_tensor).squeeze().clone();
    for (int i = 0; i < 3; ++i) {
        base_linear_vel[i] = result[i].item<float>();
        std::cout << base_linear_vel[i] << " ";
        // outfile << base_linear_vel[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "base_w_angular_vel: ";
    for (int i=0; i<3; i++) {
        base_w_angular_vel[i] = _ctrlComp->ioInter->_base_w_angular_vel[i];
    }
    torch::Tensor w_angular_vel_tensor = torch::from_blob(base_w_angular_vel.data(), {int64_t(base_w_angular_vel.size())}, opts).unsqueeze(0).clone();
    result = quat_rotate_inverse(orientation_tensor, w_angular_vel_tensor).squeeze().clone();
    for (int i = 0; i < 3; ++i) {
        base_angular_vel[i] = result[i].item<float>();
        std::cout << base_angular_vel[i] << " ";
        // outfile << base_angular_vel[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "dof_vel: ";
    for(int i=0; i<12; i++){
        joint_vel[i] = _lowState->motorState[reindex[i]].dq;
        // outfile << joint_vel[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "foot_vel";
    for (  int  i=0;i<12;i++ )
    {
        // outfile << foot_vel[i] << " ";
    }
    // outfile << std::endl;
    // outfile << std::endl;
    // outfile << std::endl;
    motion_data.insert(motion_data.end(), base_w_pos.begin(), base_w_pos.end());
    motion_data.insert(motion_data.end(), base_w_orientation.begin(), base_w_orientation.end());
    motion_data.insert(motion_data.end(), joint_pos.begin(), joint_pos.end());
    motion_data.insert(motion_data.end(), foot_pos.begin(), foot_pos.end());
    motion_data.insert(motion_data.end(), base_linear_vel.begin(), base_linear_vel.end());
    motion_data.insert(motion_data.end(), base_angular_vel.begin(), base_angular_vel.end());
    motion_data.insert(motion_data.end(), joint_vel.begin(), joint_vel.end());
    motion_data.insert(motion_data.end(), foot_vel.begin(), foot_vel.end());
    outfile << "[";
    for (size_t i = 0; i < motion_data.size(); ++i) {
        outfile << std::fixed << std::setprecision(5) << motion_data[i];
        if (i != motion_data.size() - 1) {
            outfile << ", ";
        }
    }
    motion_data.clear();
    outfile << "],";
    outfile << std::endl;
}

void State_Trotting::open_amp_save_file()
{
    const std::string content = R"({
"LoopMode": "Wrap",
"FrameDuration": 0.020,
"EnableCycleOffsetPosition": true,
"EnableCycleOffsetRotation": true,
"MotionWeight": 1,

"Frames":
[
)";
    // 获取当前系统时间
    std::time_t cTime = std::time(nullptr);
    std::tm* currentTm = std::localtime(&cTime);
    // 构建文件名，格式为 systime + 年-月-日.txt
    std::ostringstream fileNameStream;
    fileNameStream << "/home/chy/log/gazebo/" << "temp";
    std::string fileName = fileNameStream.str();
    // 以追加模式打开文件
    outfile = std::ofstream(fileName, std::ios::out | std::ios::app);
    if (!outfile) {
        std::cerr << "无法打开文件!" << std::endl;
    } else {
        std::cout << "文件打开成功!" << std::endl;
        outfile << content;
    }
}

void State_Trotting::close_amp_save_file()
{
    if (outfile.is_open()) {
        // 获取当前文件的写入位置
        std::streampos current_pos = outfile.tellp();

        if (current_pos > 0) {
            // 退回一个字符
            outfile.seekp(current_pos - 1);
        }

        // 写入结尾内容
        const std::string content = R"(]
}
)";
        outfile << content;

        // 关闭文件
        outfile.close();
        std::cout << "文件关闭保存成功!" << std::endl;
    }
}

torch::Tensor State_Trotting::quat_rotate_inverse(const torch::Tensor& q, const torch::Tensor& v) {
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