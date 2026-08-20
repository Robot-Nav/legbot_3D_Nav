/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

void RosShutDown(int sig){
	ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}

IOROS::IOROS():IOInterface(){
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;
    ros::param::get("/robot_name", _robot_name);
    std::cout << "robot_name: " << _robot_name << std::endl;

    // start subscriber
    initRecv();
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(300000);     //wait for subscribers start
    // initialize publisher
    initSend();   

    signal(SIGINT, RosShutDown);

    cmdPanel = new KeyBoard();
}

IOROS::~IOROS(){
    delete cmdPanel;
    ros::shutdown();
}

void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd){
    for(int i(0); i < 12; ++i){
        _lowCmd.motorCmd[i].mode = lowCmd->motorCmd[i].mode;
        _lowCmd.motorCmd[i].q = lowCmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].tau = lowCmd->motorCmd[i].tau;
        _lowCmd.motorCmd[i].Kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].Kp = lowCmd->motorCmd[i].Kp;
    }
    for(int m(0); m < 12; ++m){
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
}

void IOROS::recvState(LowlevelState *state){
    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].ddq = _lowState.motorState[i].ddq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
    }
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
}

void IOROS::initSend(){
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_thigh_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_calf_controller/command", 1);
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_hip_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_thigh_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_calf_controller/command", 1);
    _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_hip_controller/command", 1);
    _servo_pub[7] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_calf_controller/command", 1);
    _servo_pub[9] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_hip_controller/command", 1);
    _servo_pub[10] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_thigh_controller/command", 1);
    _servo_pub[11] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_calf_controller/command", 1);
}

void IOROS::initRecv(){
    _imu_sub = _nm.subscribe("/trunk_imu", 1, &IOROS::imuCallback, this);
    _servo_sub[0] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_hip_controller/state", 1, &IOROS::FRhipCallback, this);
    _servo_sub[1] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_thigh_controller/state", 1, &IOROS::FRthighCallback, this);
    _servo_sub[2] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_calf_controller/state", 1, &IOROS::FRcalfCallback, this);
    _servo_sub[3] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_hip_controller/state", 1, &IOROS::FLhipCallback, this);
    _servo_sub[4] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_thigh_controller/state", 1, &IOROS::FLthighCallback, this);
    _servo_sub[5] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_calf_controller/state", 1, &IOROS::FLcalfCallback, this);
    _servo_sub[6] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_hip_controller/state", 1, &IOROS::RRhipCallback, this);
    _servo_sub[7] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_thigh_controller/state", 1, &IOROS::RRthighCallback, this);
    _servo_sub[8] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_calf_controller/state", 1, &IOROS::RRcalfCallback, this);
    _servo_sub[9] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_hip_controller/state", 1, &IOROS::RLhipCallback, this);
    _servo_sub[10] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_thigh_controller/state", 1, &IOROS::RLthighCallback, this);
    _servo_sub[11] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_calf_controller/state", 1, &IOROS::RLcalfCallback, this);
    _foot_states_sub[0] = _nm.subscribe("/ground_truth/FL_foot", 1, &IOROS::FL_footCallback, this);
    _foot_states_sub[1] = _nm.subscribe("/ground_truth/FR_foot", 1, &IOROS::FR_footCallback, this);
    _foot_states_sub[2] = _nm.subscribe("/ground_truth/RL_foot", 1, &IOROS::RL_footCallback, this);
    _foot_states_sub[3] = _nm.subscribe("/ground_truth/RR_foot", 1, &IOROS::RR_footCallback, this);
    _base_w_sub = _nm.subscribe("/ground_truth/base_w", 1, &IOROS::baseWorldCallback, this);
    _base_t_sub = _nm.subscribe("/ground_truth/base_trunk", 1, &IOROS::baseTrunkCallback, this);
    _time_sub = _nm.subscribe("/clock", 1, &IOROS::timeCallback, this);
    joy_sub = _nm.subscribe("/joy", 1, &IOROS::joyCallback, this);
}

void IOROS::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    axes = msg->axes;          // 更新 axes 数据
    buttons = msg->buttons;    // 更新 buttons 数据
    // 打印数据
    // ROS_INFO("Axes: ");
    // for (float axis : axes) {
    //     std::cout << axis << " ";
    // }
    // std::cout << std::endl;
    // //
    // ROS_INFO("Buttons: ");
    // for (int button : buttons) {
    //     std::cout << button << " ";
    // }
    // std::cout << std::endl;
}

void IOROS::timeCallback(const rosgraph_msgs::Clock& msg) {
    current_time = (msg.clock.sec)*1e6 + (msg.clock.nsec)/1000;
    // std::cout << "current_time: " << current_time << std::endl;
}

void IOROS::baseWorldCallback(const nav_msgs::Odometry& msg) {
    // current_time = ros::Time::now();
    _base_w_pos[0] = msg.pose.pose.position.x;
    _base_w_pos[1] = msg.pose.pose.position.y;
    _base_w_pos[2] = msg.pose.pose.position.z;
    _base_w_ori[0] = msg.pose.pose.orientation.x;
    _base_w_ori[1] = msg.pose.pose.orientation.y;
    _base_w_ori[2] = msg.pose.pose.orientation.z;
    _base_w_ori[3] = msg.pose.pose.orientation.w;
    _base_w_linear_vel[0] = msg.twist.twist.linear.x;
    _base_w_linear_vel[1] = msg.twist.twist.linear.y;
    _base_w_linear_vel[2] = msg.twist.twist.linear.z;
    _base_w_angular_vel[0] = msg.twist.twist.angular.x;
    _base_w_angular_vel[1] = msg.twist.twist.angular.y;
    _base_w_angular_vel[2] = msg.twist.twist.angular.z;
    // std::cout << "_base_w_angular_vel" << _base_w_angular_vel[0] << " " << _base_w_angular_vel[1] << " " << _base_w_angular_vel[2] << std::endl;
}

void IOROS::baseTrunkCallback(const nav_msgs::Odometry& msg) {
    // current_time = ros::Time::now();
    _base_t_pos[0] = msg.pose.pose.position.x;
    _base_t_pos[1] = msg.pose.pose.position.y;
    _base_t_pos[2] = msg.pose.pose.position.z;
    _base_t_ori[0] = msg.pose.pose.orientation.x;
    _base_t_ori[1] = msg.pose.pose.orientation.y;
    _base_t_ori[2] = msg.pose.pose.orientation.z;
    _base_t_ori[3] = msg.pose.pose.orientation.w;
    _base_t_linear_vel[0] = msg.twist.twist.linear.x;
    _base_t_linear_vel[1] = msg.twist.twist.linear.y;
    _base_t_linear_vel[2] = msg.twist.twist.linear.z;
    _base_t_angular_vel[0] = msg.twist.twist.angular.x;
    _base_t_angular_vel[1] = msg.twist.twist.angular.y;
    _base_t_angular_vel[2] = msg.twist.twist.angular.z;
    // std::cout << "_base_t_angular_vel" << _base_t_angular_vel[0] << " " << _base_t_angular_vel[1] << " " << _base_t_angular_vel[2] << std::endl;
}

void IOROS::FL_footCallback(const nav_msgs::Odometry& msg) {
    // std::cout << "Received FL foot position:" << std::endl;
    _FL_foot_pos[0] = msg.pose.pose.position.x;
    _FL_foot_pos[1] = msg.pose.pose.position.y;
    _FL_foot_pos[2] = msg.pose.pose.position.z;
    _FL_foot_vel[0] = msg.twist.twist.linear.x;
    _FL_foot_vel[1] = msg.twist.twist.linear.y;
    _FL_foot_vel[2] = msg.twist.twist.linear.z;
}

void IOROS::FR_footCallback(const nav_msgs::Odometry& msg) {
    _FR_foot_pos[0] = msg.pose.pose.position.x;
    _FR_foot_pos[1] = msg.pose.pose.position.y;
    _FR_foot_pos[2] = msg.pose.pose.position.z;
    _FR_foot_vel[0] = msg.twist.twist.linear.x;
    _FR_foot_vel[1] = msg.twist.twist.linear.y;
    _FR_foot_vel[2] = msg.twist.twist.linear.z;
}

void IOROS::RL_footCallback(const nav_msgs::Odometry& msg) {
    // std::cout << "Received RL foot position:" << std::endl;
    _RL_foot_pos[0] = msg.pose.pose.position.x;
    _RL_foot_pos[1] = msg.pose.pose.position.y;
    _RL_foot_pos[2] = msg.pose.pose.position.z;
    _RL_foot_vel[0] = msg.twist.twist.linear.x;
    _RL_foot_vel[1] = msg.twist.twist.linear.y;
    _RL_foot_vel[2] = msg.twist.twist.linear.z;
}

void IOROS::RR_footCallback(const nav_msgs::Odometry& msg) {
    // std::cout << "Received RR foot position:" << std::endl;
    _RR_foot_pos[0] = msg.pose.pose.position.x;
    _RR_foot_pos[1] = msg.pose.pose.position.y;
    _RR_foot_pos[2] = msg.pose.pose.position.z;
    _RR_foot_vel[0] = msg.twist.twist.linear.x;
    _RR_foot_vel[1] = msg.twist.twist.linear.y;
    _RR_foot_vel[2] = msg.twist.twist.linear.z;
}

void IOROS::imuCallback(const sensor_msgs::Imu & msg)
{ 
    _lowState.imu.quaternion[0] = msg.orientation.w;
    _lowState.imu.quaternion[1] = msg.orientation.x;
    _lowState.imu.quaternion[2] = msg.orientation.y;
    _lowState.imu.quaternion[3] = msg.orientation.z;

    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;
    
    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void IOROS::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[0].mode = msg.mode;
    _lowState.motorState[0].q = msg.q;
    _lowState.motorState[0].dq = msg.dq;
    _lowState.motorState[0].tauEst = msg.tauEst;
}

void IOROS::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[1].mode = msg.mode;
    _lowState.motorState[1].q = msg.q;
    _lowState.motorState[1].dq = msg.dq;
    _lowState.motorState[1].tauEst = msg.tauEst;
}

void IOROS::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[2].mode = msg.mode;
    _lowState.motorState[2].q = msg.q;
    _lowState.motorState[2].dq = msg.dq;
    _lowState.motorState[2].tauEst = msg.tauEst;
}

void IOROS::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[3].mode = msg.mode;
    _lowState.motorState[3].q = msg.q;
    _lowState.motorState[3].dq = msg.dq;
    _lowState.motorState[3].tauEst = msg.tauEst;
}

void IOROS::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[4].mode = msg.mode;
    _lowState.motorState[4].q = msg.q;
    _lowState.motorState[4].dq = msg.dq;
    _lowState.motorState[4].tauEst = msg.tauEst;
}

void IOROS::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[5].mode = msg.mode;
    _lowState.motorState[5].q = msg.q;
    _lowState.motorState[5].dq = msg.dq;
    _lowState.motorState[5].tauEst = msg.tauEst;
}

void IOROS::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[6].mode = msg.mode;
    _lowState.motorState[6].q = msg.q;
    _lowState.motorState[6].dq = msg.dq;
    _lowState.motorState[6].tauEst = msg.tauEst;
}

void IOROS::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[7].mode = msg.mode;
    _lowState.motorState[7].q = msg.q;
    _lowState.motorState[7].dq = msg.dq;
    _lowState.motorState[7].tauEst = msg.tauEst;
}

void IOROS::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[8].mode = msg.mode;
    _lowState.motorState[8].q = msg.q;
    _lowState.motorState[8].dq = msg.dq;
    _lowState.motorState[8].tauEst = msg.tauEst;
}

void IOROS::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[9].mode = msg.mode;
    _lowState.motorState[9].q = msg.q;
    _lowState.motorState[9].dq = msg.dq;
    _lowState.motorState[9].tauEst = msg.tauEst;
}

void IOROS::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[10].mode = msg.mode;
    _lowState.motorState[10].q = msg.q;
    _lowState.motorState[10].dq = msg.dq;
    _lowState.motorState[10].tauEst = msg.tauEst;
}

void IOROS::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[11].mode = msg.mode;
    _lowState.motorState[11].q = msg.q;
    _lowState.motorState[11].dq = msg.dq;
    _lowState.motorState[11].tauEst = msg.tauEst;
}


#endif  // COMPILE_WITH_ROS