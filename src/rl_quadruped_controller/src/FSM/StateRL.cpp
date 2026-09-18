//
// Created by biao on 24-10-6.
//

#include "rl_quadruped_controller/FSM/StateRL.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>
#include <array>

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node)
{
    std::vector<T> values;
    for (const auto& val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

StateRL::StateRL(CtrlInterfaces& ctrl_interfaces,
                 CtrlComponent& ctrl_component,
                 const std::vector<double>& target_pos) :
    FSMState(FSMStateName::RL, "rl", ctrl_interfaces),
    node_(ctrl_component.node_),
    enable_estimator_(ctrl_component.enable_estimator_),
    estimator_(ctrl_component.estimator_)
{
    node_->declare_parameter("robot_pkg", robot_pkg_);
    node_->declare_parameter("model_folder", model_folder_);
    node_->declare_parameter("use_rl_thread", use_rl_thread_);
    robot_pkg_ = node_->get_parameter("robot_pkg").as_string();
    model_folder_ = node_->get_parameter("model_folder").as_string();
    use_rl_thread_ = node_->get_parameter("use_rl_thread").as_bool();

    RCLCPP_INFO(node_->get_logger(), "Using robot model from %s", robot_pkg_.c_str());
    const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
    const std::string model_path = package_share_directory + "/config/" + model_folder_;

    for (int i = 0; i < 12; i++)
    {
        init_pos_[i] = target_pos[i];
    }

    // read params from yaml
    loadYaml(model_path);

    if (params_.history_length > 1)
    {
        history_obs_buf_ = std::make_shared<ObservationBuffer>(1, params_.num_observations,
                                                               params_.history_length);
    }

    if (use_rl_thread_)
        throw std::runtime_error("Use synchronous inference with decimation; asynchronous upstream thread is disabled");
    if (params_.decimation <= 0 || params_.num_of_dofs != 12 || params_.history_length <= 0 ||
        (params_.num_observations != 42 && params_.num_observations != 45))
        throw std::runtime_error("Invalid GO2 policy contract");
    if (params_.history_layout != "frame_major" && params_.history_layout != "term_major")
        throw std::runtime_error("history_layout must be frame_major or term_major");
    const auto term_dimensions = observationTermDimensions();
    int observation_count = 0;
    for (const int dimension : term_dimensions) observation_count += dimension;
    if (observation_count != params_.num_observations)
        throw std::runtime_error("Configured observation terms do not match num_observations");

    const auto model_file = model_path + "/" + params_.model_name;
    const int64_t policy_input_size = params_.num_observations *
        static_cast<int64_t>(params_.history_length);
    RCLCPP_INFO(node_->get_logger(), "Loading %s policy: %s", params_.backend.c_str(), model_file.c_str());
    if (params_.backend == "torchscript")
    {
        model_ = torch::jit::load(model_file);
        model_.eval();
        torch::set_num_threads(1);
    }
    else if (params_.backend == "onnxruntime")
    {
        onnx_session_options_.SetIntraOpNumThreads(1);
        onnx_session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        onnx_session_ = std::make_unique<Ort::Session>(onnx_env_, model_file.c_str(), onnx_session_options_);
        if (onnx_session_->GetInputCount() != 1 || onnx_session_->GetOutputCount() != 1)
            throw std::runtime_error("GO2 ONNX policy must have exactly one input and one output");
        Ort::AllocatorWithDefaultOptions allocator;
        auto input_name = onnx_session_->GetInputNameAllocated(0, allocator);
        auto output_name = onnx_session_->GetOutputNameAllocated(0, allocator);
        onnx_input_name_ = input_name.get();
        onnx_output_name_ = output_name.get();
        const auto input_shape = onnx_session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        const auto output_shape = onnx_session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if (input_shape.size() != 2 || input_shape[1] != policy_input_size ||
            output_shape.size() != 2 || output_shape[1] != 12)
            throw std::runtime_error("GO2 ONNX policy dimensions do not match config.yaml");
    }
    else
    {
        throw std::runtime_error("Unsupported policy backend: " + params_.backend);
    }

}

void StateRL::enter()
{
    // Init observations
    obs_.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    obs_.commands = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    obs_.dof_pos = params_.default_dof_pos;
    obs_.dof_vel = torch::zeros({1, params_.num_of_dofs});
    obs_.actions = torch::zeros({1, params_.num_of_dofs});

    obs_.gravity_vec = obs_.gravity_vec.to(torch::kFloat32);
    // Init output
    output_torques = torch::zeros({1, params_.num_of_dofs});
    output_dof_pos_ = params_.default_dof_pos;

    // Init control
    control_.x = 0.0;
    control_.y = 0.0;
    control_.yaw = 0.0;

    // history
    if (history_obs_buf_) {
        history_obs_buf_->clear();
    }

    inference_tick_ = 0;
    first_inference_logged_ = false;
    running_ = true;
}

void StateRL::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    getState();
    if (inference_tick_++ % params_.decimation == 0)
    {
        runModel();
    }
    setCommand();
}

void StateRL::exit()
{
    running_ = false;
}

FSMStateName StateRL::checkChange()
{
    if (enable_estimator_ and !estimator_->safety())
    {
        return FSMStateName::PASSIVE;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        // Mode 2 is the public "fixed stand" command.  Entering FIXEDDOWN
        // here made a completed navigation mission crouch to the floor and
        // required a second, undocumented mode-2 pulse to stand again.  The
        // fixed-stand state already interpolates safely from the current RL
        // joint positions, so transition to it directly.
        return FSMStateName::FIXEDSTAND;
    default:
        return FSMStateName::RL;
    }
}

torch::Tensor StateRL::computeObservation()
{
    std::vector<torch::Tensor> obs_list;

    for (const std::string& observation : params_.observations)
    {
        if (observation == "lin_vel")
        {
            obs_list.push_back(obs_.lin_vel * params_.lin_vel_scale);
        }
        else if (observation == "ang_vel")
        {
            obs_list.push_back(obs_.ang_vel * params_.ang_vel_scale);
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(quatRotateInverse(obs_.base_quat, obs_.gravity_vec, params_.framework));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(obs_.commands * params_.commands_scale);
        }
        else if (observation == "dof_pos")
        {
            obs_list.push_back((obs_.dof_pos - params_.default_dof_pos) * params_.dof_pos_scale);
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(obs_.dof_vel * params_.dof_vel_scale);
        }
        else if (observation == "actions")
        {
            obs_list.push_back(obs_.actions);
        }
    }

    const torch::Tensor obs = cat(obs_list, 1);

    // std::cout << "Observation: " << obs << std::endl;
    torch::Tensor clamped_obs = clamp(obs, -params_.clip_obs, params_.clip_obs);
    return clamped_obs;
}

std::vector<int> StateRL::observationTermDimensions() const
{
    std::vector<int> dimensions;
    dimensions.reserve(params_.observations.size());
    for (const auto& observation : params_.observations)
    {
        if (observation == "lin_vel" || observation == "ang_vel" ||
            observation == "gravity_vec" || observation == "commands")
            dimensions.push_back(3);
        else if (observation == "dof_pos" || observation == "dof_vel" || observation == "actions")
            dimensions.push_back(params_.num_of_dofs);
        else
            throw std::runtime_error("Unsupported policy observation: " + observation);
    }
    return dimensions;
}

void StateRL::loadYaml(const std::string& config_path)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path + "/config.yaml");
    }
    catch ([[maybe_unused]] YAML::BadFile& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "The file '%s' does not exist", config_path.c_str());
        throw;
    }

    params_.model_name = config["model_name"].as<std::string>();
    params_.backend = config["backend"] ? config["backend"].as<std::string>() : "torchscript";
    params_.framework = config["framework"].as<std::string>();
    if (config["observations_history"].IsNull())
    {
        params_.observations_history = {};
    }
    else
    {
        params_.observations_history = ReadVectorFromYaml<int>(config["observations_history"]);
    }
    params_.history_length = config["history_length"]
        ? config["history_length"].as<int>()
        : static_cast<int>(params_.observations_history.empty() ? 1 : params_.observations_history.size());
    params_.history_layout = config["history_layout"]
        ? config["history_layout"].as<std::string>() : "frame_major";
    if (!params_.observations_history.empty() &&
        params_.history_length != static_cast<int>(params_.observations_history.size()))
        throw std::runtime_error("history_length must match observations_history when both are configured");
    params_.decimation = config["decimation"].as<int>();
    params_.num_observations = config["num_observations"].as<int>();
    params_.observations = ReadVectorFromYaml<std::string>(config["observations"]);
    params_.clip_obs = config["clip_obs"].as<double>();
    if (config["clip_actions_lower"].IsNull() && config["clip_actions_upper"].IsNull())
    {
        params_.clip_actions_upper = torch::tensor({}).view({1, -1});
        params_.clip_actions_lower = torch::tensor({}).view({1, -1});
    }
    else
    {
        params_.clip_actions_upper = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_upper"])).view({1, -1});
        params_.clip_actions_lower = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_lower"])).view({1, -1});
    }
    params_.action_scale = config["action_scale"].as<double>();
    params_.hip_scale_reduction = config["hip_scale_reduction"].as<double>();
    params_.hip_scale_reduction_indices = ReadVectorFromYaml<int>(config["hip_scale_reduction_indices"]);
    params_.num_of_dofs = config["num_of_dofs"].as<int>();
    params_.lin_vel_scale = config["lin_vel_scale"].as<double>();
    params_.ang_vel_scale = config["ang_vel_scale"].as<double>();
    params_.dof_pos_scale = config["dof_pos_scale"].as<double>();
    params_.dof_vel_scale = config["dof_vel_scale"].as<double>();
    const auto command_scales = ReadVectorFromYaml<double>(config["commands_scale"]);
    if (command_scales.size() != 3)
        throw std::runtime_error("commands_scale must contain [vx, vy, yaw] scaling");
    params_.commands_scale = torch::tensor(command_scales, torch::kFloat32).view({1, 3});
    params_.rl_kp = torch::tensor(ReadVectorFromYaml<double>(config["rl_kp"])).view({
        1, -1
    });
    params_.rl_kd = torch::tensor(ReadVectorFromYaml<double>(config["rl_kd"])).view({
        1, -1
    });
    params_.torque_limits = torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"])).view({1, -1});

    params_.default_dof_pos = torch::from_blob(init_pos_, {12}, torch::kDouble).clone().to(torch::kFloat).unsqueeze(0);
}

torch::Tensor StateRL::quatRotateInverse(const torch::Tensor& q, const torch::Tensor& v, const std::string& framework)
{
    torch::Tensor q_w;
    torch::Tensor q_vec;
    if (framework == "isaacsim")
    {
        q_w = q.index({torch::indexing::Slice(), 0});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(1, 4)});
    }
    else if (framework == "isaacgym")
    {
        q_w = q.index({torch::indexing::Slice(), 3});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    }
    const c10::IntArrayRef shape = q.sizes();

    const torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    const torch::Tensor b = cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
    const torch::Tensor c = q_vec * bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

torch::Tensor StateRL::forward()
{
    torch::autograd::GradMode::set_enabled(false);
    torch::Tensor clamped_obs = computeObservation();
    torch::Tensor actions;

    torch::Tensor policy_input;
    if (history_obs_buf_)
    {
        history_obs_buf_->insert(clamped_obs);
        if (params_.history_layout == "term_major")
            history_obs_ = history_obs_buf_->getTermMajorObsVec(observationTermDimensions());
        else
            history_obs_ = history_obs_buf_->getObsVec(params_.observations_history);
        policy_input = history_obs_;
    }
    else
    {
        policy_input = clamped_obs;
    }

    if (params_.backend == "torchscript")
    {
        actions = model_.forward({policy_input}).toTensor();
    }
    else
    {
        policy_input = policy_input.to(torch::kCPU, torch::kFloat32).contiguous();
        const std::array<int64_t, 2> input_shape{1, policy_input.numel()};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto input_value = Ort::Value::CreateTensor<float>(
            memory_info, policy_input.data_ptr<float>(), policy_input.numel(), input_shape.data(), input_shape.size());
        const char* input_names[] = {onnx_input_name_.c_str()};
        const char* output_names[] = {onnx_output_name_.c_str()};
        auto outputs = onnx_session_->Run(Ort::RunOptions{nullptr}, input_names, &input_value, 1, output_names, 1);
        const auto output_info = outputs[0].GetTensorTypeAndShapeInfo();
        if (output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            output_info.GetElementCount() != 12)
            throw std::runtime_error("GO2 ONNX policy returned an invalid output tensor");
        actions = torch::from_blob(outputs[0].GetTensorMutableData<float>(), {1, 12}, torch::kFloat32).clone();
    }

    if (params_.clip_actions_upper.numel() != 0 && params_.clip_actions_lower.numel() != 0)
    {
        return clamp(actions, params_.clip_actions_lower, params_.clip_actions_upper);
    }
    return actions;
}

void StateRL::getState()
{
    if (params_.framework == "isaacgym")
    {
        robot_state_.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[0].get().get_optional().value();
        robot_state_.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[1].get().get_optional().value();
        robot_state_.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[2].get().get_optional().value();
        robot_state_.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[3].get().get_optional().value();
    }
    else if (params_.framework == "isaacsim")
    {
        robot_state_.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[0].get().get_optional().value();
        robot_state_.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[1].get().get_optional().value();
        robot_state_.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[2].get().get_optional().value();
        robot_state_.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[3].get().get_optional().value();
    }

    robot_state_.imu.gyroscope[0] = ctrl_interfaces_.imu_state_interface_[4].get().get_optional().value();
    robot_state_.imu.gyroscope[1] = ctrl_interfaces_.imu_state_interface_[5].get().get_optional().value();
    robot_state_.imu.gyroscope[2] = ctrl_interfaces_.imu_state_interface_[6].get().get_optional().value();

    robot_state_.imu.accelerometer[0] = ctrl_interfaces_.imu_state_interface_[7].get().get_optional().value();
    robot_state_.imu.accelerometer[1] = ctrl_interfaces_.imu_state_interface_[8].get().get_optional().value();
    robot_state_.imu.accelerometer[2] = ctrl_interfaces_.imu_state_interface_[9].get().get_optional().value();

    for (int i = 0; i < 12; i++)
    {
        robot_state_.motor_state.q[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional().
            value();
        robot_state_.motor_state.dq[i] = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_optional().
            value();
        robot_state_.motor_state.tauEst[i] = ctrl_interfaces_.joint_effort_state_interface_[i].get().get_optional().
            value();
    }

    control_.x = ctrl_interfaces_.control_inputs_.ly;
    control_.y = -ctrl_interfaces_.control_inputs_.lx;
    control_.yaw = -ctrl_interfaces_.control_inputs_.rx;

    updated_ = true;
}

void StateRL::runModel()
{
    if (enable_estimator_)
    {
        obs_.lin_vel = torch::from_blob(estimator_->getVelocity().data(), {3}, torch::kDouble).clone().
            to(torch::kFloat).unsqueeze(0);
    }
    obs_.ang_vel = torch::tensor(robot_state_.imu.gyroscope, torch::kFloat32).unsqueeze(0);
    obs_.commands = torch::tensor({{control_.x, control_.y, control_.yaw}}, torch::kFloat32);
    obs_.base_quat = torch::tensor(robot_state_.imu.quaternion, torch::kFloat32).unsqueeze(0);
    const auto norm = obs_.base_quat.norm();
    if (!torch::isfinite(norm).item<bool>() || norm.item<float>() < 0.5f)
        throw std::runtime_error("Invalid GO2 IMU quaternion");
    obs_.base_quat = obs_.base_quat / norm;
    obs_.dof_pos = torch::tensor(robot_state_.motor_state.q, torch::kFloat32).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    obs_.dof_vel = torch::tensor(robot_state_.motor_state.dq, torch::kFloat32).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);

    const torch::Tensor clamped_actions = forward();
    if (clamped_actions.numel() != 12 || !torch::isfinite(clamped_actions).all().item<bool>())
        throw std::runtime_error("GO2 policy produced invalid actions");

    if (!first_inference_logged_)
    {
        const auto gravity = quatRotateInverse(obs_.base_quat, obs_.gravity_vec, params_.framework);
        RCLCPP_INFO_STREAM(
            node_->get_logger(),
            "First RL sample: quaternion=" << obs_.base_quat
            << ", projected_gravity=" << gravity
            << ", dof_position=" << obs_.dof_pos
            << ", action=" << clamped_actions);
        first_inference_logged_ = true;
    }

    for (const int i : params_.hip_scale_reduction_indices)
    {
        clamped_actions[0][i] *= params_.hip_scale_reduction;
    }

    obs_.actions = clamped_actions;

    const torch::Tensor actions_scaled = clamped_actions * params_.action_scale;
    // torch::Tensor output_torques = params_.rl_kp * (actions_scaled + params_.default_dof_pos - obs_.dof_pos) - params_.rl_kd * obs_.dof_vel;
    // output_torques = clamp(output_torques, -(params_.torque_limits), params_.torque_limits);

    output_dof_pos_ = actions_scaled + params_.default_dof_pos;

    for (int i = 0; i < params_.num_of_dofs; ++i)
    {
        robot_command_.motor_command.q[i] = output_dof_pos_[0][i].item<double>();
        robot_command_.motor_command.dq[i] = 0;
        robot_command_.motor_command.kp[i] = params_.rl_kp[0][i].item<double>();
        robot_command_.motor_command.kd[i] = params_.rl_kd[0][i].item<double>();
        robot_command_.motor_command.tau[i] = 0;
    }
}

void StateRL::setCommand() const
{
    for (int i = 0; i < 12; i++)
    {
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().
                                                                            set_value(
                                                                                robot_command_.motor_command.q[i]);
        std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(
            robot_command_.motor_command.dq[i]);
        std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(
            robot_command_.motor_command.kp[i]);
        std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(
            robot_command_.motor_command.kd[i]);
        std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().
                                                                          set_value(
                                                                              robot_command_.motor_command.tau[i]);
    }
}
