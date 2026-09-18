#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
  try
  {
    if (argc != 2)
      throw std::runtime_error("usage: onnx_policy_contract_check PATH_TO_POLICY_DIR");
    const std::string directory = argv[1];
    const auto config = YAML::LoadFile(directory + "/config.yaml");
    const int observations = config["num_observations"].as<int>();
    const int history_length = config["history_length"] ? config["history_length"].as<int>() : 1;
    const int policy_input_size = observations * history_length;
    if (config["backend"].as<std::string>() != "onnxruntime" ||
        (observations != 42 && observations != 45) ||
        config["num_of_dofs"].as<int>() != 12 ||
        config["decimation"].as<int>() != 4 || history_length <= 0)
      throw std::runtime_error("unexpected GO2 ONNX policy configuration");

    Ort::Env environment{ORT_LOGGING_LEVEL_WARNING, "go2_policy_test"};
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    Ort::Session session(environment,
                         (directory + "/" + config["model_name"].as<std::string>()).c_str(), options);
    if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)
      throw std::runtime_error("expected one policy input and one output");
    const auto input_shape = session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto output_shape = session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape.size() != 2 || input_shape[1] != policy_input_size ||
        output_shape.size() != 2 || output_shape[1] != 12)
      throw std::runtime_error("unexpected ONNX input/output dimensions");

    std::vector<float> input(static_cast<size_t>(policy_input_size), 0.0F);
    const std::array<int64_t, 2> shape{1, policy_input_size};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input_tensor = Ort::Value::CreateTensor<float>(
      memory, input.data(), input.size(), shape.data(), shape.size());
    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name = session.GetInputNameAllocated(0, allocator);
    auto output_name = session.GetOutputNameAllocated(0, allocator);
    const char* input_names[] = {input_name.get()};
    const char* output_names[] = {output_name.get()};
    auto outputs = session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
    const auto count = outputs[0].GetTensorTypeAndShapeInfo().GetElementCount();
    const float* actions = outputs[0].GetTensorData<float>();
    if (count != 12)
      throw std::runtime_error("policy did not return 12 actions");
    for (size_t i = 0; i < count; ++i)
      if (!std::isfinite(actions[i]))
        throw std::runtime_error("policy returned a non-finite action");
    std::cout << "PASS: ONNX Runtime float32 [1," << policy_input_size
              << "] -> [1,12], 200/4=50 Hz contract\n";
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
