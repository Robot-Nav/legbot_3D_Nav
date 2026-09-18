// Offline inference/serialization check. No ROS node, DDS, simulator or motor output.
#include <torch/script.h>
#include <torch/torch.h>
#include <yaml-cpp/yaml.h>
#include "ObservationBuffer.h"
#include <iostream>
#include <stdexcept>
int main(int argc,char**argv) {
 try {
  if(argc!=2)throw std::runtime_error("usage: policy_contract_check PATH_TO_TORCHSCRIPT_POLICY_DIR");
  std::string dir=argv[1];auto config=YAML::LoadFile(dir+"/config.yaml");
  const auto n=config["num_observations"].as<int>();
  if(n!=45||config["decimation"].as<int>()!=4)
   throw std::runtime_error("unexpected TorchScript observation/decimation contract");
  const bool uses_history=static_cast<bool>(config["observations_history"]);
  std::vector<int> ids;
  if(uses_history) {
   ids=config["observations_history"].as<std::vector<int>>();
   if(ids!=std::vector<int>({5,4,3,2,1,0}))
    throw std::runtime_error("unexpected TorchScript observation history contract");
  }
  ObservationBuffer history(1,n,6);
  for(int i=1;i<=6;++i)history.insert(torch::full({1,n},static_cast<float>(i)));
  if(uses_history) {
   auto stacked=history.getObsVec(ids);
   for(int i=0;i<6;++i)if(stacked[0][i*n].item<float>()!=6-i)throw std::runtime_error("history ordering mismatch");
  }
  history.clear();
  if(uses_history&&history.getObsVec(ids).abs().sum().item<float>()!=0)
   throw std::runtime_error("history reset failed");
  ObservationBuffer term_history(1,6,3);
  term_history.insert(torch::tensor({{10.,11.,12.,13.,14.,15.}}));
  term_history.insert(torch::tensor({{20.,21.,22.,23.,24.,25.}}));
  term_history.insert(torch::tensor({{30.,31.,32.,33.,34.,35.}}));
  const auto term_major=term_history.getTermMajorObsVec({2,1,3});
  const auto expected=torch::tensor({{10.,11.,20.,21.,30.,31.,
                                      12.,22.,32.,
                                      13.,14.,15.,23.,24.,25.,33.,34.,35.}});
  if(!torch::allclose(term_major,expected))throw std::runtime_error("term-major history ordering mismatch");
  auto model=torch::jit::load(dir+"/"+config["model_name"].as<std::string>(),torch::kCPU);
  model.eval();torch::NoGradGuard guard;torch::set_num_threads(1);
  auto obs=torch::zeros({1,45},torch::kFloat32);obs[0][8]=-1.0;
  torch::Tensor policy_input=obs;
  if(uses_history) {
   for(int i=0;i<6;++i)history.insert(obs);
   policy_input=history.getObsVec(ids);
  }
  auto actions=model.forward({policy_input}).toTensor();
  if(actions.sizes()!=torch::IntArrayRef({1,12})||!torch::isfinite(actions).all().item<bool>())
   throw std::runtime_error("policy did not return 12 finite actions");
  std::cout<<"PASS: history layouts, float32 [1,"<<policy_input.size(1)
           <<"] -> [1,12], reset, 200/4=50 Hz contract\n";
  return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
