#include  "free_dog_sdk_cpp/lowCmd.hpp"

namespace FDSC {

std::vector<uint8_t> lowCmd::buildCmd(bool debug) {
  std::vector<uint8_t> cmd(614);
  std::copy(head.begin(), head.end(), cmd.begin());
  cmd[2] = levelFlag;
  cmd[3] = frameReserve;
  std::copy(SN.begin(), SN.end(), cmd.begin() + 4);
  std::copy(version.begin(), version.end(), cmd.begin() + 12);
  std::copy(bandWidth.begin(), bandWidth.end(), cmd.begin() + 20);
  std::vector<uint8_t> motorCmdBytes = motorCmd_free_dog.getBytes();
  std::copy(motorCmdBytes.begin(), motorCmdBytes.end(), cmd.begin() + 22);
  std::vector<uint8_t> bmsBytes = bms.getBytes();
  std::copy(bmsBytes.begin(), bmsBytes.end(), cmd.begin() + 562);
  std::copy(wirelessRemote.begin(), wirelessRemote.end(), cmd.begin() + 566);
  std::vector<uint8_t> crcData(cmd.begin(), cmd.end() - 6);
  if (encrypt) {
    crc = encryptCrc(genCrc(crcData));
  } else {
    crc = genCrc(crcData);
  }
  std::copy(crc.begin(), crc.end(), cmd.end() - 4);

  if (debug) {
    std::cout << "Length: " << cmd.size() << std::endl;
    std::cout << "Data: ";
    int count_print = 0;
    for (const auto &byte : cmd) {
      std::cout << " 0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(byte) << " ";
      count_print++;
      if (count_print % 16 == 0) {
        std::cout << std::endl;
      }
    }
    std::cout << std::dec << std::endl;
  }

  return cmd;
}


void lowCmd::setSimStanceGain(int legID){
  motorCmd[legID*3+0].mode = 10;
  motorCmd[legID*3+0].Kp = 180;
  motorCmd[legID*3+0].Kd = 8;
  motorCmd[legID*3+1].mode = 10;
  motorCmd[legID*3+1].Kp = 180;
  motorCmd[legID*3+1].Kd = 8;
  motorCmd[legID*3+2].mode = 10;
  motorCmd[legID*3+2].Kp = 300;
  motorCmd[legID*3+2].Kd = 15;
}

void lowCmd::setRealStanceGain(int legID){
  motorCmd[legID*3+0].mode = 10;
  motorCmd[legID*3+0].Kp = 60;
  motorCmd[legID*3+0].Kd = 1;
  motorCmd[legID*3+1].mode = 10;
  motorCmd[legID*3+1].Kp = 40;
  motorCmd[legID*3+1].Kd = 1;
  motorCmd[legID*3+2].mode = 10;
  motorCmd[legID*3+2].Kp = 80;
  motorCmd[legID*3+2].Kd = 1;
}

void lowCmd::setFixDownGain(int legID){
  motorCmd[legID*3+0].mode = 10;
  motorCmd[legID*3+0].Kp = 60;
  motorCmd[legID*3+0].Kd = 1;
  motorCmd[legID*3+1].mode = 10;
  motorCmd[legID*3+1].Kp = 40;
  motorCmd[legID*3+1].Kd = 1;
  motorCmd[legID*3+2].mode = 10;
  motorCmd[legID*3+2].Kp = 80;
  motorCmd[legID*3+2].Kd = 1;
}

void lowCmd::setZeroGain(int legID){
  motorCmd[legID*3+0].mode = 10;
  motorCmd[legID*3+0].Kp = 0;
  motorCmd[legID*3+0].Kd = 0;
  motorCmd[legID*3+1].mode = 10;
  motorCmd[legID*3+1].Kp = 0;
  motorCmd[legID*3+1].Kd = 0;
  motorCmd[legID*3+2].mode = 10;
  motorCmd[legID*3+2].Kp = 0;
  motorCmd[legID*3+2].Kd = 0;
}

void lowCmd::setZeroGain(){
  for(int i(0); i<4; ++i){
    setZeroGain(i);
  }
}

void lowCmd::setStableGain(int legID){
  motorCmd[legID*3+0].mode = 10;
  motorCmd[legID*3+0].Kp = 0.8;
  motorCmd[legID*3+0].Kd = 0.8;
  motorCmd[legID*3+1].mode = 10;
  motorCmd[legID*3+1].Kp = 0.8;
  motorCmd[legID*3+1].Kd = 0.8;
  motorCmd[legID*3+2].mode = 10;
  motorCmd[legID*3+2].Kp = 0.8;
  motorCmd[legID*3+2].Kd = 0.8;
}

void lowCmd::setStableGain(){
  for(int i(0); i<4; ++i){
    setStableGain(i);
  }
}
void lowCmd::setSwingGain(int legID){
  motorCmd[legID*3+0].mode = 10;
  motorCmd[legID*3+0].Kp = 3;
  motorCmd[legID*3+0].Kd = 2;
  motorCmd[legID*3+1].mode = 10;
  motorCmd[legID*3+1].Kp = 3;
  motorCmd[legID*3+1].Kd = 2;
  motorCmd[legID*3+2].mode = 10;
  motorCmd[legID*3+2].Kp = 3;
  motorCmd[legID*3+2].Kd = 2;
}

void lowCmd::setZeroDq(int legID){
  motorCmd[legID*3+0].dq = 0;
  motorCmd[legID*3+1].dq = 0;
  motorCmd[legID*3+2].dq = 0;
}

void lowCmd::setZeroDq(){
  for(int i(0); i<4; ++i){
    setZeroDq(i);
  }
}
void lowCmd ::setZeroTau(int legID){
  motorCmd[legID*3+0].tau = 0;
  motorCmd[legID*3+1].tau = 0;
  motorCmd[legID*3+2].tau = 0;
}

}
