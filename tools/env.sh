#!/usr/bin/env bash
# Source this file from the workspace or any other directory.
legbot_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH=/usr/bin:/bin
source /opt/ros/jazzy/setup.bash
source "${legbot_workspace}/install/setup.bash"
export PATH="${legbot_workspace}/.venv-jazzy/bin:${PATH}"
export LEGBOT_DATA_DIR="${legbot_workspace}/data"
export ONNXRUNTIME_ROOT="${legbot_workspace}/third_party/onnxruntime"
# controller_manager 4.45.2 uses the diagnostic_updater 4.2.7 constructor ABI.
# Ubuntu currently has 4.2.6 installed, so every launch terminal must load the
# project-local compatible library before /opt/ros/jazzy/lib.
diagnostic_overlay="${legbot_workspace}/.deps/diagnostic_updater_4.2.7/opt/ros/jazzy/lib"
if [[ ! -f "${diagnostic_overlay}/libdiagnostic_updater.so" ]]; then
  echo "Missing diagnostic_updater 4.2.7 compatibility library: ${diagnostic_overlay}" >&2
  return 1 2>/dev/null || exit 1
fi
export LD_LIBRARY_PATH="${diagnostic_overlay}:${legbot_workspace}/third_party/onnxruntime/lib:${legbot_workspace}/third_party/libtorch/lib:${legbot_workspace}/third_party/pct/install/lib:${legbot_workspace}/third_party/Livox-SDK2/install/lib:${legbot_workspace}/third_party/unitree_sdk2/install/lib:${LD_LIBRARY_PATH:-}"
# SDK2 talks to the robot directly; keep ROS processes on the Fast DDS RMW.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset diagnostic_overlay legbot_workspace
