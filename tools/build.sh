#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
export PATH=/opt/ros/jazzy/bin:/usr/bin:/bin
export CC=/usr/bin/gcc CXX=/usr/bin/g++
export ONNXRUNTIME_ROOT="${PWD}/third_party/onnxruntime"
export CMAKE_PREFIX_PATH="${PWD}/third_party/libtorch:${PWD}/third_party/unitree_sdk2/install:${PWD}/third_party/Livox-SDK2/install:${PWD}/third_party/pct/install:/opt/ros/jazzy"
export MAKEFLAGS=-j1
export CMAKE_BUILD_PARALLEL_LEVEL=1
source /opt/ros/jazzy/setup.bash
colcon build --executor sequential "$@" --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF
