#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
thirdparty_dir="${script_dir}/lib/3rdparty"
build_jobs="${PCT_BUILD_JOBS:-$(nproc)}"

gtsam_dir="${thirdparty_dir}/gtsam-4.1.1"
cmake -S "${gtsam_dir}" -B "${gtsam_dir}/build" \
  -DCMAKE_INSTALL_PREFIX="${gtsam_dir}/install" \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_USE_SYSTEM_EIGEN=ON
cmake --build "${gtsam_dir}/build" --parallel "${build_jobs}"
cmake --install "${gtsam_dir}/build"

osqp_dir="${thirdparty_dir}/osqp"
cmake -S "${osqp_dir}" -B "${osqp_dir}/build" \
  -DCMAKE_INSTALL_PREFIX="${osqp_dir}/install" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "${osqp_dir}/build" --parallel "${build_jobs}"
cmake --install "${osqp_dir}/build"

echo "PCT third-party libraries installed successfully"
