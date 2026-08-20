#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
lib_dir="${script_dir}/lib"
build_dir="${lib_dir}/build"
build_jobs="${PCT_BUILD_JOBS:-$(nproc)}"

cmake -S "${lib_dir}" -B "${build_dir}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${build_dir}" --parallel "${build_jobs}"

cp "${build_dir}"/src/a_star/a_star*.so "${lib_dir}/"
cp "${build_dir}"/src/trajectory_optimization/traj_opt*.so "${lib_dir}/"
cp "${build_dir}"/src/ele_planner/ele_planner*.so "${lib_dir}/"
cp "${build_dir}"/src/map_manager/py_map_manager*.so "${lib_dir}/"
cp "${build_dir}"/src/common/smoothing/libcommon_smoothing.so "${lib_dir}/"

echo "PCT planner modules installed in ${lib_dir}"
