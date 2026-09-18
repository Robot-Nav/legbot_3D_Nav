# Legbot 3D Navigation for Unitree GO2 (ROS 2)

[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/24.04/)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113?logo=gazebo)](https://gazebosim.org/docs/harmonic/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)](https://isocpp.org/)
[![Python 3.12](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](LICENSE)

English | [简体中文](README_CN.md) | [Technical analysis (Chinese)](docs/PROJECT_ANALYSIS_CN.md) | [Run guide (Chinese)](运行指南.md)

Legbot is a ROS 2 Jazzy workspace for 3D navigation with the Unitree GO2. It connects LiDAR-inertial odometry, local and multi-floor planning, reinforcement-learning locomotion, Gazebo simulation, and a guarded GO2 EDU hardware interface in one launch structure.

```text
Unitree L1 / simulated LiDAR + IMU
                 │
                 ▼
             FAST-LIO ─────► odometry + registered cloud
                 │                         │
                 └─────────────────────────┤
                                           ▼
RViz goal / recorded route / PCT ─────► SCAN planner
                                           │ local B-spline
                                           ▼
                                  velocity command adapter
                                           │
                                           ▼
                            RL locomotion controller (GO2)
                                           │
                                  Gazebo or Unitree SDK2
```

## Highlights

- ROS 2 Jazzy integration for Ubuntu 24.04 and Gazebo Harmonic.
- FAST-LIO localization from simulated sensors, Livox devices, or the built-in Unitree L1 LiDAR.
- SCAN local planning with occupancy mapping, A* collision-segment repair, rebound B-spline optimization, direct RViz goals, queued goals, and recorded 3D routes.
- Optional PCT multi-floor reference planning and EGO local planning.
- TorchScript and ONNX Runtime policy backends for GO2 locomotion.
- Three validated policy profiles: `moe_cts_77k` (default), `go2_cts`, and `mjlab_flat`.
- Separate simulation and hardware launch paths. Motor commands are disabled by default on the real-robot path.
- Offline checks for launch descriptions, dependencies, policy files, and waypoint state transitions.

## Supported environment

| Component | Version / target |
| --- | --- |
| Operating system | Ubuntu 24.04 LTS (x86_64) |
| Middleware | ROS 2 Jazzy, Fast DDS |
| Simulator | Gazebo Harmonic |
| Languages | C++17, Python 3.12 |
| Robot | Unitree GO2 / GO2 EDU |
| Sensors | Unitree L1, Livox Mid-360, simulated LiDAR and IMU |
| Inference | LibTorch and ONNX Runtime |

Other distributions and architectures have not been validated by this repository.

## Repository layout

```text
src/          ROS 2 packages and vendored planner/controller sources
docs/         architecture, model provenance, upstream revision locks
tests/        non-actuating offline verification scripts
tools/        build, environment, checks, and read-only robot preflight
data/         runtime output (not committed, except its documentation)
third_party/  local SDK/runtime installations (not committed)
```

| Package | Role |
| --- | --- |
| `legbot_bringup` | Launch files, worlds, adapters, RViz configurations |
| `go2_description` | GO2 URDF, meshes, ros2_control and policy profiles |
| `fast_lio` | LiDAR-IMU odometry and registered point clouds |
| `scan_planner` | Occupancy map, local A*, B-spline planning and GO2 command adapter |
| `pct_planner` | Tomogram/elevation-map multi-floor global planning |
| `ego_planner` | Optional EGO local trajectory planner |
| `rl_quadruped_controller` | TorchScript/ONNX locomotion inference and joint PD control |
| `gz_quadruped_hardware` | Gazebo Harmonic ros2_control system |
| `hardware_unitree_sdk2` | GO2 SDK2 hardware and L1 LiDAR bridge |
| `legbot_runtime` | Shared ROS 2 compatibility and transform utilities |

## Dependencies

Install ROS 2 Jazzy Desktop and the standard development tools first. The workspace additionally requires ros2_control, ros-gz, PCL, Eigen, OpenCV, yaml-cpp, Fast DDS, NumPy, SciPy, and the ROS packages declared in each `package.xml`.

```bash
sudo apt update
sudo apt install ros-dev-tools python3-rosdep python3-vcstool \
  libeigen3-dev libpcl-dev libopencv-dev libyaml-cpp-dev \
  ros-jazzy-ros-gz ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-pcl-ros ros-jazzy-cv-bridge ros-jazzy-tf2-sensor-msgs \
  ros-jazzy-backward-ros ros-jazzy-rmw-fastrtps-cpp
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Large local runtimes are deliberately excluded from Git. Before building, provide:

```text
third_party/libtorch
third_party/onnxruntime
third_party/unitree_sdk2/install
third_party/Livox-SDK2/install
third_party/pct/install
.deps/diagnostic_updater_4.2.7/opt/ros/jazzy/lib
```

Pinned SDK revisions are listed in [`docs/SENSOR_SDK_UPSTREAM_LOCK.json`](docs/SENSOR_SDK_UPSTREAM_LOCK.json); policy files and hashes are listed in [`docs/GO2_POLICY_MODELS_LOCK.json`](docs/GO2_POLICY_MODELS_LOCK.json). LibTorch must use the C++11 ABI. The current workspace setup uses ONNX Runtime 1.23.2.

## Build and verify

```bash
git clone --branch ROS2 https://github.com/Robot-Nav/legbot_3D_Nav.git
cd legbot_3D_Nav

./tools/build.sh
source tools/env.sh
./tools/check_offline.sh
```

`check_offline.sh` does not start Gazebo and does not connect to a robot. A successful run ends with:

```text
PASS offline checks; no simulation or robot experiment was started by this check
```

## Quick start: simulation

Open one terminal per command and run `source tools/env.sh` in every new terminal.

```bash
# Terminal 1: Gazebo and the GO2 controller
ros2 launch legbot_bringup simulation.launch.py gui:=true \
  world:=$PWD/install/legbot_bringup/share/legbot_bringup/worlds/ScanFlatFastLio.sdf \
  x:=-7.0 y:=0.0

# Terminal 2: stand-up and navigation state machine
ros2 launch legbot_bringup go2_demo_control.launch.py

# Terminal 3: LiDAR-inertial odometry
ros2 launch legbot_bringup fastlio.launch.py use_sim_time:=true \
  config:=$PWD/install/legbot_bringup/share/legbot_bringup/config/fastlio_sim.yaml

# Terminal 4: local planner
ros2 launch legbot_bringup scan.launch.py use_sim_time:=true

# Terminal 5: visualization
ros2 launch legbot_bringup scan_rviz.launch.py

# Terminal 6: RViz goal handler (direct replacement by default)
ros2 launch legbot_bringup scan_waypoints.launch.py
```

Wait for `IMU Initial Done` and `FAST-LIO ready: enabling SCAN waypoint mission`, then use **2D Goal Pose** in RViz. The default navigation source is FAST-LIO; Gazebo ground truth is not fed into the control chain.

See the [complete run guide](运行指南.md) for headless mode, stairs, queued goals, 3D route recording, PCT, EGO, diagnostics, and shutdown order.

## Real robot safety

The hardware launch defaults to `enable_commands:=false`. Start with the read-only preflight:

```bash
./tools/go2_real_preflight.sh <ethernet-interface>
```

It checks the `192.168.123.x` link, joint state, body IMU, L1 IMU, and LiDAR fields without publishing `LowCmd`. Do not enable commands until the network interface, sensor timestamps, extrinsics, emergency stop, support rig, and operator responsibilities have been checked on site.

## Algorithms

FAST-LIO performs tightly coupled LiDAR-IMU state estimation with an iterated error-state Kalman filter. SCAN maintains a probabilistic sliding occupancy map, repairs colliding path segments with A*, and minimizes a weighted B-spline objective:

$$J = \lambda_s J_{smooth} + \lambda_c J_{collision} + \lambda_f J_{feasibility} + \lambda_r J_{reference}.$$

The controller converts the policy action into a joint target and applies PD control:

$$q_d = q_0 + s_a a, \qquad \tau = K_p(q_d-q)-K_d\dot q,$$

followed by the configured torque limits. The default `moe_cts_77k` profile uses ten 45-dimensional observation frames in term-major order, producing a 450-dimensional ONNX input and 12 joint actions.

For the complete derivation, parameter interpretation, topic flow, validation results, and known limitations, read [docs/PROJECT_ANALYSIS_CN.md](docs/PROJECT_ANALYSIS_CN.md).

## Project status

The Gazebo flat-ground navigation loop and the default policy have been exercised with FAST-LIO and SCAN. Stair locomotion has been tested at the dynamics level. Real-hardware actuation is intentionally not claimed as validated: site-specific networking, calibration, state estimation, and low-speed command release remain operator acceptance items.

## License and attribution

Original integration code is licensed under [Apache License 2.0](LICENSE). Vendored components and pretrained weights retain their own licenses; in particular, FAST-LIO and PCT include GPL-2.0 code. Review [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) before redistribution or commercial use.

## Acknowledgements

This integration builds on FAST-LIO, SCAN-Planner, EGO-Planner, PCT Planner, Unitree SDK2, Livox SDK/Driver2, `quadruped_ros2_control`, LibTorch, and ONNX Runtime. Upstream revisions and notices are preserved under `docs/`, package license files, and `THIRD_PARTY_NOTICES.md`.
