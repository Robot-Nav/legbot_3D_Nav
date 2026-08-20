# legbot_3D_Nav

[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](https://github.com/Robot-Nav/legbot_3D_Nav/tree/main#)
[![ROS](https://img.shields.io/badge/ROS-Noetic-22314E.svg)](http://wiki.ros.org/noetic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420.svg)](https://releases.ubuntu.com/20.04/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%2011-9cf.svg)](http://gazebosim.org/)

Three-dimensional, cross-floor navigation for a Unitree A1: FAST-LIO mapping and
odometry, PCT global planning, selectable EGO/SCAN local avoidance, and an RL
locomotion controller integrated in Gazebo.

中文文档：[README_CN.md](README_CN.md)

## Simulation Videos

| EGO-Planner | SCAN-Planner |
| :---: | :---: |
| <video src="https://github.com/user-attachments/assets/c5f88789-fc8c-410c-974f-f1057fb6bb33" width="100%" controls muted autoplay loop></video> | <video src="https://github.com/user-attachments/assets/41fd4757-47ea-4a4e-95ac-719ddbcb9ef5" width="100%" controls muted autoplay loop></video> |

## Architecture

```text
LiDAR + IMU -> FAST-LIO ---------------------> odom + registered cloud
                                                        |
PCD -> PCT tomogram -> multi-layer A* -> /pct_path      |
                                  |                     |
                                  v                     v
                         reference_path_transform -> EGO / SCAN
                                                        |
                                                  local B-spline
                                                        |
                                              /cmd_vel -> A1 RL policy
                                                        |
                                                    Gazebo A1
```

## EGO and SCAN Simulation Metrics

Both planners completed the same route in `Building.world`, generated from
`building2_9.pcd`, with obstacle avoidance enabled.

| Metric | EGO | SCAN |
| --- | ---: | ---: |
| Result | Success | Success |
| Runtime | 276.826 s | 187.176 s |
| Final XY error | 0.102 m | 0.097 m |
| Final Z error | 0.384 m | 0.379 m |
| Final 3D error | 0.397 m | 0.392 m |
| Travelled distance | 81.373 m | 79.284 m |
| Maximum command speed | 0.750 m/s | 0.500 m/s |
| Avoidance configuration | Inflated occupancy cost enabled | `collision_check_enabled=true` |

Runtime varies with hardware load and sensor timing.

## Platform and Dependencies

- Ubuntu 20.04, ROS Noetic, Gazebo Classic 11
- GCC/G++, CMake, Eigen3, OpenCV, PCL, Boost and Armadillo
- Python 3.8 with NumPy, SciPy, CuPy and Open3D
- libtorch for the A1 policy
- GTSAM and OSQP for PCT

Third-party component licenses are listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## Build

```bash
cd ~/legbot_3D_Nav
source /opt/ros/noetic/setup.bash
catkin_make --force-cmake -j2
```

PCT has a standalone native build and is intentionally excluded from catkin:

```bash
cd ~/legbot_3D_Nav/src/PCT_planner/planner
./build_thirdparty.sh
./build.sh
```

### PCT environment versions

| Component | Version |
| --- | --- |
| Ubuntu / ROS | 20.04 / Noetic |
| Python | 3.8.20 |
| NumPy / SciPy | 1.24.4 / 1.10.1 |
| Open3D | 0.19.0 |
| CuPy | `cupy-cuda12x` 12.3.0 |
| CUDA runtime / driver used by CuPy | 12.2 / 12.2 |
| GTSAM / OSQP | 4.1.1 / 0.6.2 (bundled source) |
| CMake / GCC | 3.16.3 / 9.4.0 |

Create the dedicated PCT environment with the pinned requirements:

```bash
conda create -n pct-planner python=3.8.20 -y
conda activate pct-planner
python -m pip install -r ~/legbot_3D_Nav/src/PCT_planner/requirements.txt
export PCT_PYTHON="$(which python)"
```

Source ROS Noetic before launching so that `rospy` and ROS messages are visible.
For a different CUDA major version, replace the CuPy wheel with the matching
official package. The PCT ROS wrappers use `python3`; an already prepared
environment can be selected directly:

```bash
export PCT_PYTHON=/path/to/pct-environment/bin/python
```

Configure libtorch without editing CMake files:

```bash
cd ~/legbot_3D_Nav
catkin_make --force-cmake -DCMAKE_PREFIX_PATH=/path/to/libtorch -j2
```

The `build/`, `devel/` and PCT third-party build directories are generated
locally by the commands above.

## Common Terminal Setup

Run this in every new terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/legbot_3D_Nav/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
```

## Quick Start: Gazebo Ground-Truth Odometry

Start the processes in this order.

```bash
# Terminal 1: Gazebo simulation
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find unitree_gazebo)/models
roslaunch legbot_bringup simulation.launch gui:=true

# Terminal 2: A1 controller; press 2 to stand, then 6 for RL mode
roslaunch legbot_bringup controller.launch

# Terminal 3: select exactly one local planner
roslaunch legbot_bringup local_planners.launch local_planner:=scan
# or: roslaunch legbot_bringup local_planners.launch local_planner:=ego

# Terminal 4: generate and publish the Building tomogram
roslaunch legbot_bringup pct_tomography.launch scene:=Building

# Terminal 5: plan and publish /pct_path
roslaunch legbot_bringup pct_plan.launch scene:=Building

# Terminal 6: dedicated navigation RViz
roslaunch legbot_bringup visualization.launch
```

Do not launch EGO and SCAN together: both ultimately publish `/cmd_vel`.

## Local Planner Parameters

| Parameter | EGO | SCAN |
| --- | ---: | ---: |
| Planning horizon | 2.5 m | 2.0 m |
| Maximum velocity | 0.75 m/s | 0.50 m/s |
| Maximum acceleration | 0.50 m/s² | 0.30 m/s² |
| Collision clearance / optimizer distance | 0.35 m | 0.20 m |
| Body/self-return handling | Ellipsoid 0.25/0.25/0.15 m | Double cylinder radius 0.15 m |
| Global reference | PCT interface | Piecewise linear by default |
| Height progress weighting | — | 8.0 |

Explicit SCAN launch:

```bash
roslaunch legbot_bringup local_planners.launch local_planner:=scan \
  planning_horizon:=2.0 reference_mode:=piecewise_linear \
  z_projection_weight:=8.0 enable_collision_check:=true
```

Explicit EGO launch:

```bash
roslaunch legbot_bringup local_planners.launch local_planner:=ego \
  ego_planning_horizon:=2.5 ego_collision_clearance:=0.35 \
  ego_robot_clearance_x:=0.25 ego_robot_clearance_y:=0.25 \
  ego_robot_clearance_z:=0.15
```

### SCAN RViz markers

| Topic | Display |
| --- | --- |
| `/local_planner/goal` | Current PCT/reference segment waypoint |
| `/scan/local_target` | Collision-checked moving local target |
| `/scan/heading_vector` | Horizontal target heading |
| `/local_planner/optimal_path` | Current optimized local path |
| `/local_planner/grid_map/occupancy_inflate` | Inflated local obstacles |

## FAST-LIO: Build a Map First

Do not start PCT navigation while collecting a new map. Explore the environment
manually, close FAST-LIO cleanly, then prepare the saved PCD.

```bash
# Terminal 1: sensor simulation; disable ground-truth navigation odometry
roslaunch legbot_bringup simulation.launch publish_ground_truth:=false

# Terminal 2: accumulate one PCD in the FAST-LIO odom frame
roslaunch legbot_bringup fastlio.launch save_pcd:=true pcd_save_interval:=-1

# Terminal 3: controller; press 2, then 6
roslaunch legbot_bringup controller.launch

# Terminal 4: inspect odometry and coverage
roslaunch legbot_bringup visualization.launch
```

Drive the robot with a joystick or conservative `/cmd_vel` commands. Check
`/fast_lio/odometry_base`, `/fast_lio/cloud_registered` and `/fast_lio/map`.
Press Ctrl-C in the FAST-LIO terminal only after coverage is complete. A clean
shutdown writes `src/FAST_LIO/PCD/scans.pcd`.

Prepare a PCT input without a machine-specific path:

```bash
python3 src/PCT_planner/tomography/scripts/downsample_pcd.py \
  src/FAST_LIO/PCD/scans.pcd \
  src/PCT_planner/src/pcd/my_building.pcd \
  --voxel-size 0.05 --remove-outliers
```

Optional `--min-bound x,y,z --max-bound x,y,z` removes unrelated regions.
Inspect the result before planning; floors, stairs and landings must remain.

Generate a custom tomogram and plan without editing source code:

```bash
roslaunch legbot_bringup pct_tomography.launch \
  scene:=Building pcd_file:=my_building.pcd

roslaunch legbot_bringup pct_plan.launch \
  scene:=Building tomogram:=my_building \
  start_x:=-5.5 start_y:=6.0 start_z:=0.5 \
  goal_x:=2.0 goal_y:=-3.0 goal_z:=4.5
```

`scene:=Building` selects the A1 traversability profile; it does not force the
PCD filename. Adjust `scene_building.py` for a different robot or terrain.

## FAST-LIO Navigation with an Existing PCT Map

For the included Building scene:

```bash
# 1. Gazebo sensors
roslaunch legbot_bringup simulation.launch publish_ground_truth:=false

# 2. FAST-LIO odometry
roslaunch legbot_bringup fastlio.launch

# 3. Building-only calibrated map -> odom transform
roslaunch legbot_bringup map_to_odom_static.launch \
  x:=-4.7766 y:=6.9767 z:=0.7190 pitch:=0.7850

# 4. Controller; press 2, then 6
roslaunch legbot_bringup controller.launch

# 5. Planner using FAST-LIO topics
roslaunch legbot_bringup local_planners.launch local_planner:=scan \
  odom_topic:=/fast_lio/odometry_base \
  cloud_topic:=/fast_lio/cloud_registered

# 6-8. Global planning and visualization
roslaunch legbot_bringup pct_tomography.launch scene:=Building
roslaunch legbot_bringup pct_plan.launch scene:=Building
roslaunch legbot_bringup visualization.launch
```

FAST-LIO is odometry, not localization against an old PCD. For a new run or a
new map, calibrate `T_map_odom` or add a relocalization system. Never reuse the
Building transform blindly. Full instructions:
[docs/FAST_LIO_MAPPING_CN.md](docs/FAST_LIO_MAPPING_CN.md).

## Documentation

- [Project and algorithm analysis](docs/PROJECT_ANALYSIS_CN.md)
- [FAST-LIO mapping and navigation](docs/FAST_LIO_MAPPING_CN.md)

## License

The repository's original integration code uses the
[Apache-2.0 license](https://github.com/Robot-Nav/legbot_3D_Nav/tree/main#).
Bundled upstream projects retain their own licenses; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
