# legbot_3D_Nav

[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](https://github.com/Robot-Nav/legbot_3D_Nav/tree/main#)
[![ROS](https://img.shields.io/badge/ROS-Noetic-22314E.svg)](http://wiki.ros.org/noetic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420.svg)](https://releases.ubuntu.com/20.04/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%2011-9cf.svg)](http://gazebosim.org/)

面向 Unitree A1 的跨楼层三维导航系统：集成 FAST-LIO 建图与里程计、PCT 多层
全局规划、可切换的 EGO/SCAN 局部避障，以及强化学习运动控制，构成 Gazebo
多层楼梯场景的完整导航系统。

English documentation: [README.md](README.md)

## 仿真视频

| EGO-Planner | SCAN-Planner |
| :---: | :---: |
| <video src="https://github.com/user-attachments/assets/c5f88789-fc8c-410c-974f-f1057fb6bb33" width="100%" controls muted autoplay loop></video> | <video src="https://github.com/user-attachments/assets/41fd4757-47ea-4a4e-95ac-719ddbcb9ef5" width="100%" controls muted autoplay loop></video> |

## 系统架构

```text
LiDAR + IMU -> FAST-LIO ---------------------> odom + 注册点云
                                                        |
PCD -> PCT tomogram -> 多层 A* -> /pct_path             |
                                  |                     |
                                  v                     v
                         reference_path_transform -> EGO / SCAN
                                                        |
                                                   局部 B-spline
                                                        |
                                              /cmd_vel -> A1 RL 策略
                                                        |
                                                    Gazebo A1
```

## EGO 与 SCAN 仿真指标

两种规划器均在同一组 `Building.world + building2_9.pcd` 上完成全程，避障均开启。

| 指标 | EGO | SCAN |
| --- | ---: | ---: |
| 结果 | 成功 | 成功 |
| 全程用时 | 276.826 s | 187.176 s |
| 最终 XY 误差 | 0.102 m | 0.097 m |
| 最终 Z 误差 | 0.384 m | 0.379 m |
| 最终三维误差 | 0.397 m | 0.392 m |
| 实际行程 | 81.373 m | 79.284 m |
| 最大指令速度 | 0.750 m/s | 0.500 m/s |
| 避障配置 | 膨胀占据代价开启 | `collision_check_enabled=true` |

CPU 负载和传感器时序等因素会影响运行时间。

## 运行平台与依赖

- Ubuntu 20.04、ROS Noetic、Gazebo Classic 11
- GCC/G++、CMake、Eigen3、OpenCV、PCL、Boost、Armadillo
- Python 3.8、NumPy、SciPy、CuPy、Open3D
- A1 策略推理使用 libtorch
- PCT 使用 GTSAM、OSQP

第三方组件的许可证信息见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 编译

```bash
cd ~/legbot_3D_Nav
source /opt/ros/noetic/setup.bash
catkin_make --force-cmake -j2
```

PCT 原生模块独立编译，并通过 `CATKIN_IGNORE` 与 catkin 隔离：

```bash
cd ~/legbot_3D_Nav/src/PCT_planner/planner
./build_thirdparty.sh
./build.sh
```

### PCT 环境版本

| 组件 | 版本 |
| --- | --- |
| Ubuntu / ROS | 20.04 / Noetic |
| Python | 3.8.20 |
| NumPy / SciPy | 1.24.4 / 1.10.1 |
| Open3D | 0.19.0 |
| CuPy | `cupy-cuda12x` 12.3.0 |
| CuPy 检测到的 CUDA runtime / driver | 12.2 / 12.2 |
| GTSAM / OSQP | 4.1.1 / 0.6.2（仓库内置源码） |
| CMake / GCC | 3.16.3 / 9.4.0 |

按固定版本创建 PCT 环境：

```bash
conda create -n pct-planner python=3.8.20 -y
conda activate pct-planner
python -m pip install -r ~/legbot_3D_Nav/src/PCT_planner/requirements.txt
export PCT_PYTHON="$(which python)"
```

启动前先 source ROS Noetic，保证 `rospy` 和 ROS 消息包可见。若使用不同 CUDA 主版本，
应将 CuPy 替换为官方对应 CUDA 版本的软件包。PCT ROS 包装脚本默认使用 `python3`；
已有独立环境时可直接指定：

```bash
export PCT_PYTHON=/path/to/pct-environment/bin/python
```

libtorch 路径通过 CMake 参数传入，无需修改源码：

```bash
cd ~/legbot_3D_Nav
catkin_make --force-cmake -DCMAKE_PREFIX_PATH=/path/to/libtorch -j2
```

`build/`、`devel/` 及 PCT 第三方编译目录均由以上命令在本地生成。

## 每个终端的通用环境

```bash
source /opt/ros/noetic/setup.bash
source ~/legbot_3D_Nav/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
```

## 快速启动：Gazebo 真值里程计

按以下顺序启动。

```bash
# 终端 1：Gazebo 物理仿真
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find unitree_gazebo)/models
roslaunch legbot_bringup simulation.launch gui:=true

# 终端 2：A1 控制器；按 2 站立，再按 6 进入 RL 模式
roslaunch legbot_bringup controller.launch

# 终端 3：二选一启动局部规划器
roslaunch legbot_bringup local_planners.launch local_planner:=scan
# 或：roslaunch legbot_bringup local_planners.launch local_planner:=ego

# 终端 4：生成并发布 Building tomogram
roslaunch legbot_bringup pct_tomography.launch scene:=Building

# 终端 5：规划并发布 /pct_path
roslaunch legbot_bringup pct_plan.launch scene:=Building

# 终端 6：独立导航 RViz
roslaunch legbot_bringup visualization.launch
```

不要同时启动 EGO 和 SCAN：两者最终都会向 `/cmd_vel` 输出命令。

## 局部规划器参数对比

| 参数 | EGO | SCAN |
| --- | ---: | ---: |
| 规划视距 | 2.5 m | 2.0 m |
| 最大速度 | 0.75 m/s | 0.50 m/s |
| 最大加速度 | 0.50 m/s² | 0.30 m/s² |
| 碰撞安全距离/优化距离 | 0.35 m | 0.20 m |
| 机身模型/自体点处理 | 椭球 0.25/0.25/0.15 m | 双圆柱半径 0.15 m |
| 全局参考模式 | PCT 路径接口 | 默认分段线性 |
| 路径进度高度权重 | — | 8.0 |

SCAN 显式参数：

```bash
roslaunch legbot_bringup local_planners.launch local_planner:=scan \
  planning_horizon:=2.0 reference_mode:=piecewise_linear \
  z_projection_weight:=8.0 enable_collision_check:=true
```

EGO 显式参数：

```bash
roslaunch legbot_bringup local_planners.launch local_planner:=ego \
  ego_planning_horizon:=2.5 ego_collision_clearance:=0.35 \
  ego_robot_clearance_x:=0.25 ego_robot_clearance_y:=0.25 \
  ego_robot_clearance_z:=0.15
```

### SCAN RViz 话题

| 话题 | 含义 |
| --- | --- |
| `/local_planner/goal` | 当前 PCT/参考路径分段航点 |
| `/scan/local_target` | 经过碰撞检查的动态局部前视点 |
| `/scan/heading_vector` | 指向动态前视点的水平航向 |
| `/local_planner/optimal_path` | 当前优化局部路径 |
| `/local_planner/grid_map/occupancy_inflate` | 膨胀局部障碍物 |

## FAST-LIO：先自行建图

新地图采集期间不要启动 PCT 导航。先手动遍历环境，完整覆盖楼层、楼梯和平台，正常
关闭 FAST-LIO 后再处理 PCD。

```bash
# 终端 1：传感器仿真，不发布真值导航里程计
roslaunch legbot_bringup simulation.launch publish_ground_truth:=false

# 终端 2：在 FAST-LIO odom 坐标系累积为单个 PCD
roslaunch legbot_bringup fastlio.launch save_pcd:=true pcd_save_interval:=-1

# 终端 3：控制器；按 2，再按 6
roslaunch legbot_bringup controller.launch

# 终端 4：观察里程计和点云覆盖
roslaunch legbot_bringup visualization.launch
```

使用手柄或保守的 `/cmd_vel` 指令控制机器人遍历环境。确认以下话题正常：

```bash
rostopic hz /fast_lio/odometry_base
rostopic hz /fast_lio/cloud_registered
rostopic echo -n 1 /fast_lio/odometry_base
```

覆盖完成后，最后在 FAST-LIO 终端按 Ctrl-C。正常退出会生成：

```text
src/FAST_LIO/PCD/scans.pcd
```

将点云裁剪、去除离群点并降采样后放入 PCT：

```bash
python3 src/PCT_planner/tomography/scripts/downsample_pcd.py \
  src/FAST_LIO/PCD/scans.pcd \
  src/PCT_planner/src/pcd/my_building.pcd \
  --voxel-size 0.05 --remove-outliers
```

可选参数 `--min-bound x,y,z --max-bound x,y,z` 用于裁掉无关区域。处理后必须检查
地面、楼梯踏面、平台和上下层连接仍完整。

无需修改源码即可生成自建地图的 tomogram 并设置起终点：

```bash
roslaunch legbot_bringup pct_tomography.launch \
  scene:=Building pcd_file:=my_building.pcd

roslaunch legbot_bringup pct_plan.launch \
  scene:=Building tomogram:=my_building \
  start_x:=-5.5 start_y:=6.0 start_z:=0.5 \
  goal_x:=2.0 goal_y:=-3.0 goal_z:=4.5
```

`scene:=Building` 在这里选择的是 A1 可通行性参数模板，不再强制 PCD 文件名。
不同机器人或地形应调整 `scene_building.py` 中的坡度、台阶、净空和膨胀参数。

## FAST-LIO：使用已有 PCT 地图导航

仓库内置 Building 场景按以下顺序启动：

```bash
# 1. Gazebo 与传感器
roslaunch legbot_bringup simulation.launch publish_ground_truth:=false

# 2. FAST-LIO 在线里程计
roslaunch legbot_bringup fastlio.launch

# 3. 仅适用于当前 Building 配对的 map -> odom 标定
roslaunch legbot_bringup map_to_odom_static.launch \
  x:=-4.7766 y:=6.9767 z:=0.7190 pitch:=0.7850

# 4. 控制器；按 2，再按 6
roslaunch legbot_bringup controller.launch

# 5. 局部规划器改用 FAST-LIO 话题
roslaunch legbot_bringup local_planners.launch local_planner:=scan \
  odom_topic:=/fast_lio/odometry_base \
  cloud_topic:=/fast_lio/cloud_registered

# 6-8. PCT 与独立 RViz
roslaunch legbot_bringup pct_tomography.launch scene:=Building
roslaunch legbot_bringup pct_plan.launch scene:=Building
roslaunch legbot_bringup visualization.launch
```

FAST-LIO 是里程计，不会自动在旧 PCD 中重定位。更换地图、出生点或重新启动后，必须
重新标定 `T_map_odom`，或接入重定位模块；不能直接复用 Building 的静态变换。
完整说明见 [docs/FAST_LIO_MAPPING_CN.md](docs/FAST_LIO_MAPPING_CN.md)。

## 文档

- [项目与算法分析](docs/PROJECT_ANALYSIS_CN.md)
- [FAST-LIO 建图和导航](docs/FAST_LIO_MAPPING_CN.md)

## 许可证

项目原创集成代码统一使用
[Apache-2.0 license](https://github.com/Robot-Nav/legbot_3D_Nav/tree/main#)。
上游子项目保留各自许可证，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
