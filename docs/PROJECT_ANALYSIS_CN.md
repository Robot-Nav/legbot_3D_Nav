# Legbot 3D Navigation 项目分析

相关操作文档：

- [FAST-LIO 建图、PCT 地图生成与导航](FAST_LIO_MAPPING_CN.md)

## 1. 项目概述

Legbot 3D Navigation 将 Unitree A1 四足机器人、Gazebo 仿真、三维激光里程计、
PCT 跨楼层全局规划、EGO/SCAN 局部轨迹规划和强化学习底层控制器整合在同一
ROS Noetic 工作空间中。系统以 Building 场景为已验证目标，完成从点云地图、
全局跨层路径到四足机器人底层速度指令的完整链路。

系统最终控制对象是 Unitree A1 的 RL policy。该 policy 的输入是 A1 本体状态与
`/cmd_vel` 中的二维速度指令，输出 12 个关节目标位置。局部规划器产生的三维
轨迹不能直接发送给 RL policy，因此项目中增加 `scan_a1_cmd_adapter`，将三维
B-spline 的水平投影转换为 A1 可执行的 `x/y/yaw` 速度命令。

## 2. 系统架构

```text
PCD 点云
  -> PCT tomogram
  -> PCT 多层 A*
  -> /pct_path
  -> reference_path_transform
  -> EGO / SCAN 局部规划器
  -> 局部 B-spline
  -> scan_a1_cmd_adapter / EGO cmd_vel
  -> /cmd_vel
  -> A1 RL controller
  -> Gazebo A1
```

里程计有两条路径：

1. Gazebo 真值里程计：`/Odometry_gazebo`，由 `state_from_gazebo` 发布。
2. FAST-LIO：`/fast_lio/odometry_base`，由 LiDAR + IMU 融合得到。

PCT 输出的 `/pct_path` 位于 `map` 坐标系。局部规划器使用 `odom` 坐标系时，
必须存在 `map -> odom` 变换。

## 3. 关键算法

### 3.1 PCT 全局规划

PCT 将三维点云切分为多个水平层，每个层生成二维可通行代价图。体素的可通行性
由地形坡度、台阶高度和局部净空决定。设体素代价为 `c_i`，不可通行阈值记为
`c_free`，则 A* 只允许：

```text
c_i <= c_free
```

多层 A* 在层内使用 8 邻域，跨层时通过楼梯、坡道或楼层切换 gateway 连接相邻
层。传统 A* 代价函数为：

```text
f(n) = g(n) + h(n)
```

其中 `g(n)` 是从起点到节点 `n` 的实际代价，`h(n)` 是启发函数。PCT 的层内
搜索在 `g(n)` 中加入地形代价：

```text
g(n) = g(parent) + step_cost_weight * cell_cost(n) + distance(parent, n)
```

项目在 ROS 集成中增加中线偏置：

```text
cost(n) = original_cost(n) + clearance_cost(n)
```

`clearance_cost(n)` 由距离变换得到，使路径优先选择离不可通行区域较远的路线，
同时保留楼梯等必要通道。

### 3.2 SCAN 局部规划

SCAN 使用三维滑动占据栅格地图。栅格占据概率采用 log-odds 更新：

```text
L(m_i | z_{1:t}) = L(m_i | z_{1:t-1}) + L(m_i | z_t)
```

其中：

```text
L(x) = log(x / (1 - x))
```

单次观测命中与未命中分别用：

```text
l_hit  = log(p_hit / (1 - p_hit))
l_miss = log(p_miss / (1 - p_miss))
```

碰撞模型使用双圆柱膨胀。设前圆柱中心与后圆柱中心分别为：

```text
p_front = p + offset * heading
p_rear  = p - offset * heading
```

其中 `offset` 是 `double_cylinder_offset`，`heading` 由轨迹切向估计。

局部轨迹采用三次 B-spline。控制点记为 `Q`，位置样条为：

```text
p(t) = sum_i N_i(t) Q_i
```

优化目标包含平滑项、碰撞项、可行项和拟合项：

```text
J = lambda_smooth * J_smooth
  + lambda_collision * J_collision
  + lambda_feasibility * J_feasibility
  + lambda_fitness * J_fitness
```

平滑项通常用加速度或 jerk 的平方积分：

```text
J_smooth = integral ||a(t)||^2 dt
```

碰撞项由障碍物距离场或占据栅格沿样条采样得到。SCAN 使用 A* 在滑动地图中生成
初始无碰路径，再对 B-spline 控制点进行重绑定优化。若优化后速度或加速度超过
限制，则重新分配时间。

动态可行性检查使用：

```text
||v(t)|| <= v_max + vel_tolerance
||a(t)|| <= a_max + acc_tolerance
```

#### 3.2.1 SCAN + A1 适配

A1 RL policy 只接收：

```text
u = [vx, vy, omega_z]
```

`scan_a1_cmd_adapter` 在当前 B-spline 上寻找水平最近点：

```text
t_near = argmin_t ||p_xy(t) - odom_xy||
```

再取 lookahead：

```text
t_look = min(t_near + time_forward, T)
```

期望 yaw 由轨迹切向或速度方向给出：

```text
yaw_des = atan2(p_y(t_look)-p_y(t_near), p_x(t_look)-p_x(t_near))
```

世界系速度由前馈加位置反馈组成：

```text
v_world = v_ff + kp_pos * (p_xy(t_look) - odom_xy)
```

最后旋转到 A1 body frame：

```text
v_body_x =  cos(yaw) * v_world_x + sin(yaw) * v_world_y
v_body_y = -sin(yaw) * v_world_x + cos(yaw) * v_world_y
```

### 3.3 EGO 局部规划

EGO 同样使用 B-spline 轨迹优化。不同点在于 EGO 使用 ESDF 或膨胀占据栅格计算
碰撞梯度。优化目标可写为：

```text
min_Q lambda_s * J_s + lambda_c * J_c + lambda_f * J_f
```

其中 `J_c` 是碰撞惩罚：

```text
J_c = sum_i c(p_i)
```

`c(p_i)` 是采样点处的障碍代价。EGO 通过梯度下降或 L-BFGS 求解控制点 `Q`。
项目在 A1 场景中将 EGO 的 `planning_horizon`、`max_vel` 和 `max_acc` 设置为
地面机器人可执行范围，避免轨迹优化直接使用四旋翼参数。

工程集成还在优化器前增加有限值和起点占据预检。点云与 odom 先转换到统一地图
坐标系，仅清除 A1 机身包络内部的自体返回；确定性初值失败后允许一次随机中间点
恢复。这样可以阻断“起点误占据 -> 无效初值 -> L-BFGS -1008 -> 急停循环”，但不会
掩盖真正无解的局部环境。

### 3.4 A1 RL 控制器

A1 RL policy 的观测包括：

```text
o = [angular_velocity, projected_gravity, command, joint_position, joint_velocity, action_history]
```

其中 command 为：

```text
command = [vx, vy, omega_z]
```

网络输出 12 个关节目标位置偏移，最终关节目标为：

```text
q_target = q_default + action_scale * a
```

项目使用 `policy_act_inference_stair.pt` 作为楼梯场景模型。

## 4. 项目内容

### 4.1 主要目录

```text
src/PCT_planner/                 PCT 全局规划器、tomography 脚本和 PCD 资源
src/planner/                     EGO 局部规划器及其 plan_env/bspline_opt 等模块
src/SCAN-Planner/                SCAN 局部规划器、滑动地图和 A1 适配器
src/unitree_guide/               Unitree A1 控制器、Gazebo 插件和 RL policy
src/legbot_bringup/              场景 launch、TF 适配、Rviz 配置
src/FAST_LIO/                    FAST-LIO 里程计
src/Mid360_imu_sim/              Livox Mid-360 仿真传感器
```

### 4.2 关键节点

| 节点 | 作用 |
| --- | --- |
| `state_from_gazebo` | 发布 Gazebo 真值 `odom -> base` 和 `/Odometry_gazebo` |
| `fastlio_odom_adapter` | 将 FAST-LIO IMU 位姿转为 `odom -> base` |
| `reference_path_transform` | 将 `/pct_path` 变换到局部规划器所需坐标系 |
| `scan_planner_node` | SCAN 滑动地图、局部 B-spline 规划 |
| `scan_a1_cmd_adapter` | SCAN 三维轨迹到 A1 `/cmd_vel` |
| `ego_planner_node` | EGO 局部规划 |
| `junior_ctrl` | Unitree A1 控制器和 RL policy 推理 |

## 5. 运行步骤

### 5.1 Gazebo 真值里程计模式

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source ~/legbot_3D_Nav/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find unitree_gazebo)/models
roslaunch legbot_bringup simulation.launch
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source ~/legbot_3D_Nav/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch legbot_bringup controller.launch
```

控制器终端按 `2` 站立，按 `6` 进入 RL 模式。

终端 3：

```bash
source /opt/ros/noetic/setup.bash
source ~/legbot_3D_Nav/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch legbot_bringup local_planners.launch local_planner:=scan
```

终端 4：

```bash
source /opt/ros/noetic/setup.bash
source ~/legbot_3D_Nav/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch legbot_bringup pct_tomography.launch scene:=Building
```

终端 5：

```bash
source /opt/ros/noetic/setup.bash
source ~/legbot_3D_Nav/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch legbot_bringup pct_plan.launch scene:=Building
```

终端 6：

```bash
roslaunch legbot_bringup visualization.launch
```

### 5.2 FAST-LIO 模式

FAST-LIO 模式分为“先建图”和“使用已有 PCT 地图导航”。建图时只启动仿真、FAST-LIO、
控制器和 RViz，使用 `save_pcd:=true` 正常退出后得到 `src/FAST_LIO/PCD/scans.pcd`；
完成点云清理和 PCT tomogram 生成后，才能进入导航。

终端 1：

```bash
roslaunch legbot_bringup simulation.launch publish_ground_truth:=false
```

终端 2：

```bash
roslaunch legbot_bringup fastlio.launch
```

终端 3：

```bash
roslaunch legbot_bringup map_to_odom_static.launch \
  x:=-4.7766 y:=6.9767 z:=0.7190 pitch:=0.7850
```

后续控制器、局部规划器和 PCT 步骤与真值模式一致，但局部规划器需要传入
FAST-LIO 话题：

```bash
roslaunch legbot_bringup local_planners.launch local_planner:=scan \
  odom_topic:=/fast_lio/odometry_base \
  cloud_topic:=/fast_lio/cloud_registered
```

独立 RViz 启动命令在两种模式下相同：

```bash
roslaunch legbot_bringup visualization.launch
```

FAST-LIO 不会自动在旧 PCD 中重定位；上面的静态变换只适用于当前 Building 配对。
自建地图、点云处理、tomogram 参数化启动和 TF 标定的完整步骤见
[FAST_LIO_MAPPING_CN.md](FAST_LIO_MAPPING_CN.md)。

### 5.3 Gazebo 仿真性能对比

两种规划器均在启用避障的情况下完成同一条 667 点 PCT 路径：

| 规划器 | 用时 | XY / Z / 3D 误差 | 行程 | 结果 |
| --- | ---: | --- | ---: | --- |
| EGO | 276.826 s | 0.102 / 0.384 / 0.397 m | 81.373 m | 成功 |
| SCAN | 187.176 s | 0.097 / 0.379 / 0.392 m | 79.284 m | 成功，碰撞检查开启 |

该表用于展示当前 Gazebo 场景下的运行表现，运行时间会受硬件负载和传感器时序影响。

### 5.4 关键工程整改

| 模块 | 当前工程行为 |
| --- | --- |
| EGO 地图 | 点云和 odom 统一到地图 frame，只清除 A1 机身椭球内部自体点 |
| EGO 优化 | L-BFGS 前检查有限值和起点占据，坏初值提前返回并随机重试一次 |
| EGO 安全 | 规划失败停车，odom/轨迹断流触发零速看门狗，连续失败成功后清零 |
| SCAN 进度 | 带 Z 权重的单调全局投影，局部目标按水平弧长推进 |
| SCAN 楼梯 | 自动对齐参考起点高度，保持楼梯最低沿轨速度，XY+Z 双阈值到站 |
| SCAN 避障 | collision_check_enabled 默认开启，保留滑动地图、A* 和碰撞优化 |
| SCAN 可视化 | 分离参考航点、实时 local target 和水平 heading vector |

## 6. 依赖库

### 6.1 系统依赖

```text
Ubuntu 20.04
ROS Noetic
CMake
GCC / G++
Eigen3
Armadillo
OpenCV
PCL
Boost
Gazebo Classic 11
```

### 6.2 Python 环境

PCT 已验证的固定版本如下：

| 组件 | 版本 |
| --- | --- |
| Python | 3.8.20 |
| NumPy | 1.24.4 |
| SciPy | 1.10.1 |
| Open3D | 0.19.0 |
| CuPy | cupy-cuda12x 12.3.0 |
| CUDA runtime / driver | 12.2 / 12.2 |
| rospkg | 1.6.1 |

```bash
conda create -n pct-planner python=3.8.20 -y
conda activate pct-planner
python -m pip install -r src/PCT_planner/requirements.txt
export PCT_PYTHON="$(which python)"
```

`rospy`、ROS 消息和 `catkin_pkg` 由 ROS Noetic 系统环境提供，启动 PCT 前需要先
`source /opt/ros/noetic/setup.bash`。

### 6.3 第三方算法库

```text
PCT: GTSAM 4.1.1, OSQP 0.6.2
EGO/SCAN: Eigen, OpenCV, PCL
A1 RL: libtorch
FAST-LIO: ikd-Tree
```

## 7. 关键参数

### 7.1 SCAN

```text
planning_horizon = 2.0
max_vel          = 0.5
max_acc          = 0.3
reference_mode   = piecewise_linear
z_projection_weight = 8.0
enable_collision_check = true
```

SCAN 默认使用滑动占据地图、双圆柱模型和 B-spline 碰撞代价避障。`false` 只用于隔离
地图问题，不是 Building 楼梯场景的正常配置。

SCAN 的 RViz 目标语义已明确拆分：`/local_planner/goal` 是当前参考路径点，
`/scan/local_target` 是实时规划前视点，`/scan/heading_vector` 是机器人指向前视点的
水平期望航向。前视点保留三维高度，航向箭头只表达 yaw。

### 7.2 EGO

```text
planning_horizon = 2.5
max_vel          = 0.75
max_acc          = 0.5
collision_clearance = 0.35
robot_clearance_x/y/z = 0.25 / 0.25 / 0.15
```

EGO 点云写图前统一转换到地图坐标系，并清除 A1 机身包络内部的自体/噪声返回。
该清除区不延伸到机身外，不等同于关闭碰撞检查；楼梯立面和外部障碍仍保留在占据图中。

## 8. 常见问题

### 8.1 局部规划器显示 no odom

确认 Gazebo 已启动，`map -> odom` 存在，且 `/Odometry_gazebo` 或
`/fast_lio/odometry_base` 有输出。

### 8.2 SCAN 在楼梯前停止或反复 emergency stop

保持 `enable_collision_check=true`，检查 PCT 路径 z 与机器人 odom z 是否采用同一参考高度，
再检查点云 TF、地面高度和 z 膨胀。不要通过关闭避障掩盖高度标定问题。

### 8.3 三楼时 SCAN 回到二楼全局路径

调大 `z_projection_weight`：

```bash
roslaunch legbot_bringup local_planners.launch local_planner:=scan \
  z_projection_weight:=12.0
```

### 8.4 局部目标几乎在 A1 正上方

`getLocalTarget` 按水平距离推进局部目标，避免楼梯段三维长度导致 x/y 位移过小。
该问题已修复。

## 9. 许可证与来源

项目原创集成代码统一使用
[Apache-2.0 license](https://github.com/Robot-Nav/legbot_3D_Nav/tree/main#)。仓库内保留
各上游子模块原有许可证和版权声明，详细清单见根目录 `THIRD_PARTY_NOTICES.md`。

主要上游项目：

```text
PCT planner
EGO-Planner
SCAN-Planner
Unitree A1 guide
FAST-LIO
```
