# Legbot 3D Navigation ROS 2 项目技术分析

[返回中文 README](../README_CN.md) | [English README](../README.md) | [运行指南](../运行指南.md)

## 1. 项目定位

Legbot 是面向 Unitree GO2/GO2 EDU 的三维自主导航与运动控制工作区，目标不是单独实现某一种 SLAM 或规划算法，而是把感知、定位、规划、步态控制、仿真和实物接口连接成可检查、可替换的 ROS 2 系统。

项目重点解决以下工程问题：

- 使用机载 LiDAR 与 IMU 建立不依赖外部定位的机器人状态；
- 在三维点云环境中维护局部碰撞信息，连续滚动规划；
- 支持平地 RViz 目标、目标队列和带高度的多楼层路线；
- 将规划速度转换为 GO2 强化学习策略能够跟踪的机体命令；
- 让同一套控制器能够在 Gazebo 与 Unitree SDK2 硬件接口之间切换；
- 将真机执行与只读预检分开，默认不向电机发送指令。

项目当前更适合作为 GO2 三维导航的研发与验证平台。Gazebo 闭环已有测试记录，真机电机闭环仍需要在具体机器人上完成网络、标定、急停和低速验收。

## 2. 系统分层与数据流

```text
传感器层       Unitree L1 / Livox / Gazebo LiDAR + IMU
                                  │
定位层                            ▼
                              FAST-LIO
                       ┌──────────┴──────────┐
                       │                     │
              /fast_lio/odometry_base   注册点云
                       │                     │
任务层                 │      RViz 目标 / queue / 3D 路线 / PCT
                       │                     │
规划层                 └────────► SCAN ◄────┘
                                      │
                                局部 B 样条轨迹
                                      │
命令层                          GO2 命令适配器
                                      │ 机体系 vx, vy, wz
控制层                      RL 策略 + 关节 PD 控制
                                      │ 12 关节命令
执行层                 Gazebo ros2_control / Unitree SDK2
```

默认仿真链路中，`publish_ground_truth=false`，规划与控制使用 FAST-LIO 结果。Gazebo 真值只在显式诊断模式下参与误差统计，避免形成“看起来闭环、实际绕过定位”的测试结果。

### 2.1 主要话题

| 数据 | 默认话题或接口 | 作用 |
| --- | --- | --- |
| LiDAR | `/unitree/lidar` 或仿真点云 | FAST-LIO 原始量测 |
| LiDAR 内置 IMU | `/unitree/lidar_imu` | 点云去畸变和状态传播 |
| 机身 IMU | `/imu_sensor_broadcaster/imu` | 备用惯性输入、姿态与控制状态 |
| FAST-LIO 里程计 | `/fast_lio/odometry_base` | SCAN、任务状态机和控制输入 |
| 注册点云 | 由 FAST-LIO launch 配置 | SCAN 占据图输入 |
| RViz 目标 | `2D Goal Pose` 对应目标话题 | direct/queue 任务入口 |
| SCAN 轨迹 | B 样条消息与可视化话题 | 命令适配和 RViz 显示 |
| 导航速度 | GO2 控制输入接口 | RL 策略的速度命令 |
| 关节状态 | `/joint_states` | 控制观测与只读预检 |

具体话题可由 launch 参数覆盖；仿真、真值诊断和真机链路必须成套切换，不能只替换其中一个节点的数据源。

## 3. 软件包职责

| 软件包 | 关键内容 |
| --- | --- |
| `control_input_msgs` | 控制模式和速度输入消息 |
| `controller_common` | ros2_control 控制器公共接口 |
| `keyboard_input` | 键盘模式与速度输入 |
| `go2_description` | URDF、网格、ros2_control、策略配置与权重 |
| `gz_quadruped_hardware` | Gazebo Harmonic 四足硬件系统插件 |
| `hardware_unitree_sdk2` | Unitree SDK2 硬件接口、模式管理、L1 DDS 桥 |
| `rl_quadruped_controller` | 固定站立/RL 状态机、观测历史、推理和关节命令 |
| `fast_lio` | LiDAR-IMU 紧耦合里程计和增量地图 |
| `scan_planner` | 滑动占据图、A*、B 样条优化、滚动重规划 |
| `pct_planner` | 多层层析地图、全局路径和 GPMP 优化 |
| `ego_planner` | EGO 搜索与局部轨迹优化入口 |
| `legbot_runtime` | ROS 2 时间、参数、消息与变换兼容层 |
| `legbot_bringup` | 统一 launch、适配节点、世界和 RViz 配置 |

## 4. 定位：FAST-LIO

### 4.1 状态与 IMU 传播

FAST-LIO 使用迭代误差状态卡尔曼滤波。可将名义状态概括为

$$
\mathbf{x}=\left(\mathbf{R},\mathbf{p},\mathbf{v},
\mathbf{b}_g,\mathbf{b}_a,\mathbf{g},
\mathbf{R}_{LI},\mathbf{p}_{LI}\right),
$$

其中 $\mathbf{R},\mathbf{p},\mathbf{v}$ 是 IMU 在世界系中的姿态、位置和速度，$\mathbf{b}_g,\mathbf{b}_a$ 是陀螺仪与加速度计偏置，$\mathbf{R}_{LI},\mathbf{p}_{LI}$ 是 LiDAR 到 IMU 的外参。

去偏后的 IMU 传播模型为

$$
\dot{\mathbf{R}}=\mathbf{R}[\boldsymbol\omega_m-\mathbf{b}_g-\mathbf{n}_g]_\times,
$$

$$
\dot{\mathbf{v}}=\mathbf{R}(\mathbf{a}_m-\mathbf{b}_a-\mathbf{n}_a)+\mathbf{g},
\qquad
\dot{\mathbf{p}}=\mathbf{v}.
$$

逐点时间字段用于把扫描内各点补偿到统一时刻。Unitree L1 桥保留 `time` 与 `ring` 字段，并对点云和内置 IMU 使用同一个主机时钟偏移，以保留两者的相对时间关系。

### 4.2 点到平面更新

LiDAR 点 $\mathbf{p}_j^L$ 经外参和当前状态变换到世界系：

$$
\mathbf{p}_j^W=\mathbf{R}\left(\mathbf{R}_{LI}\mathbf{p}_j^L+\mathbf{p}_{LI}\right)+\mathbf{p}.
$$

从局部地图邻域拟合平面 $(\mathbf{n}_j,\mathbf{q}_j)$，量测残差为

$$
r_j=\mathbf{n}_j^\top(\mathbf{p}_j^W-\mathbf{q}_j).
$$

滤波器对残差反复线性化并更新误差状态，收敛后把新点加入 ikd-Tree。相比“先配准、再融合”的松耦合方式，点到平面约束直接进入状态估计，对快速运动和非重复扫描 LiDAR 更合适。

### 4.3 工程约束

- IMU 初始化要求机器人连续静止；下蹲、站起或明显晃动会重置初始化窗口。
- 外参方向必须与 FAST-LIO 配置定义一致。
- 缺少逐点时间会直接影响运动去畸变。
- 长走廊、平面退化和错误时间同步仍会造成漂移；项目提供真值对比模式用于量化，而不是掩盖误差。

## 5. 环境表示：滑动概率占据图

SCAN 使用分辨率为 $r$ 的局部三维栅格。对每条激光束执行射线遍历：终点作为命中，射线路径作为空闲观测。若体素占据概率为 $p_t$，其 log-odds 为

$$
l_t=\log\frac{p_t}{1-p_t}.
$$

一次量测后的递推形式为

$$
l_t=\operatorname{clip}\left(l_{t-1}+l(z_t)-l_0, l_{min},l_{max}\right).
$$

当前配置中的主要参数为：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `resolution` | 0.10 m | 栅格分辨率 |
| `sliding_map_size_x/y/z` | 10/10/5 m | 局部地图范围 |
| `p_hit` / `p_miss` | 0.85 / 0.30 | 命中与穿过更新概率 |
| `p_min` / `p_max` | 0.12 / 0.98 | 概率截断 |
| `p_occ` | 0.80 | 占据判定阈值 |
| `max_ray_length` | 5.0 m | 最大射线长度 |
| `lidar_horizontal_fov_deg` | 240° | SCAN 使用的前向视场 |

FAST-LIO 仍使用 360° 点云定位，240° 限制只作用于局部避障。该设置不为持续倒退提供后方防撞保护。碰撞模型使用前后双圆柱，当前半径 0.20 m、前后偏移 0.07 m；PCT 负责地形可通行性，SCAN 负责局部占据避障。

## 6. SCAN 局部规划

### 6.1 A* 搜索

初始轨迹与障碍相交时，规划器截取碰撞段并用 A* 寻找绕行方向。基本评价函数为

$$
f(n)=g(n)+h(n),
$$

其中累计代价按相邻栅格的欧氏长度更新：

$$
g(n')=g(n)+\sqrt{\Delta x^2+\Delta y^2+\Delta z^2}.
$$

代码使用带微小 tie-breaker 的对角距离启发项，使代价相同的节点更倾向朝目标展开。GO2 路径搜索以 XY 邻域扩展，并沿起终点构成的搜索平面插值 Z；这避免平地局部规划无意义地上下搜索，同时保留跨层参考路径的高度趋势。单次搜索设有 0.2 s 上限。

### 6.2 均匀 B 样条

轨迹由 $p$ 阶 B 样条表示：

$$
\mathbf{c}(u)=\sum_{i=0}^{n}N_{i,p}(u)\mathbf{P}_i,
$$

$\mathbf{P}_i$ 为控制点，$N_{i,p}$ 为基函数。代码使用 de Boor 算法求值。导数仍是 B 样条，其控制点满足

$$
\mathbf{P}^{(1)}_i=
\frac{p(\mathbf{P}_{i+1}-\mathbf{P}_i)}{u_{i+p+1}-u_{i+1}}.
$$

这使速度、加速度约束可以直接在控制点差分上检查。若轨迹超出物理限制，规划器按

$$
\rho=\max\left(\frac{v_{max,observed}}{v_{limit}},
\sqrt{\frac{a_{max,observed}}{a_{limit}}}\right)
$$

放大时间跨度。

### 6.3 优化目标

控制点优化的总体形式为

$$
J=\lambda_sJ_s+\lambda_cJ_c+\lambda_fJ_f+\lambda_rJ_r.
$$

- $J_s$：控制点高阶差分的平滑代价，抑制曲率和加加速度突变；
- $J_c$：控制点到障碍边界小于安全距离时的惩罚；
- $J_f$：速度或加速度超限惩罚；
- $J_r$：精修阶段对初始/参考轨迹的拟合代价。

反弹阶段实际组合 $J_s+J_c+J_f$，精修阶段组合 $J_s+J_r+J_f$。当前配置权重为

$$
\lambda_s=1.0,\quad \lambda_c=1.0,\quad
\lambda_f=0.1,\quad \lambda_r=1.0,
$$

规划速度上限 0.50 m/s、加速度上限 0.30 m/s²、局部前视距离 3.0 m、控制点间距 0.2 m、优化安全距离 0.25 m。规划器检查完整局部轨迹，不忽略末段。

### 6.4 滚动重规划与任务模式

机器人沿轨迹前进约 0.2 m 后即可触发常规重规划。规划失败时先停车，再按受限频率重试；不会因一次“起点占据”直接丢弃任务。

- `direct`：新 RViz 目标立即替换当前目标，适合交互调试；
- `queue`：按顺序执行多个 XY 航点；
- `navi_mode=2`：加载带 XYZ 的关键点路线，适合多楼层任务；
- `Publish Point`：删除邻近未完成点，并支持撤销和清空服务。

普通 RViz 2D 目标按 XY 判断到达，Z 随当前定位高度平滑调整；显式三维路线才保留楼层高度语义。

## 7. PCT 与 EGO

### 7.1 PCT 多楼层规划

PCT 把点云转换为多个高度切片组成的 tomogram，在每个切片计算地形代价和障碍膨胀，再在多层图上搜索全局参考路径。其原生模块使用 Eigen、GTSAM、METIS、OSQP 和 pybind11；Python 层使用 NumPy/SciPy，层析图生成还依赖 Open3D/CuPy。

路径搜索后可用 GPMP 风格因子图优化。抽象目标可写为

$$
\mathbf{x}^*=\arg\min_{\mathbf{x}}
\left(
\|\mathbf{x}-\boldsymbol\mu\|_{K^{-1}}^2+
\sum_k\|h_k(\mathbf{x})\|_{\Sigma_k^{-1}}^2
\right),
$$

第一项是连续时间运动先验，第二项包含障碍、插值障碍、航向变化等因子。PCT 在本项目中主要提供全局参考，局部碰撞与实时滚动仍由 SCAN 处理。

### 7.2 EGO 入口

EGO 同样包含栅格搜索、B 样条优化和轨迹服务器，可作为独立局部规划方案。当前主运行链以 SCAN 为默认，EGO 保留为可切换入口；两套规划器不应同时向同一个底层控制入口发送命令。

## 8. 速度适配与 GO2 控制

### 8.1 轨迹跟踪

命令适配器在前视时刻 $t+t_f$ 采样 B 样条，将位置误差旋转到机体系，使用比例项生成平移命令并裁剪：

$$
\mathbf{v}_{cmd}=\operatorname{sat}
\left(\mathbf{v}_{ref}+K_p\mathbf{e}_{body},\ \mathbf{v}_{max}\right).
$$

期望航向取路径切向，角速度为

$$
\omega_{cmd}=\operatorname{sat}
\left(K_\psi\operatorname{wrap}(\psi_d-\psi),\ \omega_{max}\right).
$$

较大转角会进入原地转向区间；普通弯道按角度连续降速，接近目标时按剩余距离降速。当前主要限制为 `max_vx=0.50 m/s`、`max_vy=0.22 m/s`、`max_vyaw=1.00 rad/s`，线加速度限制 0.75 m/s²。

### 8.2 强化学习策略观测

默认每帧观测为

$$
\mathbf{o}_t=
[s_\omega\boldsymbol\omega_t,
\mathbf{R}_t^\top\mathbf{g},
\mathbf{s}_c\odot\mathbf{c}_t,
s_q(\mathbf{q}_t-\mathbf{q}_0),
s_{\dot q}\dot{\mathbf{q}}_t,
\mathbf{a}_{t-1}],
$$

维度为 $3+3+3+12+12+12=45$。`moe_cts_77k` 保存 10 帧历史，并按 observation-term-major 排列：先放 10 帧角速度，再放 10 帧重力方向，依次类推，因此模型输入为 $1\times450$，不是简单的逐帧拼接顺序。

### 8.3 动作与关节控制

策略 $\pi_\theta$ 输出 12 维动作：

$$
\mathbf{a}_t=\pi_\theta(\mathbf{o}_{t-H+1:t}),
\qquad
\mathbf{q}_d=\mathbf{q}_0+s_a\mathbf{a}_t.
$$

关节命令采用 PD 形式：

$$
\boldsymbol\tau=
\mathbf{K}_p(\mathbf{q}_d-\mathbf{q})-
\mathbf{K}_d\dot{\mathbf{q}}+\boldsymbol\tau_{ff},
$$

并按配置中的每关节力矩上限裁剪。默认策略使用 `action_scale=0.25`、`Kp=20`、`Kd=0.5`，推理 decimation 为 4。同步推理保证观测、动作与控制周期对应；模型输入输出维度在加载时检查。

### 8.4 策略契约

| 策略 | 后端 | 输入 | 动作缩放 | 说明 |
| --- | --- | ---: | ---: | --- |
| `moe_cts_77k` | ONNX Runtime | 450 | 0.25 | 默认，10 帧 term-major |
| `go2_cts` | TorchScript | 45 | 0.25 | 平地对照，Isaac Gym 契约 |
| `mjlab_flat` | ONNX Runtime | 45 | 0.50 | Unitree Velocity Flat |

模型不能只按输入输出维度互换。观测顺序、缩放、历史布局、默认关节角、动作缩放、PD 增益和力矩限制共同构成部署契约。

## 9. 仿真与真机接口

### 9.1 Gazebo

`gz_quadruped_hardware` 把 Gazebo 关节状态和命令映射到 ros2_control。`simulation.launch.py` 负责模型、控制器、传感器桥和可选真值通道。平地、无界面、楼梯与障碍场景在 `src/legbot_bringup/worlds/` 中分开保存。

### 9.2 Unitree SDK2

`hardware_unitree_sdk2` 读取 LowState、发布关节与 IMU 状态，并在显式放行后发送 LowCmd。L1 DDS 桥通过同一根机器人控制网线接收点云和内置 IMU，使计算机端继续运行 FAST-LIO、SCAN、RViz 和策略控制器。

真机安全顺序是：网络连通 → 只读状态 → 传感器字段与时间 → 外参与静止漂移 → 吊架/急停 → 站立与零速 → 极低速直行 → 再进入导航。`real.launch.py` 默认禁用命令，不能把修改默认值当作现场验收的替代品。

## 10. 构建与运行

### 10.1 编译

```bash
cd /home/fatu08/legbot_3D_Nav_ROS2
./tools/build.sh
source tools/env.sh
./tools/check_offline.sh
```

构建脚本使用 Release、顺序编译和单任务并行，减少大型 C++ 模块同时编译造成的内存压力。`tools/env.sh` 统一设置 ROS、LibTorch、ONNX Runtime、PCT、Livox SDK2、Unitree SDK2 和 diagnostic_updater 兼容库路径。

### 10.2 最小仿真流程

按顺序在独立终端启动：

1. `simulation.launch.py`；
2. `go2_demo_control.launch.py`；
3. `fastlio.launch.py`；
4. `scan.launch.py`；
5. `scan_rviz.launch.py`；
6. `scan_waypoints.launch.py`。

完整命令和参数见[运行指南](../运行指南.md)。

### 10.3 离线检查范围

`tools/check_offline.sh` 聚合以下检查：

- 本地依赖与动态库是否存在；
- launch 文件是否可加载、默认参数是否符合安全约束；
- TorchScript/ONNX 模型文件、哈希、输入输出与观测契约；
- direct 目标替换和停止确认状态机；
- 不启动 Gazebo、不连接机器人、不发送电机命令。

## 11. 已记录的验证结果

以下数据来自仓库现有架构记录，应理解为特定配置下的回归结果，而不是对所有地形与硬件的性能保证。

| 配置 | 场景 | 结果 |
| --- | --- | --- |
| `mjlab_flat` | 平地 0.20 m/s，20 s | 前进 2.99 m，横漂 0.80 m，最大倾角 9.26° |
| `moe_cts_77k` | 平地 0.20 m/s，20 s | 前进 3.13 m，横漂 0.03 m，最大倾角 6.12° |
| `moe_cts_77k` | 八级楼梯 0.40 m/s，10 s | 高度 0.31 m 至 1.12 m，最大倾角 18.56° |
| FAST-LIO + SCAN queue | 平地三目标 | 三点依次到达，65.12 s |
| FAST-LIO + SCAN direct | 正前方 8 m、路径穿障碍 | 30.60 s 到达，11 次轨迹更新，横向绕行 1.22 m |

这些结果说明默认策略、定位和局部规划能够组成仿真闭环，也说明策略部署契约对运动质量有直接影响。

## 12. 依赖与发布边界

### 12.1 系统依赖

- ROS 2 Jazzy、ament、colcon、launch/launch_ros；
- ros2_control、controller_manager、ros2_controllers；
- ros_gz_sim、ros_gz_bridge、Gazebo Harmonic；
- Eigen3、PCL、OpenCV、yaml-cpp、Boost；
- tf2、message_filters、cv_bridge、pcl_ros/pcl_conversions；
- Python 3.12、NumPy、SciPy、Open3D/CuPy（PCT 制图功能）；
- GTSAM、METIS、OSQP、pybind11（PCT 原生模块）；
- LibTorch、ONNX Runtime、Unitree SDK2、Livox SDK2。

### 12.2 Git 中不包含的内容

`build/`、`install/`、`log/`、`.venv-jazzy/`、`.deps/`、`third_party/` 和运行数据不进入仓库。源码中包含策略权重、GO2 网格与示例 PCT 数据；发布前应继续保留模型来源和第三方许可证记录。

### 12.3 许可证

根目录 Apache-2.0 只覆盖本项目原创集成代码，不替代上游许可证。FAST-LIO 与 PCT 包含 GPL-2.0，Livox、Unitree、GO2 控制代码和模型权重各有独立条款。完整说明见 [`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md)。

## 13. 已知限制与后续工作

- 当前工作区依赖本地预装的 `third_party` 和 `.deps`，新机器需要按锁定版本准备这些运行库；
- PCT 原生依赖和 Python/CUDA 环境比主 SCAN 链复杂，运行前应执行库检查；
- 240° SCAN 视场不覆盖持续倒退安全；
- 普通 2D 目标按 XY 到达，跨楼层必须使用显式三维路线或 PCT 参考；
- 仿真传感器噪声、接触参数与真实 GO2 存在差异，策略仿真成功不等于可直接无吊架上机；
- 真机网络接口名、LiDAR/IMU 时间戳、外参和 FAST-LIO 静止漂移必须逐机标定；
- 当前没有自动化硬件在环测试，也没有声明完成真机导航验收；
- 后续可补充依赖下载脚本、CI 构建矩阵、rosbag 回归数据和统一性能评测工具。

## 14. 阅读顺序建议

首次使用按以下顺序阅读：

1. [中文 README](../README_CN.md)：确认环境、依赖和快速启动；
2. [运行指南](../运行指南.md)：按场景执行完整命令；
3. 本文：理解算法、参数和模块边界；
4. `docs/*_LOCK.json` 与 [第三方声明](../THIRD_PARTY_NOTICES.md)：确认来源、版本和分发条件。
