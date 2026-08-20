# FAST-LIO 建图、PCT 地图生成与导航

本文区分两个流程：

1. **建图**：FAST-LIO 在线累计点云，机器人由人工控制，不启动 PCT/EGO/SCAN。
2. **导航**：PCT 使用已处理的 PCD 生成全局路径，FAST-LIO 提供当前运行的里程计。

FAST-LIO 本身不是旧地图重定位系统。已有 PCD 与新一次 FAST-LIO 的 odom 原点不一定
一致，导航前必须提供正确的 map 到 odom 变换。

## 1. 建图前检查

每个终端执行：

~~~bash
source /opt/ros/noetic/setup.bash
source ~/legbot_3D_Nav/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
~~~

确认 MID-360 仿真输入：

~~~bash
rostopic info /livox/points_raw
rostopic info /livox/imu
~~~

项目配置位于：

- src/legbot_bringup/config/fastlio_mid360.yaml
- src/legbot_bringup/launch/fastlio.launch

仿真使用普通 sensor_msgs/PointCloud2，对应 lidar_type 4；不要改回需要旧版 Livox
CustomMsg 的配置。

## 2. FAST-LIO 自行建图

终端 1：

~~~bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find unitree_gazebo)/models
roslaunch legbot_bringup simulation.launch publish_ground_truth:=false
~~~

终端 2：

~~~bash
roslaunch legbot_bringup fastlio.launch \
  save_pcd:=true pcd_save_interval:=-1
~~~

-1 表示运行期间在内存中累计，节点正常退出时写为一个 scans.pcd。大场景内存不足时可
设置正整数，例如 pcd_save_interval:=1000，FAST-LIO 会写多个 scans_N.pcd；进入
PCT 前需要先合并这些分片。

终端 3：

~~~bash
roslaunch legbot_bringup controller.launch
~~~

在控制器终端按 2 站立，再按 6 进入 RL 模式。

终端 4：

~~~bash
roslaunch legbot_bringup visualization.launch
~~~

建图期间不要启动 pct_plan.launch 或局部规划器。可用手柄，也可在 Gazebo 中用低速
命令分段控制；每次转弯前先停止：

~~~bash
# 低速前进，完成这一段后 Ctrl-C
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 原地缓慢转向，完成后 Ctrl-C
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.30}}'
~~~

采图时应：

- 先覆盖起点附近，等待 IMU/LiDAR 初始化稳定；
- 每层沿不同方向重复观察，减少墙面和楼梯遮挡；
- 完整经过楼梯踏面、楼梯口、平台和终点区域；
- 避免高速旋转、长时间退行和碰撞；
- 定期确认 odom 没有明显跳变。

## 3. 建图质量检查和保存

~~~bash
rostopic hz /fast_lio/odometry_base
rostopic hz /fast_lio/cloud_registered
rostopic echo -n 1 /fast_lio/odometry_base
rosrun tf tf_echo odom base
~~~

完成后先停止运动，再最后向 FAST-LIO 终端发送 Ctrl-C。正常退出生成：

~~~text
src/FAST_LIO/PCD/scans.pcd
~~~

强制杀进程、断电或直接关闭终端可能来不及执行最终写盘。确认文件存在且非空：

~~~bash
ls -lh src/FAST_LIO/PCD/scans.pcd
pcl_viewer src/FAST_LIO/PCD/scans.pcd
~~~

## 4. 点云清理与降采样

~~~bash
cd ~/legbot_3D_Nav
python3 src/PCT_planner/tomography/scripts/downsample_pcd.py \
  src/FAST_LIO/PCD/scans.pcd \
  src/PCT_planner/src/pcd/my_building.pcd \
  --voxel-size 0.05 --remove-outliers
~~~

裁剪示例：

~~~bash
python3 src/PCT_planner/tomography/scripts/downsample_pcd.py \
  src/FAST_LIO/PCD/scans.pcd \
  src/PCT_planner/src/pcd/my_building.pcd \
  --voxel-size 0.05 --remove-outliers \
  --min-bound=-12,-12,-1 --max-bound=12,12,8
~~~

若目标已存在，明确添加 --force 才会覆盖；--visualize 会打开 Open3D 窗口。检查：

- 主要楼梯踏面应连续；
- 上下平台之间不能出现大面积断层；
- 不应把天花板或另一楼层误当作同层地面；
- 起点和终点周围应有足够可站立区域；
- 体素尺寸不应大到抹掉楼梯。

## 5. 生成 PCT tomogram

scene:=Building 表示复用 A1 的 Building 可通行性参数模板；PCD 可单独指定：

~~~bash
roslaunch legbot_bringup pct_tomography.launch \
  scene:=Building pcd_file:=my_building.pcd
~~~

成功后生成 src/PCT_planner/src/tomogram/my_building.pickle，并发布 /global_points、
/tomogram、/layer_G_* 和 /layer_C_*。通过专用 RViz 检查：

~~~bash
roslaunch legbot_bringup visualization.launch
~~~

不同机器人或地形需要在 scene_building.py 中调整 resolution、slice_dh、
slope_max、step_max、safe_margin 和 inflation 等可通行性参数。

## 6. 设置起终点并发布 PCT 路径

~~~bash
roslaunch legbot_bringup pct_plan.launch \
  scene:=Building tomogram:=my_building \
  start_x:=-5.5 start_y:=6.0 start_z:=0.5 \
  goal_x:=2.0 goal_y:=-3.0 goal_z:=4.5
~~~

~~~bash
rostopic echo -n 1 /pct_path/header
rostopic echo -n 1 /pct_path/poses
~~~

路径坐标系应为 map，起终点必须位于有效可通行层附近。RViz 交互标记仍可在运行中
调整起终点。

## 7. map 到 odom 标定

若 PCD 直接来自当前仍在运行的 FAST-LIO，且未对 PCD 做额外旋转/平移，通常可使用
身份变换：

~~~bash
roslaunch legbot_bringup map_to_odom_static.launch
~~~

重新启动 FAST-LIO、改变出生点或对 PCD 做坐标变换后，需要重新计算：

~~~text
T_map_odom = T_map_base × inverse(T_odom_base)
~~~

map_to_odom_static.launch 的参数顺序是平移 x y z 和欧拉角 yaw pitch roll。仓库中的
以下值只适用于已验证 Building 配对：

~~~bash
roslaunch legbot_bringup map_to_odom_static.launch \
  x:=-4.7766 y:=6.9767 z:=0.7190 pitch:=0.7850
~~~

~~~bash
rosrun tf tf_echo map odom
rosrun tf tf_echo map base
~~~

PCT 起点与 map 下机器人位置应接近，楼层高度语义也必须一致。静态 TF 不是重定位；
实机长期运行应接入点云地图重定位或回环定位。

## 8. FAST-LIO 导航启动顺序

~~~bash
# 1
roslaunch legbot_bringup simulation.launch publish_ground_truth:=false
# 2
roslaunch legbot_bringup fastlio.launch
# 3：使用已标定值
roslaunch legbot_bringup map_to_odom_static.launch \
  x:=-4.7766 y:=6.9767 z:=0.7190 pitch:=0.7850
# 4：按 2、6
roslaunch legbot_bringup controller.launch
# 5：EGO 或 SCAN 二选一
roslaunch legbot_bringup local_planners.launch local_planner:=scan \
  odom_topic:=/fast_lio/odometry_base \
  cloud_topic:=/fast_lio/cloud_registered
# 6
roslaunch legbot_bringup pct_tomography.launch scene:=Building
# 7
roslaunch legbot_bringup pct_plan.launch scene:=Building
# 8
roslaunch legbot_bringup visualization.launch
~~~

## 9. 常见失败

| 现象 | 优先检查 |
| --- | --- |
| FAST-LIO 没有 odom | LiDAR/IMU 话题、时间戳、lidar_type 4 |
| 没有生成 scans.pcd | save_pcd、是否正常 Ctrl-C、PCD 目录 |
| tomogram 为空或断层 | PCD 裁剪、楼梯覆盖、scene 可通行性参数 |
| 局部规划器认为起点占用 | map/odom TF、点云 frame、高度基准 |
| 机器人沿错误楼层前进 | T_map_odom、PCT 路径 Z、SCAN 高度投影 |
| 新运行对不上旧地图 | FAST-LIO 无全局重定位，需重新标定或增加重定位 |
