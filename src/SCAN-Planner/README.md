<div align="center">
  <h1>SCAN-Planner</h1>
  <h2>Spatial Collision-Aware Local Planning for Route-Guided<br/>Long-Range Quadruped Navigation</h2>
  <p align="center">
    Han Zheng,
    Zhe Chen,
    Yiwen Fu,
    Ming Yang,
    Tong Qin<sup>*</sup>
  </p>
  <a href="https://arxiv.org/abs/2606.19555" target="_blank"><img alt="Paper" src="https://img.shields.io/badge/Paper-arXiv-b31b1b?logo=arxiv&logoColor=white"/></a>
  <a href="https://www.bilibili.com/video/BV15a7P6UEXb/" target="_blank"><img alt="Video" src="https://img.shields.io/badge/Video-Bilibili-FB7299?logo=bilibili&logoColor=white"/></a>
  <a href="https://wuyi2121.github.io/SCAN-Planner/" target="_blank"><img alt="Project Page" src="https://img.shields.io/badge/Project_Page-Website-4A90E2?logo=googlechrome&logoColor=white"/></a>
  <br/>
</div>

<p align="center">
  <img src="assets/images/abstract_real.jpg" width="100%"/>
</p>

<p align="justify">
  SCAN-Planner is a spatial collision-aware local planner, providing a <strong>robust low-level planning foundation</strong> for various upper-level tasks, such as autonomous exploration and vision-language navigation.
</p>

## 🧭 System Overview

<p align="center">
  <img src="assets/images/framework.jpg" width="90%"/>
</p>

## 📢 News
- **[Jul. 29, 2026]**: CAD files for our [sensor layout](https://github.com/zhechen003/GO2-EDU-sensor_layout) are now available.
- **[Jul. 13, 2026]**: ROS2 support is now available from community! Thanks to [xiaoqi371317](https://github.com/xiaoqi371317) for contributing the ROS2 interface. Check out the [ros2-community](https://github.com/wuyi2121/SCAN-Planner/tree/ros2-community) branch.
- **[Jul. 9, 2026]**:  Release the main algorithm of **SCAN-Planner**.

## 🤖 Demonstrations

<table>
  <tr>
    <td align="center" width="50%">
      <img src="assets/gif/maze_compress.gif" width="100%"/>
    </td>
    <td align="center" width="50%">
      <img src="assets/gif/sshape_compress.gif" width="100%"/>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="assets/gif/terrain_compress.gif" width="100%"/>
    </td>
    <td align="center" width="50%">
      <img src="assets/gif/dynamic_compress.gif" width="100%"/>
    </td>
  </tr>
</table>

More videos and interactive demonstrations are available on the
<a href="https://wuyi2121.github.io/SCAN-Planner/" target="_blank">project page</a>.

## 🛠️ Installation

> Tested on Ubuntu 20.04 with ROS Noetic

**Step 1**. Install [Armadillo](http://arma.sourceforge.net/), which is required by **simulator**.
```
sudo apt-get install libarmadillo-dev
``` 

**Step 2**. Clone our repository and compile. 
```
git clone https://github.com/wuyi2121/SCAN-Planner.git
cd SCAN-Planner
catkin_make
```

## 🚀 Quick Start

Launch RViz in one terminal:
```
source devel/setup.bash && roslaunch scan_planner rviz.launch
```

Launch the algorithm in another terminal:
```
source devel/setup.bash && roslaunch scan_planner run.launch
```

## 🔧 Important Functions

The main launch options are defined in [`run.launch`](src/planner/plan_manage/launch/run.launch):

- `is_real_world`: set to `true` when running with real robot topics (body_pose_topic and sensor_pose_topic), and `false` when testing with the simulator.
- `navi_mode`: selects the navigation interface:
  - `1`: interactive 2D Nav Goal mode
  - `2`: keypoint-based multi-floor navigation; see [`tools/README.md`](tools/README.md)
  - `3`: reference-path tracking with local obstacle avoidance; see [`TravExplorer`](https://github.com/wuyi2121/TravExplorer)

  **Note: If the robot cannot climb stairs, increase the z height of body, keypoints or initial path.**
- `sensor_type`: select the sensing input. Use `lidar` for point-cloud sensors such as MID360, and `depth` for depth cameras such as RealSense D435.

Other algorithm-related parameters are listed in [`advanced_param.xml`](src/planner/plan_manage/launch/advanced_param.xml). The default settings are tuned for Unitree Go2 and should be adjusted when using a different robot platform.

## ⚙️ Optional
**local_sensing** provides CPU and GPU implementations: `pcl_render_node` and `opengl_render_node`.
The CPU version is built by default for better compatibility. To build the GPU backend, first install the dependencies:

```bash
sudo apt-get install libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

Then enable the GPU build option and compile:

```bash
catkin_make -DUSE_GPU=ON
```

The `use_gpu` option in [`simulator.xml`](src/planner/plan_manage/launch/simulator.xml) selects which sensing node to launch.

## 🔩 Hardware

To support the robotics community and enhance the reproducibility of our work, we provide CAD files for our [sensor layout](https://github.com/zhechen003/GO2-EDU-sensor_layout), available in ".SLDPRT" and ".STL" formats. These files can be opened and edited using Solidworks.

## 🤓 Acknowledgements
We would like to express our gratitude to the following projects, which have provided significant support and inspiration for our work:

- Our planner supports various high-level tasks, such as a cross-floor embodied exploration project [TravExplorer](https://github.com/wuyi2121/TravExplorer).

- Our localization module is based on [Elevator-LIO](https://github.com/xiaofan4122/Elevator-LIO), a robust multi-floor extension of [FAST-LIO2](https://github.com/hku-mars/FAST_LIO).

- Our framework builds on [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner), which achieves impressive performance in quadrotor local planning.

- Our map representation is inspired by [ROG-Map](https://github.com/hku-mars/ROG-Map), a high-performance robot-centric mapping framework.

- Our simulator is adapted from [MARSIM](https://github.com/hku-mars/MARSIM), with map generation from [Mockamap](https://github.com/HKUST-Aerial-Robotics/mockamap) and trotting motion from [Leg-KILO](https://github.com/ouguangjun/Leg-KILO).


## 📚 Citation

```bibtex
@article{zheng2026scan,
  title={SCAN-Planner: Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation},
  author={Zheng, Han and Chen, Zhe and Fu, Yiwen and Yang, Ming and Qin, Tong},
  journal={arXiv preprint arXiv:2606.19555},
  year={2026}
}
```

## ⚖️ License

This project is licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.
