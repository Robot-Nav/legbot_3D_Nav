# Third-party notices

The original integration code developed for this repository is distributed
under the Apache License 2.0 in the root `LICENSE` file. This repository also
bundles code from several upstream projects. Their copyright notices and
licenses remain in force and are not replaced by the root license.

| Component | Location | License file / status |
| --- | --- | --- |
| Original integration code | repository root, `src/legbot_bringup` | Apache-2.0 |
| EGO-derived planner tree | `src/planner` | Original upstream terms apply |
| PCT planner | `src/PCT_planner` | GPL-2.0 |
| FAST-LIO | `src/FAST_LIO` | GPL-2.0 |
| SCAN-Planner | `src/SCAN-Planner` | Apache-2.0; nested simulator dependencies may differ |
| Unitree guide/ROS/SDK | `src/unitree_guide` | BSD-3-Clause and bundled dependency licenses |
| Mid360 IMU simulator | `src/Mid360_imu_sim` | MIT |
| UAV simulator utilities | `src/uav_simulator` | mixed BSD/LGPL/GPL licenses |

The root Apache-2.0 license applies to the original integration code. Bundled
components remain subject to their corresponding upstream terms.
