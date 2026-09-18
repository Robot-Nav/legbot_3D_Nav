# Third-party notices

The original integration code developed for this repository is distributed
under the Apache License 2.0 in the root `LICENSE` file. This repository also
bundles code from several upstream projects. Their copyright notices and
licenses remain in force and are not replaced by the root license.

| Component | Location | License file / status |
| --- | --- | --- |
| Original integration code | repository root, `src/legbot_bringup` | Apache-2.0 |
| EGO planner port | `src/ego_planner` | Original upstream terms apply |
| PCT planner | `src/pct_planner` | GPL-2.0; license retained in package |
| FAST-LIO | `src/fast_lio` | GPL-2.0; license retained in package |
| SCAN-Planner port | `src/scan_planner` | Apache-2.0; license retained in package |
| GO2 ROS/controller code | `src/go2_description`, `src/rl_quadruped_controller` | BSD-3-Clause upstream terms |
| Unitree SDK2 interface | `src/hardware_unitree_sdk2`, `third_party/unitree_sdk2` | Upstream terms retained |
| Livox ROS Driver2 | `src/livox_ros_driver2` | Mixed terms listed in its `LICENSE.txt` |
| GO2 Velocity Flat PPO weights | `src/go2_description/config/mjlab_flat` | Hugging Face metadata: BSD-3-Clause |
| GO2 MoE-CTS 77k weights | `src/go2_description/config/moe_cts_77k` | Hugging Face metadata: MIT |

The root Apache-2.0 license applies to the original integration code. Bundled
components remain subject to their corresponding upstream terms.

## Jazzy/GO2 integration

Active packages are under `src/`. GO2 description, HIMLoco weights,
RL/controller-common code and hardware plugins are vendored from
[quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control),
at the revision recorded in `docs/GO2_UPSTREAM_LOCK.json`. Upstream licenses
are retained in `src/upstream_licenses` and in the relevant packages.
FAST-LIO and PCT retain GPL-2.0; Livox driver retains its upstream license.
SDK origins and revisions are recorded in `docs/SENSOR_SDK_UPSTREAM_LOCK.json`.
The root Apache-2.0 license does not replace upstream licenses or establish
separate ownership of pretrained policy weights.

The optional GO2 `stairs_trot` research weight is copied from
[shivam-sood00/unitree-sim2real](https://github.com/shivam-sood00/unitree-sim2real)
at the revision recorded in `docs/GO2_STAIRS_POLICY_LOCK.json`. That revision
contains an MIT badge in its README but no license text. Its inclusion here does
not assert an open-source license grant; confirm permission with the upstream
author before redistribution or product use.

ONNX Runtime 1.23.2 is distributed by Microsoft under the MIT License. Its
release license and third-party notices are retained under
`third_party/onnxruntime/`.

The exact model repositories, revisions and file hashes for the two additional
GO2 policies are recorded in `docs/GO2_POLICY_MODELS_LOCK.json`. Their model
licenses cover the weights; their corresponding deployment source remains
subject to each source repository's own license.
