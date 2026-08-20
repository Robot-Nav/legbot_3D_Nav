# keypoint_recorder.py

`keypoint_recorder.py` records waypoint positions from a ROS odometry topic and writes them to `tools/keypoint.yaml`.

## 1. Start the Recorder

```bash
source devel/setup.bash
python3 tools/keypoint_recorder.py
```

By default, the recorder subscribes to:

```bash
/LIO/odom_vehicle
```

To use another odometry topic:

```bash
python3 tools/keypoint_recorder.py --odom /XXX
```

Note: If the robot cannot climb stairs, increase the recorded odom z height.

## 2. Keyboard Commands

After the script starts, use these keys in the terminal:

- `Enter` / `Space` / `a`: record the current position
- `r`: replace an existing waypoint with the current position
- `d`: delete an existing waypoint
- `u`: undo the last waypoint
- `l`: list recorded waypoints
- `s`: save the YAML file
- `h` / `?`: show help
- `q`: save and quit

## 3. Output Format

The default output file is `tools/keypoint.yaml`:

```yaml
fsm:
  waypoint_num: 2
  waypoint0_x: 1.0
  waypoint0_y: 2.0
  waypoint0_z: 0.5
  waypoint1_x: 3.0
  waypoint1_y: 4.0
  waypoint1_z: 0.5
```

Waypoint indices start from `waypoint0`.
