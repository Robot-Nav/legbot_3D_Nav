"""Offline callback checks for repeatable direct-goal stand/walk transitions."""

import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
from types import SimpleNamespace


def load_script(name, path):
    spec = importlib.util.spec_from_loader(name, SourceFileLoader(name, str(path)))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


root = Path(__file__).resolve().parents[1]
mission_module = load_script(
    'scan_waypoint_mission_test',
    root / 'src/legbot_bringup/scripts/scan_waypoint_mission')
sequencer_module = load_script(
    'go2_demo_sequencer_test',
    root / 'src/legbot_bringup/scripts/go2_demo_sequencer')


class Publisher:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(bool(msg.data))


mission = object.__new__(mission_module.WaypointMission)
mission.completed = False
mission.failed = False
mission.pose = (0.30, 0.0, -0.7)
mission.ready = True
mission.mode = 'direct'
mission.direct_goal = (0.0, 0.0, 0.0)
mission.last_send = 1.0
mission.reach_xy = 0.40
mission.reach_z = 0.0
mission.reached_since = None
mission.goal_stamp = (1, 2)
mission.stop_ack_received = False
mission.frame_id = 'odom'
mission.dwell_seconds = 0.15
mission.max_reach_speed = 0.0
mission.linear_speed = None
mission.complete_pub = Publisher()
states = []
mission.write_state = states.append
mission.get_logger = lambda: SimpleNamespace(info=lambda message: None)
mission.last_goal_stamp_ns = -1
mission.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
    to_msg=lambda: SimpleNamespace(sec=1, nanosec=2)))
first_stamp = mission.next_goal_stamp()
second_stamp = mission.next_goal_stamp()
assert (first_stamp.sec, first_stamp.nanosec) == (1, 2)
assert (second_stamp.sec, second_stamp.nanosec) == (1, 3)

now = [10.0]
mission_module.time.monotonic = lambda: now[0]
def stop_ack(sec, x):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=2), frame_id='odom'),
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0.0, z=-0.7)))

# Old-goal and out-of-radius execution acknowledgments cannot finish this goal.
mission.on_stop_ack(stop_ack(0, 0.30))
mission.on_stop_ack(stop_ack(1, 0.60))
mission.step_direct()
assert mission.reached_since is None
assert not mission.completed

mission.on_stop_ack(stop_ack(1, 0.30))
assert mission.reached_since == 10.0
assert not mission.completed and mission.complete_pub.values == []

now[0] = 10.2
mission.step_direct()
assert mission.completed
assert mission.complete_pub.values == [True]
assert states == ['settling_at_direct_goal', 'direct_goal_complete']


sequencer = object.__new__(sequencer_module.DemoSequencer)
sequencer.state = 'complete'
sequencer.plan_seen = True
sequencer.mission_complete = True
sequencer.get_logger = lambda: SimpleNamespace(info=lambda message: None)
sequencer.enter = lambda state: setattr(sequencer, 'state', state)
sequencer.on_complete(SimpleNamespace(data=False))
assert not sequencer.mission_complete
assert not sequencer.plan_seen
assert sequencer.state == 'waiting_for_scan_plan'

# Once the coordinator exists, an old moving spline is not permission to walk.
sequencer.plan_ready_source_seen = True
sequencer.on_plan(SimpleNamespace(pos_pts=[
    SimpleNamespace(x=0.0, y=0.0, z=0.0),
    SimpleNamespace(x=1.0, y=0.0, z=0.0)]))
assert not sequencer.plan_seen
sequencer.on_plan_ready(SimpleNamespace(data=True))
assert sequencer.plan_seen

# A nearby goal can be reached without any new B-spline; completion must
# bypass the RL pulse and ask for fixed stand.
sequencer.mission_complete = True
sequencer.ready_pub = Publisher()
sequencer.state_started = 0.0
sequencer_module.time.monotonic = lambda: 1.0
sequencer.tick()
assert sequencer.state == 'stop_pulse'

print('PASS execution stop ack, fixed-stand handoff, stale-plan rejection, and rearm')
