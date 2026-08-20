#!/usr/bin/env python3

import argparse
import math
import os
import select
import sys
import termios
import tty

try:
    import rospy
    from nav_msgs.msg import Odometry
except ImportError as exc:
    print("Failed to import ROS Python modules: {}".format(exc), file=sys.stderr)
    print("Source your ROS workspace first, for example:", file=sys.stderr)
    print("  source devel/setup.bash", file=sys.stderr)
    sys.exit(2)


DEFAULT_OUTPUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "keypoint.yaml")


def strip_ros_args(argv):
    return [arg for arg in argv if ":=" not in arg and not arg.startswith("__")]


def format_float(value):
    text = "{:.6f}".format(float(value)).rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def atomic_write(path, content):
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)

    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        handle.write(content)
    os.replace(tmp_path, path)


class KeypointRecorder:
    def __init__(self, args):
        self.odom_topic = args.odom
        self.output_path = DEFAULT_OUTPUT
        self.latest_odom = None
        self.waypoints = []

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

    def odom_callback(self, msg):
        self.latest_odom = msg

    def current_point(self):
        if self.latest_odom is None:
            return None

        position = self.latest_odom.pose.pose.position
        point = (float(position.x), float(position.y), float(position.z))
        return point if all(math.isfinite(value) for value in point) else None

    def record_current(self):
        point = self.current_point()
        if point is None:
            rospy.logwarn("No valid odometry yet; cannot record waypoint.")
            return

        self.waypoints.append(point)
        rospy.loginfo(
            "Recorded waypoint %d: [%.3f, %.3f, %.3f]",
            len(self.waypoints),
            point[0],
            point[1],
            point[2],
        )

    def replace_current(self, index):
        if index < 1 or index > len(self.waypoints):
            rospy.logwarn("Invalid waypoint index %d. Valid range: 1-%d.", index, len(self.waypoints))
            return

        point = self.current_point()
        if point is None:
            rospy.logwarn("No valid odometry yet; cannot replace waypoint.")
            return

        self.waypoints[index - 1] = point
        rospy.loginfo(
            "Replaced waypoint %d: [%.3f, %.3f, %.3f]",
            index,
            point[0],
            point[1],
            point[2],
        )

    def delete_waypoint(self, index):
        if index < 1 or index > len(self.waypoints):
            rospy.logwarn("Invalid waypoint index %d. Valid range: 1-%d.", index, len(self.waypoints))
            return

        point = self.waypoints.pop(index - 1)
        rospy.loginfo(
            "Deleted waypoint %d: [%.3f, %.3f, %.3f]",
            index,
            point[0],
            point[1],
            point[2],
        )

    def undo_last(self):
        if not self.waypoints:
            rospy.logwarn("No waypoint to undo.")
            return

        point = self.waypoints.pop()
        rospy.loginfo(
            "Removed waypoint %d: [%.3f, %.3f, %.3f]",
            len(self.waypoints) + 1,
            point[0],
            point[1],
            point[2],
        )

    def list_waypoints(self):
        if not self.waypoints:
            rospy.loginfo("No recorded waypoints.")
            return

        rospy.loginfo("Recorded waypoints:")
        for index, point in enumerate(self.waypoints, 1):
            rospy.loginfo("  %02d: [%.3f, %.3f, %.3f]", index, point[0], point[1], point[2])

    def build_yaml(self):
        lines = ["fsm:", "  waypoint_num: {}".format(len(self.waypoints))]
        for index, point in enumerate(self.waypoints):
            lines.append("  waypoint{}_x: {}".format(index, format_float(point[0])))
            lines.append("  waypoint{}_y: {}".format(index, format_float(point[1])))
            lines.append("  waypoint{}_z: {}".format(index, format_float(point[2])))
        return "\n".join(lines) + "\n"

    def save(self):
        atomic_write(self.output_path, self.build_yaml())
        rospy.loginfo("Saved %d waypoint(s) to %s", len(self.waypoints), self.output_path)

    def print_help(self):
        print("")
        print("keypoint_recorder")
        print("  odom topic : {}".format(self.odom_topic))
        print("  output     : {}".format(self.output_path))
        print("")
        print("Keyboard commands:")
        print("  Enter / Space / a : record current odom position")
        print("  r                 : replace an existing waypoint")
        print("  d                 : delete an existing waypoint")
        print("  u                 : undo last waypoint")
        print("  l                 : list waypoints")
        print("  s                 : save YAML")
        print("  h / ?             : show this help")
        print("  q                 : save and quit")
        print("")

    def prompt_index(self, old_settings, action):
        if not self.waypoints:
            rospy.logwarn("No waypoint to %s.", action)
            return None

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        try:
            text = input("{} waypoint index (1-{}): ".format(action.capitalize(), len(self.waypoints))).strip()
            if not text:
                print("{} cancelled.".format(action.capitalize()))
                return None
            return int(text)
        except ValueError:
            rospy.logwarn("Invalid waypoint index: %s", text)
            return None
        finally:
            tty.setcbreak(sys.stdin.fileno())

    def run(self):
        self.print_help()
        rospy.loginfo("Waiting for odometry on %s ...", self.odom_topic)

        if not sys.stdin.isatty():
            rospy.logerr("stdin is not a TTY; keyboard control is required.")
            return

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        rate = rospy.Rate(20)

        try:
            while not rospy.is_shutdown():
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                if ready:
                    ch = sys.stdin.read(1)
                    if ch in ("\n", "\r", " ", "a", "A"):
                        self.record_current()
                    elif ch in ("r", "R"):
                        index = self.prompt_index(old_settings, "replace")
                        if index is not None:
                            self.replace_current(index)
                    elif ch in ("d", "D"):
                        index = self.prompt_index(old_settings, "delete")
                        if index is not None:
                            self.delete_waypoint(index)
                    elif ch in ("u", "U"):
                        self.undo_last()
                    elif ch in ("l", "L"):
                        self.list_waypoints()
                    elif ch in ("s", "S"):
                        self.save()
                    elif ch in ("h", "H", "?"):
                        self.print_help()
                    elif ch in ("q", "Q"):
                        self.save()
                        rospy.signal_shutdown("user requested quit")
                        break
                    elif ch == "\x03":
                        rospy.signal_shutdown("keyboard interrupt")
                        break
                rate.sleep()
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            rospy.signal_shutdown("interrupted")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Record odometry positions to tools/keypoint.yaml.")
    parser.add_argument("--odom", default="/LIO/odom_vehicle", help="nav_msgs/Odometry topic to record.")
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args(strip_ros_args(sys.argv[1:]))

    rospy.init_node("keypoint_recorder")
    recorder = KeypointRecorder(args)
    recorder.run()


if __name__ == "__main__":
    main()
