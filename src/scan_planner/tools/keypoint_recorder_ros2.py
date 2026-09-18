#!/usr/bin/env python3
"""Record the robot's current odometry as a reusable 3D SCAN route."""

import argparse
import copy
import math
import os
import select
import sys
import termios
import tty

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def format_float(value):
    text = f'{float(value):.6f}'.rstrip('0').rstrip('.')
    return '0' if text in ('', '-0') else text


class KeypointRecorder(Node):
    def __init__(self, args):
        super().__init__('scan_keypoint_recorder')
        self.output = os.path.abspath(os.path.expanduser(args.output))
        self.autosave = as_bool(args.autosave)
        self.latest = None
        self.frame_id = args.frame_id
        self.keypoints = []

        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.route_pub = self.create_publisher(Path, args.route_topic, route_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/scan/recorded_keypoint_markers', route_qos)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '/scan/keypoints/record_current', self.on_record)
        self.create_service(Trigger, '/scan/keypoints/undo', self.on_undo)
        self.create_service(Trigger, '/scan/keypoints/clear', self.on_clear)
        self.create_service(Trigger, '/scan/keypoints/save', self.on_save)
        self.create_service(Trigger, '/scan/keypoints/publish_route', self.on_publish)
        self.create_timer(0.5, self.publish_visualization)

        if as_bool(args.load_existing) and os.path.isfile(self.output):
            self.load()
        self.get_logger().info(
            f'Keypoint recorder ready: odom={args.odom_topic}, output={self.output}, '
            f'loaded={len(self.keypoints)}')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        values = (float(p.x), float(p.y), float(p.z))
        if all(math.isfinite(value) for value in values):
            self.latest = values
            if msg.header.frame_id:
                self.frame_id = msg.header.frame_id

    def record_current(self):
        if self.latest is None:
            return False, 'no valid odometry received'
        self.keypoints.append(self.latest)
        self.changed()
        index = len(self.keypoints)
        text = 'recorded %d: [%.3f, %.3f, %.3f]' % (index, *self.latest)
        self.get_logger().info(text)
        return True, text

    def replace_current(self, index):
        if self.latest is None:
            return False, 'no valid odometry received'
        if index < 1 or index > len(self.keypoints):
            return False, f'index must be between 1 and {len(self.keypoints)}'
        self.keypoints[index - 1] = self.latest
        self.changed()
        return True, 'replaced %d: [%.3f, %.3f, %.3f]' % (index, *self.latest)

    def delete(self, index):
        if index < 1 or index > len(self.keypoints):
            return False, f'index must be between 1 and {len(self.keypoints)}'
        point = self.keypoints.pop(index - 1)
        self.changed()
        return True, 'deleted %d: [%.3f, %.3f, %.3f]' % (index, *point)

    def undo(self):
        if not self.keypoints:
            return False, 'no keypoint to undo'
        point = self.keypoints.pop()
        self.changed()
        return True, 'removed %d: [%.3f, %.3f, %.3f]' % (
            len(self.keypoints) + 1, *point)

    def clear(self):
        count = len(self.keypoints)
        if count == 0:
            return False, 'no keypoints to clear'
        self.keypoints.clear()
        self.changed()
        return True, f'cleared {count} keypoints'

    def changed(self):
        self.publish_visualization()
        if self.autosave:
            self.save()

    def yaml_text(self):
        lines = [
            'scan_planner_node:',
            '  ros__parameters:',
            f'    fsm.waypoint_num: {len(self.keypoints)}',
        ]
        for index, point in enumerate(self.keypoints):
            for axis, value in zip(('x', 'y', 'z'), point):
                lines.append(
                    f'    fsm.waypoint{index}_{axis}: {format_float(value)}')
        return '\n'.join(lines) + '\n'

    def save(self):
        directory = os.path.dirname(self.output)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = self.output + '.tmp'
        with open(temporary, 'w', encoding='utf-8') as stream:
            stream.write(self.yaml_text())
        os.replace(temporary, self.output)
        self.get_logger().info(f'saved {len(self.keypoints)} keypoints to {self.output}')

    def load(self):
        document = yaml.safe_load(open(self.output, encoding='utf-8')) or {}
        params = document.get('scan_planner_node', {}).get('ros__parameters', {})
        count = int(params.get('fsm.waypoint_num', 0))
        loaded = []
        for index in range(count):
            point = tuple(float(params[f'fsm.waypoint{index}_{axis}'])
                          for axis in ('x', 'y', 'z'))
            if not all(math.isfinite(value) for value in point):
                raise ValueError(f'non-finite keypoint {index + 1} in {self.output}')
            loaded.append(point)
        self.keypoints = loaded

    def route_message(self):
        route = Path()
        route.header.stamp = self.get_clock().now().to_msg()
        route.header.frame_id = self.frame_id
        for x, y, z in self.keypoints:
            pose = PoseStamped()
            pose.header = route.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            route.poses.append(pose)
        return route

    def publish_route(self):
        if len(self.keypoints) < 1:
            return False, 'route has no keypoints'
        self.route_pub.publish(self.route_message())
        return True, f'published {len(self.keypoints)} keypoints'

    def publish_visualization(self):
        route = self.route_message()
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        line = Marker()
        line.header = route.header
        line.ns = 'recorded_route'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.r, line.color.g, line.color.b, line.color.a = 0.9, 0.25, 0.9, 0.9
        line.points = [pose.pose.position for pose in route.poses]
        array.markers.append(line)
        for index, pose in enumerate(route.poses, 1):
            marker = Marker()
            marker.header = route.header
            marker.ns = 'recorded_keypoints'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = copy.deepcopy(pose.pose)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.9, 0.25, 0.9, 0.95
            array.markers.append(marker)
            label = Marker()
            label.header = route.header
            label.ns = 'recorded_keypoint_labels'
            label.id = 10000 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = copy.deepcopy(pose.pose)
            label.pose.position.z += 0.28
            label.scale.z = 0.22
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = str(index)
            array.markers.append(label)
        self.marker_pub.publish(array)

    @staticmethod
    def fill(response, result):
        response.success, response.message = result
        return response

    def on_record(self, request, response):
        del request
        return self.fill(response, self.record_current())

    def on_undo(self, request, response):
        del request
        return self.fill(response, self.undo())

    def on_clear(self, request, response):
        del request
        return self.fill(response, self.clear())

    def on_save(self, request, response):
        del request
        self.save()
        return self.fill(response, (True, f'saved {len(self.keypoints)} keypoints'))

    def on_publish(self, request, response):
        del request
        return self.fill(response, self.publish_route())

    def list_points(self):
        if not self.keypoints:
            print('No recorded keypoints.', flush=True)
            return
        for index, point in enumerate(self.keypoints, 1):
            print('%02d: [%.3f, %.3f, %.3f]' % (index, *point), flush=True)


def print_help():
    print('''
SCAN ROS 2 keypoint recorder
  Enter / Space / a : record current odometry XYZ
  r                 : replace a keypoint with current odometry
  d                 : delete a keypoint
  u                 : undo last keypoint
  l                 : list keypoints
  s                 : save ROS 2 parameter YAML
  p                 : publish the recorded 3D route
  h / ?             : show help
  q                 : save and quit
''', flush=True)


def prompt_index(old_settings, action, count):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    try:
        value = input(f'{action} keypoint index (1-{count}, blank cancels): ').strip()
        return int(value) if value else None
    except ValueError:
        return None
    finally:
        tty.setcbreak(sys.stdin.fileno())


def keyboard_loop(node):
    print_help()
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                continue
            key = sys.stdin.read(1).lower()
            result = None
            if key in ('\n', '\r', ' ', 'a'):
                result = node.record_current()
            elif key == 'u':
                result = node.undo()
            elif key == 'l':
                node.list_points()
            elif key == 's':
                node.save()
            elif key == 'p':
                result = node.publish_route()
            elif key in ('r', 'd'):
                index = prompt_index(old_settings, 'replace' if key == 'r' else 'delete',
                                     len(node.keypoints))
                if index is not None:
                    result = (node.replace_current(index) if key == 'r'
                              else node.delete(index))
            elif key in ('h', '?'):
                print_help()
            elif key == 'q' or key == '\x03':
                node.save()
                break
            if result is not None:
                print(result[1], flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom-topic', default='/fast_lio/odometry_base')
    parser.add_argument('--output', required=True)
    parser.add_argument('--route-topic', default='/scan/recorded_keypoints')
    parser.add_argument('--frame-id', default='odom')
    parser.add_argument('--autosave', default='true')
    parser.add_argument('--load-existing', default='false')
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init(args=sys.argv)
    node = KeypointRecorder(args)
    try:
        if sys.stdin.isatty():
            keyboard_loop(node)
        else:
            node.get_logger().info('No TTY; use /scan/keypoints/* Trigger services')
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
