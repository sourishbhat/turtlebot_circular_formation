#!/usr/bin/env python3
"""ROS 2 node: global observer of the formation (evaluation only).

Publishes  : /formation/metrics (std_msgs/Float64MultiArray)
                 [mean_radius, mean_radius_err, max_radius_err,
                  max_gap_err_deg, min_separation, converged(0/1)]
             /formation/markers (visualization_msgs/MarkerArray) for RViz
Logs a line every `monitor_period` seconds and a one-off message when the
formation has stayed within tolerance for `hold_time` seconds.

The robots never read this node; it is not part of the control loop.
"""

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import ColorRGBA, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import control_point, odom_to_world, quat_to_yaw
from .metrics import formation_metrics

PALETTE = [(0.90, 0.25, 0.25), (0.25, 0.75, 0.30), (0.25, 0.45, 0.95),
           (0.95, 0.65, 0.15), (0.65, 0.35, 0.85), (0.20, 0.80, 0.80),
           (0.90, 0.45, 0.70), (0.55, 0.55, 0.55)]


class FormationMonitor(Node):

    def __init__(self):
        super().__init__('formation_monitor')
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('spawn_xs', [0.0])
        self.declare_parameter('spawn_ys', [0.0])
        self.declare_parameter('spawn_yaws', [0.0])
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('center_mode', 'centroid')
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('look_ahead', 0.15)
        self.declare_parameter('monitor_period', 1.0)
        self.declare_parameter('tol_radius', 0.05)
        self.declare_parameter('tol_gap_deg', 5.0)
        self.declare_parameter('hold_time', 10.0)
        self.declare_parameter('frame_id', 'world')

        gp = self.get_parameter
        self.n = int(gp('num_robots').value)
        self.spawn_xs = list(gp('spawn_xs').value)
        self.spawn_ys = list(gp('spawn_ys').value)
        self.spawn_yaws = list(gp('spawn_yaws').value)
        self.radius = float(gp('radius').value)
        self.center_mode = str(gp('center_mode').value)
        self.fixed_center = (float(gp('center_x').value),
                             float(gp('center_y').value))
        self.look_ahead = float(gp('look_ahead').value)
        self.tol_radius = float(gp('tol_radius').value)
        self.tol_gap = float(gp('tol_gap_deg').value)
        self.hold_time = float(gp('hold_time').value)
        self.frame_id = str(gp('frame_id').value)

        self._pose: Dict[int, Tuple[float, float, float]] = {}
        self._ok_since: Optional[float] = None
        self._reported = False

        self.metrics_pub = self.create_publisher(
            Float64MultiArray, '/formation/metrics', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/formation/markers', 10)
        for j in range(self.n):
            self.create_subscription(
                Odometry, f'/robot_{j}/odom',
                lambda msg, rid=j: self._odom_cb(msg, rid),
                qos_profile_sensor_data)
        self.create_timer(float(gp('monitor_period').value), self._tick)
        self.get_logger().info(
            f'Formation monitor: {self.n} robots, target R={self.radius} m')

    def _spawn(self, rid):
        def pick(seq):
            return seq[rid] if rid < len(seq) else 0.0
        return pick(self.spawn_xs), pick(self.spawn_ys), pick(self.spawn_yaws)

    def _odom_cb(self, msg, rid):
        p = msg.pose.pose
        q = p.orientation
        self._pose[rid] = odom_to_world(
            self._spawn(rid), (p.position.x, p.position.y),
            quat_to_yaw(q.x, q.y, q.z, q.w))

    def _tick(self):
        if len(self._pose) < self.n:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        pts = [control_point(*self._pose[i], self.look_ahead)
               for i in range(self.n)]
        m = formation_metrics(pts, self.radius, self.center_mode,
                              self.fixed_center)
        ok = (m.max_radius_err <= self.tol_radius
              and m.max_gap_err_deg <= self.tol_gap)
        if ok:
            if self._ok_since is None:
                self._ok_since = now
        else:
            self._ok_since = None
            self._reported = False
        converged = ok and self._ok_since is not None \
            and now - self._ok_since >= self.hold_time

        msg = Float64MultiArray()
        msg.data = [m.mean_radius, m.mean_radius_err, m.max_radius_err,
                    m.max_gap_err_deg, m.min_separation,
                    1.0 if converged else 0.0]
        self.metrics_pub.publish(msg)
        self._publish_markers(pts, m.center)

        self.get_logger().info(
            f'R_mean={m.mean_radius:.2f} m  max|R-err|={m.max_radius_err:.3f} m  '
            f'max gap err={m.max_gap_err_deg:.1f} deg  '
            f'min sep={m.min_separation:.2f} m')
        if converged and not self._reported:
            self._reported = True
            self.get_logger().info(
                f'FORMATION CONVERGED: within {self.tol_radius} m / '
                f'{self.tol_gap} deg for {self.hold_time:.0f} s')

    def _publish_markers(self, pts: List[Tuple[float, float]],
                         center: Tuple[float, float]):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        circle = Marker()
        circle.header.frame_id = self.frame_id
        circle.header.stamp = stamp
        circle.ns, circle.id = 'target_circle', 0
        circle.type, circle.action = Marker.LINE_STRIP, Marker.ADD
        circle.scale.x = 0.03
        circle.color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=0.9)
        circle.pose.orientation.w = 1.0
        for k in range(73):
            a = 2.0 * math.pi * k / 72
            circle.points.append(Point(
                x=center[0] + self.radius * math.cos(a),
                y=center[1] + self.radius * math.sin(a), z=0.02))
        arr.markers.append(circle)

        for i, (x, y) in enumerate(pts):
            r, g, b = PALETTE[i % len(PALETTE)]
            s = Marker()
            s.header.frame_id = self.frame_id
            s.header.stamp = stamp
            s.ns, s.id = 'robots', i
            s.type, s.action = Marker.SPHERE, Marker.ADD
            s.pose.position.x, s.pose.position.y, s.pose.position.z = x, y, 0.1
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = 0.25
            s.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            arr.markers.append(s)

            t = Marker()
            t.header.frame_id = self.frame_id
            t.header.stamp = stamp
            t.ns, t.id = 'labels', i
            t.type, t.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, 0.45
            t.pose.orientation.w = 1.0
            t.scale.z = 0.25
            t.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
            t.text = f'R{i}'
            arr.markers.append(t)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FormationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
