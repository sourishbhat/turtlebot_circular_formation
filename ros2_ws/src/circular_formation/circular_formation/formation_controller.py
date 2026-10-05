#!/usr/bin/env python3
"""ROS 2 node: one per robot, runs the circular formation control law.

Subscribes : /robot_<i>/odom for itself and every peer (nav_msgs/Odometry)
Publishes  : /robot_<i>/cmd_vel (geometry_msgs/Twist)

Decentralised: the node only uses peer poses it has received recently and
that lie within `comm_range`, exactly like NSKsim's robot_node uses peer odom.
DiffDrive odometry starts at zero at the spawn pose, so every pose is shifted
into the world frame with the spawn poses passed in by the launch file.
"""

import math
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from .control_law import FormationParams, compute_control
from .geometry import control_point, odom_to_world, quat_to_yaw

# Latest-pose-only stream. The gz bridge publishes RELIABLE; a best-effort
# subscriber is compatible with that (the reverse is not).
ODOM_QOS = qos_profile_sensor_data

CMD_VEL_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class FormationController(Node):

    def __init__(self):
        super().__init__('formation_controller')

        d = FormationParams()
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_robots', d.num_robots)
        self.declare_parameter('spawn_xs', [0.0])
        self.declare_parameter('spawn_ys', [0.0])
        self.declare_parameter('spawn_yaws', [0.0])
        self.declare_parameter('radius', d.radius)
        self.declare_parameter('omega', d.omega)
        self.declare_parameter('center_mode', d.center_mode)
        self.declare_parameter('center_x', d.center_x)
        self.declare_parameter('center_y', d.center_y)
        self.declare_parameter('k_radial', d.k_radial)
        self.declare_parameter('k_angular', d.k_angular)
        self.declare_parameter('k_repulse', d.k_repulse)
        self.declare_parameter('d_safe', d.d_safe)
        self.declare_parameter('rho_min', d.rho_min)
        self.declare_parameter('look_ahead', d.look_ahead)
        self.declare_parameter('v_max', d.v_max)
        self.declare_parameter('w_max', d.w_max)
        self.declare_parameter('comm_range', d.comm_range)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('peer_timeout', 2.0)
        self.declare_parameter('start_delay', 2.0)
        self.declare_parameter('log_period', 5.0)

        gp = self.get_parameter
        self.robot_id = int(gp('robot_id').value)
        self.num_robots = int(gp('num_robots').value)
        self.spawn_xs = list(gp('spawn_xs').value)
        self.spawn_ys = list(gp('spawn_ys').value)
        self.spawn_yaws = list(gp('spawn_yaws').value)
        self.peer_timeout = float(gp('peer_timeout').value)
        self.start_delay = float(gp('start_delay').value)
        self.log_period = float(gp('log_period').value)

        center_mode = str(gp('center_mode').value)
        if center_mode not in ('centroid', 'fixed'):
            raise ValueError(
                f"center_mode must be 'centroid' or 'fixed', got "
                f"{center_mode!r}")
        self.prm = FormationParams(
            radius=float(gp('radius').value),
            omega=float(gp('omega').value),
            center_mode=center_mode,
            center_x=float(gp('center_x').value),
            center_y=float(gp('center_y').value),
            k_radial=float(gp('k_radial').value),
            k_angular=float(gp('k_angular').value),
            k_repulse=float(gp('k_repulse').value),
            d_safe=float(gp('d_safe').value),
            rho_min=float(gp('rho_min').value),
            look_ahead=float(gp('look_ahead').value),
            v_max=float(gp('v_max').value),
            w_max=float(gp('w_max').value),
            comm_range=float(gp('comm_range').value),
            num_robots=self.num_robots,
        )

        # robot id -> (x, y, yaw) in world frame, and receive time [s]
        self._pose: Dict[int, Tuple[float, float, float]] = {}
        self._stamp: Dict[int, float] = {}
        self._t0: Optional[float] = None
        self._last_log = -1e9

        self.cmd_pub = self.create_publisher(
            Twist, f'/robot_{self.robot_id}/cmd_vel', CMD_VEL_QOS)
        for j in range(self.num_robots):
            self.create_subscription(
                Odometry, f'/robot_{j}/odom',
                lambda msg, rid=j: self._odom_cb(msg, rid), ODOM_QOS)

        self.create_timer(1.0 / float(gp('control_rate').value), self._step)
        self.get_logger().info(
            f'[Robot {self.robot_id}] circular formation: R={self.prm.radius} '
            f'm, omega={self.prm.omega} rad/s, centre={center_mode}')

    # ── helpers ──────────────────────────────────────────────────────────

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _spawn(self, rid: int) -> Tuple[float, float, float]:
        def pick(seq):
            return seq[rid] if rid < len(seq) else 0.0
        return pick(self.spawn_xs), pick(self.spawn_ys), pick(self.spawn_yaws)

    def _odom_cb(self, msg: Odometry, rid: int):
        p = msg.pose.pose
        q = p.orientation
        self._pose[rid] = odom_to_world(
            self._spawn(rid), (p.position.x, p.position.y),
            quat_to_yaw(q.x, q.y, q.z, q.w))
        self._stamp[rid] = self._now()

    # ── control loop ─────────────────────────────────────────────────────

    def _step(self):
        now = self._now()
        if self._t0 is None:
            self._t0 = now
        mine = self._pose.get(self.robot_id)
        if mine is None or now - self._t0 < self.start_delay:
            return

        x, y, yaw = mine
        la = self.prm.look_ahead
        me = control_point(x, y, yaw, la)
        peers = {
            j: control_point(p[0], p[1], p[2], la)
            for j, p in self._pose.items()
            if j != self.robot_id and now - self._stamp[j] <= self.peer_timeout
        }

        out = compute_control(self.robot_id, me, yaw, peers, self.prm)

        cmd = Twist()
        cmd.linear.x = float(out.v)
        cmd.angular.z = float(out.w)
        self.cmd_pub.publish(cmd)

        if now - self._last_log >= self.log_period:
            self._last_log = now
            self.get_logger().info(
                f'[Robot {self.robot_id}] rho={out.rho:.2f} m '
                f'theta={math.degrees(out.theta):6.1f} deg '
                f'gaps(next/prev)={math.degrees(out.gap_next):.0f}/'
                f'{math.degrees(out.gap_prev):.0f} deg '
                f'peers={out.visible} v={out.v:.2f} w={out.w:.2f}')

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FormationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:  # context may already be shut down
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
