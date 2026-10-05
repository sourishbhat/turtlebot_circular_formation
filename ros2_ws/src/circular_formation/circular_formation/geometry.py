"""Pure-math helpers shared by the controller, monitor and tests (no ROS)."""

import math
from typing import List, Sequence, Tuple

TWO_PI = 2.0 * math.pi


def wrap_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    a = math.fmod(angle + math.pi, TWO_PI)
    if a <= 0.0:
        a += TWO_PI
    return a - math.pi


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about +z) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def odom_to_world(spawn: Tuple[float, float, float],
                  odom_xy: Tuple[float, float],
                  odom_yaw: float) -> Tuple[float, float, float]:
    """Express a DiffDrive odometry pose in the world frame.

    Gazebo's DiffDrive odometry starts at (0, 0, 0) at the spawn pose, so the
    odom frame is the spawn frame. world = spawn (+) odom.
    """
    sx, sy, syaw = spawn
    c, s = math.cos(syaw), math.sin(syaw)
    x = sx + c * odom_xy[0] - s * odom_xy[1]
    y = sy + s * odom_xy[0] + c * odom_xy[1]
    return x, y, wrap_pi(syaw + odom_yaw)


def control_point(x: float, y: float, yaw: float,
                  look_ahead: float) -> Tuple[float, float]:
    """The point `look_ahead` metres in front of the robot base."""
    return x + look_ahead * math.cos(yaw), y + look_ahead * math.sin(yaw)


def centroid(points: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
    n = len(points)
    return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)


def polar(point: Tuple[float, float],
          center: Tuple[float, float]) -> Tuple[float, float]:
    """(radius, angle in [0, 2pi)) of `point` about `center`."""
    dx, dy = point[0] - center[0], point[1] - center[1]
    return math.hypot(dx, dy), math.atan2(dy, dx) % TWO_PI


def ring_gaps(angles: Sequence[float],
              keys: Sequence[int],
              index: int) -> Tuple[float, float, int, int]:
    """Angular gaps from element `index` to its ring neighbours.

    `angles` are in [0, 2pi); ties are broken by `keys` (robot ids) so the
    ordering is total and identical on every robot. Returns
    (gap_next, gap_prev, next_idx, prev_idx) where gap_next is the CCW angle
    to the next robot and gap_prev the CW angle to the previous one. With one
    other robot the two gaps sum to 2pi; with none they are both 2pi.
    """
    n = len(angles)
    if n < 2:
        return TWO_PI, TWO_PI, index, index
    order = sorted(range(n), key=lambda k: (angles[k], keys[k]))
    pos = order.index(index)
    nxt = order[(pos + 1) % n]
    prv = order[(pos - 1) % n]
    gap_next = angles[nxt] - angles[index]
    if pos == n - 1:
        gap_next += TWO_PI
    gap_prev = angles[index] - angles[prv]
    if pos == 0:
        gap_prev += TWO_PI
    return gap_next, gap_prev, nxt, prv


def all_gaps(angles: Sequence[float], keys: Sequence[int]) -> List[float]:
    """Sorted CCW gaps between consecutive robots on the ring (sum = 2pi)."""
    n = len(angles)
    if n < 2:
        return [TWO_PI]
    order = sorted(range(n), key=lambda k: (angles[k], keys[k]))
    gaps = []
    for i in range(n):
        a = angles[order[i]]
        b = angles[order[(i + 1) % n]]
        gaps.append(b - a if i < n - 1 else b - a + TWO_PI)
    return gaps
