"""Initial robot poses for the formation simulation (pure Python, no ROS)."""

import math
from typing import List, Tuple

Pose = Tuple[float, float, float]  # x, y, yaw

GOLDEN_ANGLE = math.pi * (3.0 - math.sqrt(5.0))


def spawn_poses(n: int, kind: str = 'spiral', scale: float = 0.8,
                center: Tuple[float, float] = (0.0, 0.0)) -> List[Pose]:
    """Collision-free start poses, all with yaw 0.

    yaw is kept at 0 on purpose: DiffDrive odometry is expressed in the spawn
    frame, so a zero yaw keeps odom axis-aligned with the world (the same
    convention NSKsim's launch file uses).

    kind='spiral': Vogel sunflower scatter, r_k = scale*sqrt(k+0.5). Distinct
                   radii *and* angles, so the start is not a symmetric
                   fixed point of the controller.
    kind='ring'  : regular n-gon of radius `scale` -- already evenly spaced.
    """
    cx, cy = center
    poses = []
    for k in range(n):
        if kind == 'ring':
            a = 2.0 * math.pi * k / n
            r = scale
        elif kind == 'spiral':
            a = k * GOLDEN_ANGLE
            r = scale * math.sqrt(k + 0.5)
        else:
            raise ValueError(f"unknown spawn layout {kind!r} "
                             "(expected 'spiral' or 'ring')")
        poses.append((cx + r * math.cos(a), cy + r * math.sin(a), 0.0))
    return poses


def min_separation(poses: List[Pose]) -> float:
    best = float('inf')
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            best = min(best, math.hypot(poses[i][0] - poses[j][0],
                                        poses[i][1] - poses[j][1]))
    return best
