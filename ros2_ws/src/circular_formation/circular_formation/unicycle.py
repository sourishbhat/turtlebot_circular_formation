"""Map a desired planar velocity onto (v, w) for a unicycle / diff-drive base.

Feedback linearisation about a point `look_ahead` metres in front of the base
(Lawton/Desai style): for p = base + d*[cos(yaw), sin(yaw)],

    p_dot = [[cos, -d sin], [sin, d cos]] [v, w]^T

which is invertible for d > 0, so any desired p_dot = u is realisable.
"""

import math
from typing import Tuple


def velocity_to_cmd(ux: float, uy: float, yaw: float, look_ahead: float,
                    v_max: float, w_max: float) -> Tuple[float, float]:
    """Return (v, w) tracking world-frame velocity (ux, uy) at the control
    point, scaled uniformly (curvature preserved) to respect the limits."""
    c, s = math.cos(yaw), math.sin(yaw)
    v = c * ux + s * uy
    w = (-s * ux + c * uy) / look_ahead
    scale = 1.0
    if abs(v) > v_max:
        scale = min(scale, v_max / abs(v))
    if abs(w) > w_max:
        scale = min(scale, w_max / abs(w))
    return v * scale, w * scale
