"""Offline kinematic simulation of the controller (no ROS, no Gazebo).

Each robot is a unicycle with DiffDrive-like acceleration limits and a
first-order command lag; all robots run `compute_control` on the others'
control points, exactly as the ROS node does. Used by the tests and by
tools/run_offline_sim.py to sanity-check gains before touching Gazebo.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .control_law import FormationParams, compute_control
from .geometry import control_point, wrap_pi
from .metrics import Metrics, formation_metrics


@dataclass
class SimResult:
    t: List[float]
    paths: List[List[Tuple[float, float]]]   # per robot, base positions
    metrics: List[Metrics]
    min_separation: float                     # over the whole run (bases)


def simulate(poses: Sequence[Tuple[float, float, float]],
             prm: FormationParams, duration: float = 240.0,
             dt: float = 0.05, accel: float = 1.0,
             lag: float = 0.1, record_every: int = 10) -> SimResult:
    n = len(poses)
    prm.num_robots = n
    x = [p[0] for p in poses]
    y = [p[1] for p in poses]
    th = [p[2] for p in poses]
    v = [0.0] * n
    w = [0.0] * n
    paths = [[] for _ in range(n)]
    ts: List[float] = []
    ms: List[Metrics] = []
    min_sep = float('inf')

    steps = int(duration / dt)
    for k in range(steps + 1):
        cps = [control_point(x[i], y[i], th[i], prm.look_ahead)
               for i in range(n)]
        if k % record_every == 0:
            ts.append(k * dt)
            ms.append(formation_metrics(cps, prm.radius, prm.center_mode,
                                        (prm.center_x, prm.center_y)))
            for i in range(n):
                paths[i].append((x[i], y[i]))
        for i in range(n):
            peers = {j: cps[j] for j in range(n) if j != i}
            out = compute_control(i, cps[i], th[i], peers, prm)
            # first-order lag toward the command, then accel limit
            a = dt / (lag + dt)
            v_t = v[i] + a * (out.v - v[i])
            w_t = w[i] + a * (out.w - w[i])
            dv = max(-accel * dt, min(accel * dt, v_t - v[i]))
            v[i] += dv
            w[i] = w_t
            x[i] += v[i] * math.cos(th[i]) * dt
            y[i] += v[i] * math.sin(th[i]) * dt
            th[i] = wrap_pi(th[i] + w[i] * dt)
        for i in range(n):
            for j in range(i + 1, n):
                min_sep = min(min_sep, math.hypot(x[i] - x[j], y[i] - y[j]))
    return SimResult(ts, paths, ms, min_sep)
