"""Decentralised circular formation control law (pure Python, no ROS).

Goal: N robots settle on a circle of radius R, evenly spaced, optionally
rotating about its centre at a constant angular rate omega.

Each robot i computes, from its own control point p_i and the control points
of the peers it can see:

  1. centre c      fixed point, or the centroid of the visible robots
                   (self included) -- a leaderless consensus on the centre.
  2. radial term   u_r = -k_radial * (rho_i - R)          (pull onto circle)
  3. angular term  theta_dot_i = omega + k_angular * (gap_next - gap_prev)
                   where the gaps are CCW angles to the next / CW angles to
                   the previous visible robot about c. Equal gaps is the only
                   equilibrium: an evenly spaced ring. The ring neighbours are
                   chosen by sorting angles, so no ids or ordering have to be
                   agreed in advance.
  4. separation    short-range repulsion inside d_safe (transient safety).

The world-frame velocity u = u_r e_r + rho_eff * theta_dot e_theta + u_rep is
turned into (v, w) by `unicycle.velocity_to_cmd`.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

from .geometry import TWO_PI, centroid, polar, ring_gaps
from .unicycle import velocity_to_cmd

Point = Tuple[float, float]


@dataclass
class FormationParams:
    radius: float = 2.0            # R, metres
    omega: float = 0.05            # rad/s, CCW positive; 0 = static ring
    center_mode: str = 'centroid'  # 'centroid' | 'fixed'
    center_x: float = 0.0
    center_y: float = 0.0
    k_radial: float = 0.8          # 1/s
    k_angular: float = 0.4         # 1/s
    k_repulse: float = 0.6         # m/s per metre of penetration
    d_safe: float = 0.6            # m, repulsion starts inside this
    rho_min: float = 0.6           # m, floor on the radius in rho*theta_dot
    look_ahead: float = 0.15       # m, control point ahead of base
    v_max: float = 0.20            # m/s  (Burger limit is 0.22)
    w_max: float = 2.0             # rad/s (Burger limit is 2.84)
    comm_range: float = 50.0       # m
    num_robots: int = 5


@dataclass
class ControlOutput:
    v: float = 0.0
    w: float = 0.0
    center: Point = (0.0, 0.0)
    rho: float = 0.0
    theta: float = 0.0
    gap_next: float = TWO_PI
    gap_prev: float = TWO_PI
    visible: int = 0
    u: Point = (0.0, 0.0)
    info: Dict[str, float] = field(default_factory=dict)


def visible_peers(me: Point, peers: Dict[int, Point],
                  comm_range: float) -> Dict[int, Point]:
    """Peers whose control point is within `comm_range` of `me`."""
    return {j: p for j, p in peers.items()
            if math.hypot(p[0] - me[0], p[1] - me[1]) <= comm_range}


def compute_control(robot_id: int, me: Point, yaw: float,
                    peers: Dict[int, Point],
                    prm: FormationParams) -> ControlOutput:
    """One control step for robot `robot_id`.

    `me` is this robot's control point, `peers` maps peer id -> control point.
    """
    vis = visible_peers(me, peers, prm.comm_range)

    # 1. centre
    if prm.center_mode == 'fixed':
        c = (prm.center_x, prm.center_y)
    else:
        c = centroid([me] + list(vis.values()))

    rho, theta = polar(me, c)
    if rho < 1e-3:
        # Degenerate (sitting on the centre): leave along an id-based ray.
        phi = TWO_PI * robot_id / max(prm.num_robots, 1)
        e_r = (math.cos(phi), math.sin(phi))
        theta = phi
    else:
        e_r = ((me[0] - c[0]) / rho, (me[1] - c[1]) / rho)
    e_t = (-e_r[1], e_r[0])

    # 2. radial
    u_r = -prm.k_radial * (rho - prm.radius)

    # 3. angular (ring-neighbour gap equalisation)
    gap_next = gap_prev = TWO_PI
    theta_dot = prm.omega
    if vis:
        ids = [robot_id] + list(vis.keys())
        pts = [me] + [vis[j] for j in vis]
        angles = [polar(p, c)[1] for p in pts]
        gap_next, gap_prev, _, _ = ring_gaps(angles, ids, 0)
        theta_dot += prm.k_angular * (gap_next - gap_prev)
    u_t = max(rho, prm.rho_min) * theta_dot

    ux = u_r * e_r[0] + u_t * e_t[0]
    uy = u_r * e_r[1] + u_t * e_t[1]

    # 4. separation
    for p in vis.values():
        dx, dy = me[0] - p[0], me[1] - p[1]
        d = math.hypot(dx, dy)
        if 1e-6 < d < prm.d_safe:
            mag = prm.k_repulse * (prm.d_safe - d) / d
            ux += mag * dx
            uy += mag * dy

    v, w = velocity_to_cmd(ux, uy, yaw, prm.look_ahead, prm.v_max, prm.w_max)
    return ControlOutput(v=v, w=w, center=c, rho=rho, theta=theta,
                         gap_next=gap_next, gap_prev=gap_prev,
                         visible=len(vis), u=(ux, uy))
