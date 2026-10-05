import math

import pytest

from circular_formation.control_law import (FormationParams, compute_control,
                                            visible_peers)


def ring(n, r, c=(0.0, 0.0), phase=0.0):
    return [(c[0] + r * math.cos(phase + 2 * math.pi * k / n),
             c[1] + r * math.sin(phase + 2 * math.pi * k / n))
            for k in range(n)]


def step(i, pts, prm, yaw=0.0):
    peers = {j: p for j, p in enumerate(pts) if j != i}
    return compute_control(i, pts[i], yaw, peers, prm)


def test_equilibrium_static_ring_is_still():
    prm = FormationParams(omega=0.0, center_mode='fixed', num_robots=6)
    pts = ring(6, prm.radius)
    for i in range(6):
        out = step(i, pts, prm)
        assert math.hypot(*out.u) < 1e-9
        assert out.rho == pytest.approx(prm.radius)


def test_rotating_ring_velocity_is_tangential_omega_R():
    prm = FormationParams(omega=0.08, center_mode='centroid', num_robots=6)
    pts = ring(6, prm.radius)
    out = step(0, pts, prm)                  # robot 0 at angle 0 -> tangent +y
    assert out.u[0] == pytest.approx(0.0, abs=1e-9)
    assert out.u[1] == pytest.approx(prm.omega * prm.radius)


def test_inside_robot_is_pushed_out_outside_pulled_in():
    prm = FormationParams(omega=0.0, center_mode='fixed', num_robots=2)
    inside = compute_control(0, (0.5, 0.0), 0.0, {}, prm)
    outside = compute_control(0, (3.5, 0.0), 0.0, {}, prm)
    assert inside.u[0] > 0 and outside.u[0] < 0


def test_crowded_gap_pushes_robot_toward_larger_gap():
    prm = FormationParams(omega=0.0, center_mode='fixed', num_robots=3,
                          k_repulse=0.0)
    R = prm.radius
    a = [0.0, 0.5, 4.0]   # robot 0: next gap 0.5 (small), prev gap ~2.28
    pts = [(R * math.cos(t), R * math.sin(t)) for t in a]
    out = step(0, pts, prm)
    # gap_prev > gap_next -> theta_dot < 0 -> moves clockwise (u_y < 0 at 0 rad)
    assert out.u[1] < 0


def test_comm_range_filters_peers():
    me = (0.0, 0.0)
    peers = {1: (1.0, 0.0), 2: (10.0, 0.0)}
    assert set(visible_peers(me, peers, 5.0)) == {1}
    prm = FormationParams(comm_range=5.0, center_mode='centroid')
    out = compute_control(0, me, 0.0, peers, prm)
    assert out.visible == 1


def test_robot_on_centre_does_not_blow_up():
    prm = FormationParams(center_mode='fixed', num_robots=4)
    out = compute_control(2, (0.0, 0.0), 0.0, {}, prm)
    assert math.isfinite(out.v) and math.isfinite(out.w)
    assert math.hypot(*out.u) > 0.0          # it is told to leave


def test_commands_respect_limits():
    prm = FormationParams(v_max=0.2, w_max=2.0, center_mode='fixed')
    out = compute_control(0, (9.0, -9.0), 2.0, {1: (9.1, -9.0)}, prm)
    assert abs(out.v) <= 0.2 + 1e-9 and abs(out.w) <= 2.0 + 1e-9


def test_repulsion_only_inside_d_safe():
    base = dict(omega=0.0, center_mode='fixed', k_radial=0.0, k_angular=0.0,
                num_robots=2)
    near = compute_control(0, (2.0, 0.0), 0.0, {1: (2.3, 0.0)},
                           FormationParams(**base))
    far = compute_control(0, (2.0, 0.0), 0.0, {1: (3.0, 0.0)},
                          FormationParams(**base))
    assert near.u[0] < 0                      # pushed away from the peer at +x
    assert far.u == (0.0, 0.0) or math.hypot(*far.u) < 1e-12
