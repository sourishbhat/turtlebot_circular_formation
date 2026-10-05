"""Closed-loop checks with the offline kinematic simulator (no ROS/Gazebo)."""

import math

import pytest

from circular_formation.control_law import FormationParams
from circular_formation.simulation import simulate
from circular_formation.spawn_layout import spawn_poses

ROBOT_DIAMETER = 0.21   # TurtleBot3 Burger footprint


@pytest.mark.parametrize('n', [3, 5, 8])
@pytest.mark.parametrize('mode', ['centroid', 'fixed'])
def test_converges_to_even_ring(n, mode):
    prm = FormationParams(center_mode=mode, omega=0.05)
    res = simulate(spawn_poses(n), prm, duration=300.0)
    m = res.metrics[-1]
    assert m.max_radius_err < 0.02
    assert m.max_gap_err_deg < 2.0
    if mode == 'fixed':
        assert m.center == pytest.approx((0.0, 0.0))
    assert res.min_separation > ROBOT_DIAMETER


def test_static_formation_comes_to_rest():
    prm = FormationParams(center_mode='fixed', omega=0.0)
    res = simulate(spawn_poses(5), prm, duration=300.0)
    p_end, p_before = res.paths[0][-1], res.paths[0][-20]
    assert math.dist(p_end, p_before) < 0.05


def test_rotating_formation_keeps_rotating_at_omega():
    prm = FormationParams(center_mode='fixed', omega=0.06, radius=2.0)
    res = simulate(spawn_poses(5), prm, duration=300.0, record_every=20)
    c = (0.0, 0.0)
    # angle swept by robot 0 base over the last 20 s of samples (1 s apart)
    a0 = math.atan2(res.paths[0][-21][1] - c[1], res.paths[0][-21][0] - c[0])
    a1 = math.atan2(res.paths[0][-1][1] - c[1], res.paths[0][-1][0] - c[0])
    swept = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
    rate = swept / (20 * 1.0)
    assert rate == pytest.approx(0.06, abs=0.01)


def test_tight_start_still_converges_without_collisions():
    # all robots start within ~0.5 m of the origin
    prm = FormationParams(center_mode='centroid', omega=0.0)
    res = simulate(spawn_poses(6, 'spiral', scale=0.3), prm, duration=300.0)
    m = res.metrics[-1]
    assert m.max_radius_err < 0.03 and m.max_gap_err_deg < 3.0
    assert res.min_separation > 0.15


def test_already_evenly_spaced_ring_stays_put():
    prm = FormationParams(center_mode='fixed', omega=0.0, radius=0.9)
    res = simulate(spawn_poses(5, 'ring', scale=0.9), prm, duration=30.0)
    assert res.metrics[-1].max_radius_err < 0.02
    assert res.metrics[-1].max_gap_err_deg < 1.0
