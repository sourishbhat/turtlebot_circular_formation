import math

import pytest

from circular_formation.geometry import (TWO_PI, all_gaps, centroid,
                                         odom_to_world, polar, quat_to_yaw,
                                         ring_gaps, wrap_pi)


def test_wrap_pi_range():
    for a in (-10.0, -math.pi, 0.0, math.pi, 3.5, 10.0):
        w = wrap_pi(a)
        assert -math.pi < w <= math.pi + 1e-12
        assert math.isclose(math.cos(w), math.cos(a), abs_tol=1e-9)
        assert math.isclose(math.sin(w), math.sin(a), abs_tol=1e-9)


def test_quat_to_yaw():
    yaw = 0.7
    assert math.isclose(
        quat_to_yaw(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)), yaw)


def test_odom_to_world_zero_spawn_yaw_is_translation():
    x, y, yaw = odom_to_world((1.0, -2.0, 0.0), (0.5, 0.25), 0.3)
    assert (x, y) == pytest.approx((1.5, -1.75))
    assert yaw == pytest.approx(0.3)


def test_odom_to_world_rotated_spawn():
    # Spawned at (1, 0) facing +y; driving 1 m "forward" in odom is +y.
    x, y, yaw = odom_to_world((1.0, 0.0, math.pi / 2), (1.0, 0.0), 0.0)
    assert (x, y) == pytest.approx((1.0, 1.0))
    assert yaw == pytest.approx(math.pi / 2)


def test_polar_and_centroid():
    assert centroid([(0, 0), (2, 0), (2, 2), (0, 2)]) == (1.0, 1.0)
    r, a = polar((0.0, -2.0), (0.0, 0.0))
    assert r == pytest.approx(2.0)
    assert a == pytest.approx(1.5 * math.pi)


def test_ring_gaps_three_robots():
    angles = [0.1, 2.0, 4.0]
    gn, gp, nxt, prv = ring_gaps(angles, [0, 1, 2], 0)
    assert (nxt, prv) == (1, 2)
    assert gn == pytest.approx(1.9)
    assert gp == pytest.approx(0.1 + TWO_PI - 4.0)
    assert gn + gp + (4.0 - 2.0) == pytest.approx(TWO_PI)


def test_ring_gaps_wraps_at_both_ends():
    angles = [0.2, 3.0, 6.0]
    # last in order: next wraps to first
    gn, _, nxt, _ = ring_gaps(angles, [0, 1, 2], 2)
    assert nxt == 0 and gn == pytest.approx(0.2 + TWO_PI - 6.0)
    # first in order: prev wraps to last
    _, gp, _, prv = ring_gaps(angles, [0, 1, 2], 0)
    assert prv == 2 and gp == pytest.approx(0.2 + TWO_PI - 6.0)


def test_ring_gaps_single_peer_sum_two_pi():
    gn, gp, _, _ = ring_gaps([1.0, 2.0], [0, 1], 0)
    assert gn + gp == pytest.approx(TWO_PI)


def test_ring_gaps_alone():
    assert ring_gaps([1.0], [0], 0)[:2] == (TWO_PI, TWO_PI)


def test_ring_gaps_tie_broken_by_id():
    # Identical angles: the order must be total and identical for every
    # viewer. Id 3 sorts before id 5, so from id 5 the "next" robot is id 3
    # a full turn away, and from id 3 the next robot (id 5) is 0 rad away.
    a, ids = [1.0, 1.0], [5, 3]
    gn0, gp0, nxt0, _ = ring_gaps(a, ids, 0)   # robot with id 5
    gn1, gp1, nxt1, _ = ring_gaps(a, ids, 1)   # robot with id 3
    assert nxt0 == 1 and nxt1 == 0
    assert gn0 == pytest.approx(TWO_PI) and gp0 == pytest.approx(0.0)
    assert gn1 == pytest.approx(0.0) and gp1 == pytest.approx(TWO_PI)


def test_all_gaps_sum_to_two_pi():
    gaps = all_gaps([0.3, 5.0, 2.2, 4.1], [0, 1, 2, 3])
    assert sum(gaps) == pytest.approx(TWO_PI)
    assert all(g >= 0 for g in gaps)
