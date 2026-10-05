import math

import pytest

from circular_formation.spawn_layout import min_separation, spawn_poses


@pytest.mark.parametrize('n', [2, 5, 8, 12])
def test_spiral_is_collision_free_with_zero_yaw(n):
    poses = spawn_poses(n)
    assert len(poses) == n
    assert all(p[2] == 0.0 for p in poses)
    assert min_separation(poses) > 0.6


def test_ring_is_regular_polygon():
    poses = spawn_poses(5, 'ring', scale=0.9)
    assert all(math.hypot(x, y) == pytest.approx(0.9) for x, y, _ in poses)


def test_centre_offset():
    poses = spawn_poses(4, 'ring', scale=1.0, center=(3.0, -1.0))
    cx = sum(p[0] for p in poses) / 4
    cy = sum(p[1] for p in poses) / 4
    assert (cx, cy) == pytest.approx((3.0, -1.0))


def test_unknown_layout_rejected():
    with pytest.raises(ValueError):
        spawn_poses(3, 'blob')
