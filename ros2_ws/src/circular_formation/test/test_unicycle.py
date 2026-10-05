import math

import pytest

from circular_formation.unicycle import velocity_to_cmd


@pytest.mark.parametrize('yaw', [0.0, 0.9, -2.0, math.pi])
def test_control_point_velocity_is_realised(yaw):
    d = 0.15
    ux, uy = 0.08, -0.05
    v, w = velocity_to_cmd(ux, uy, yaw, d, v_max=10, w_max=100)
    # p_dot of the look-ahead point under (v, w)
    px = v * math.cos(yaw) - d * w * math.sin(yaw)
    py = v * math.sin(yaw) + d * w * math.cos(yaw)
    assert (px, py) == pytest.approx((ux, uy))


def test_limits_respected_and_ratio_preserved():
    v, w = velocity_to_cmd(1.0, 1.0, 0.0, 0.15, v_max=0.2, w_max=2.0)
    assert abs(v) <= 0.2 + 1e-12 and abs(w) <= 2.0 + 1e-12
    v0, w0 = velocity_to_cmd(1.0, 1.0, 0.0, 0.15, 1e9, 1e9)
    assert w / v == pytest.approx(w0 / v0)


def test_zero_command():
    assert velocity_to_cmd(0.0, 0.0, 1.0, 0.15, 0.2, 2.0) == (0.0, 0.0)
