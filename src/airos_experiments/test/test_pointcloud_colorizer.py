from __future__ import annotations

from airos_experiments.pointcloud_colorizer import _height_rgb
from airos_experiments.pointcloud_colorizer import colorize_points


def test_height_color_ramp_is_clamped_and_monotonic_enough() -> None:
    low = _height_rgb(-10.0, -0.4, 2.2)
    mid = _height_rgb(0.9, -0.4, 2.2)
    high = _height_rgb(10.0, -0.4, 2.2)

    assert low[2] >= low[0]
    assert mid[1] >= 120
    assert high[0] == 255


def test_colorize_points_keeps_xyz_and_adds_rgb_float() -> None:
    colored = colorize_points(
        [(1.0, 2.0, -0.4), (3.0, 4.0, 2.2)],
        min_z=-0.4,
        max_z=2.2,
    )

    assert colored[0][:3] == (1.0, 2.0, -0.4)
    assert colored[1][:3] == (3.0, 4.0, 2.2)
    assert isinstance(colored[0][3], float)
