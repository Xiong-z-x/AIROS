from __future__ import annotations

from pathlib import Path

from airos_experiments.pointcloud_colorizer import _height_rgb
from airos_experiments.pointcloud_colorizer import _slam_map_rgb
from airos_experiments.pointcloud_colorizer import colorize_points


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _read_text(relative_path: str) -> str:
    return (_repo_root() / relative_path).read_text(encoding='utf-8')


def test_slam_map_palette_uses_distinct_surface_colors() -> None:
    low = _height_rgb(-10.0, -0.4, 2.2)
    ramp = _height_rgb(0.30, -0.4, 2.2)
    deck = _height_rgb(0.70, -0.4, 2.2)
    high = _height_rgb(10.0, -0.4, 2.2)

    assert abs(low[0] - low[1]) <= 12
    assert abs(low[1] - low[2]) <= 12
    assert ramp[1] > ramp[0]
    assert ramp[1] > ramp[2]
    assert deck[0] > deck[1] > deck[2]
    assert high[0] > high[1]
    assert high[0] > high[2]


def test_slam_map_palette_adds_subtle_xy_texture() -> None:
    first = _slam_map_rgb(0.0, 0.0, 0.0, -0.4, 2.2)
    second = _slam_map_rgb(0.9, 0.9, 0.0, -0.4, 2.2)

    assert first != second


def test_colorize_points_keeps_xyz_and_adds_rgb_float() -> None:
    colored = colorize_points(
        [(1.0, 2.0, -0.4), (3.0, 4.0, 2.2)],
        min_z=-0.4,
        max_z=2.2,
    )

    assert colored[0][:3] == (1.0, 2.0, -0.4)
    assert colored[1][:3] == (3.0, 4.0, 2.2)
    assert isinstance(colored[0][3], float)


def test_colorize_points_can_hide_low_ground_layer() -> None:
    colored = colorize_points(
        [(0.0, 0.0, 0.0), (1.0, 1.0, 0.12), (2.0, 2.0, 0.70)],
        min_z=-0.4,
        max_z=2.2,
        min_visible_z=0.08,
    )

    assert [point[:3] for point in colored] == [
        (1.0, 1.0, 0.12),
        (2.0, 2.0, 0.70),
    ]


def test_colorizer_publishes_reliable_for_rviz_compatibility() -> None:
    source = _read_text(
        'src/airos_experiments/airos_experiments/pointcloud_colorizer.py'
    )

    assert 'publish_qos = QoSProfile' in source
    assert 'reliability=ReliabilityPolicy.RELIABLE' in source
    assert "self.declare_parameter('min_visible_z', 0.03)" in source
