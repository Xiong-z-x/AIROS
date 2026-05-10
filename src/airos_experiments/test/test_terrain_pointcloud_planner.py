from __future__ import annotations

import math
from pathlib import Path

from airos_experiments.sdf_geometry import sample_world_cloud
from airos_experiments.terrain_pct_planner import (
    build_terrain_graph,
    plan_terrain_path,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _realistic_world() -> Path:
    return _repo_root() / 'src/airos_sim/worlds/realistic_multilevel_ramp.sdf'


def test_sdf_surface_cloud_includes_ramp_and_mezzanine_deck() -> None:
    points = sample_world_cloud(_realistic_world(), spacing=0.25)

    ramp_points = [
        point
        for point in points
        if -2.7 <= point[0] <= 0.7
        and -3.6 <= point[1] <= 3.6
        and point[3] >= 130.0
    ]
    deck_points = [
        point
        for point in points
        if -2.4 <= point[0] <= 6.0
        and 2.1 <= point[1] <= 7.5
        and 0.60 <= point[2] <= 0.72
        and point[3] >= 120.0
    ]

    assert len(points) > 20000
    assert len(ramp_points) > 300
    assert max(point[2] for point in ramp_points) - min(
        point[2] for point in ramp_points
    ) > 0.85
    assert len(deck_points) > 250


def test_terrain_pct_planner_routes_from_floor_over_ramp_to_upper_deck() -> None:
    graph = build_terrain_graph(_realistic_world(), grid_resolution=0.40)
    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(2.0, 6.8),
        start_z=0.0,
        goal_z_policy='highest',
    )
    labels = {node.surface_label for node in path}

    assert len(graph.nodes) > 1000
    assert path
    assert any('wide_access_ramp' in label for label in labels)
    assert any('mezzanine_deck_visual' in label for label in labels)
    assert max(node.z for node in path) > 0.60
    assert path[-1].z > 0.60

    for first, second in zip(path, path[1:]):
        horizontal = math.hypot(second.x - first.x, second.y - first.y)
        dz = abs(second.z - first.z)
        assert dz <= 0.36
        assert dz / max(horizontal, 1e-6) <= 0.58
