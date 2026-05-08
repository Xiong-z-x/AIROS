from __future__ import annotations

import argparse
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from airos_experiments.scan_emulator import (
    CircleObstacle,
    RectObstacle,
    _load_obstacles,
)


def _is_occupied(
    x: float,
    y: float,
    obstacles: list[RectObstacle | CircleObstacle],
    inflate: float,
) -> bool:
    for obstacle in obstacles:
        if isinstance(obstacle, RectObstacle):
            if (
                obstacle.cx - obstacle.hx - inflate <= x <= obstacle.cx + obstacle.hx + inflate
                and obstacle.cy - obstacle.hy - inflate <= y <= obstacle.cy + obstacle.hy + inflate
            ):
                return True
        else:
            dx = x - obstacle.cx
            dy = y - obstacle.cy
            if dx * dx + dy * dy <= (obstacle.radius + inflate) ** 2:
                return True
    return False


def generate_map(
    world_file: Path,
    output_prefix: Path,
    resolution: float,
    world_size: float,
    inflate: float,
) -> None:
    obstacles = _load_obstacles(world_file)
    if not obstacles:
        raise RuntimeError(f'no obstacles parsed from {world_file}')

    width = int(round(world_size / resolution))
    height = width
    origin = -world_size / 2.0

    pgm_path = output_prefix.with_suffix('.pgm')
    yaml_path = output_prefix.with_suffix('.yaml')
    pgm_path.parent.mkdir(parents=True, exist_ok=True)

    data = bytearray()
    for row in range(height):
        y = origin + (height - row - 0.5) * resolution
        for col in range(width):
            x = origin + (col + 0.5) * resolution
            data.append(0 if _is_occupied(x, y, obstacles, inflate) else 254)

    with pgm_path.open('wb') as stream:
        header = f'P5\n# AIROS generated world seed map\n{width} {height}\n255\n'
        stream.write(header.encode('ascii'))
        stream.write(data)

    yaml_text = (
        f'image: {pgm_path.name}\n'
        'mode: trinary\n'
        f'resolution: {resolution:.6f}\n'
        f'origin: [{origin:.6f}, {origin:.6f}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
    )
    yaml_path.write_text(yaml_text, encoding='ascii')


def main() -> None:
    pkg_sim = get_package_share_directory('airos_sim')
    pkg_nav = get_package_share_directory('airos_nav')
    source_nav_prefix = Path.cwd() / 'src' / 'airos_nav' / 'maps' / 'single_floor_lab'
    default_output_prefix = (
        source_nav_prefix
        if source_nav_prefix.parent.is_dir()
        else Path(pkg_nav) / 'maps' / 'single_floor_lab'
    )
    parser = argparse.ArgumentParser(
        description='Generate a 2D Nav2 seed map from AIROS SDF obstacles.'
    )
    parser.add_argument(
        '--world-file',
        default=str(Path(pkg_sim) / 'worlds' / 'single_floor_lab.sdf'),
    )
    parser.add_argument(
        '--output-prefix',
        default=str(default_output_prefix),
    )
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--world-size', type=float, default=24.0)
    parser.add_argument('--inflate', type=float, default=0.10)
    args = parser.parse_args()

    generate_map(
        world_file=Path(args.world_file),
        output_prefix=Path(args.output_prefix),
        resolution=args.resolution,
        world_size=args.world_size,
        inflate=args.inflate,
    )
    print(f'wrote {Path(args.output_prefix).with_suffix(".yaml")}')


if __name__ == '__main__':
    main()
