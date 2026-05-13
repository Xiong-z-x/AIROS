from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

CloudPoint = tuple[float, float, float, float]
Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]


@dataclass(frozen=True)
class RigidTransform:
    position: tuple[float, float, float]
    rotation: Matrix3


@dataclass(frozen=True)
class BoxCollision:
    model_name: str
    link_name: str
    collision_name: str
    transform: RigidTransform
    size: tuple[float, float, float]
    static: bool

    @property
    def label(self) -> str:
        return _joined_label(
            self.model_name,
            self.link_name,
            self.collision_name,
        )


@dataclass(frozen=True)
class CylinderCollision:
    model_name: str
    link_name: str
    collision_name: str
    transform: RigidTransform
    radius: float
    length: float
    static: bool

    @property
    def label(self) -> str:
        return _joined_label(
            self.model_name,
            self.link_name,
            self.collision_name,
        )


CollisionGeometry = BoxCollision | CylinderCollision


def identity_transform() -> RigidTransform:
    return RigidTransform(
        (0.0, 0.0, 0.0),
        (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
    )


def parse_pose(text: Optional[str]) -> RigidTransform:
    values = [float(part) for part in (text or '').split()]
    while len(values) < 6:
        values.append(0.0)
    return RigidTransform(
        (values[0], values[1], values[2]),
        rotation_from_rpy(values[3], values[4], values[5]),
    )


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> Matrix3:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def compose_transform(
    parent: RigidTransform,
    child: RigidTransform,
) -> RigidTransform:
    return RigidTransform(
        add3(parent.position, matvec(parent.rotation, child.position)),
        matmul(parent.rotation, child.rotation),
    )


def transform_point(
    transform: RigidTransform,
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    return add3(transform.position, matvec(transform.rotation, point))


def inverse_transform_point(
    transform: RigidTransform,
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = point[0] - transform.position[0]
    dy = point[1] - transform.position[1]
    dz = point[2] - transform.position[2]
    rotation = transform.rotation
    return (
        rotation[0][0] * dx + rotation[1][0] * dy + rotation[2][0] * dz,
        rotation[0][1] * dx + rotation[1][1] * dy + rotation[2][1] * dz,
        rotation[0][2] * dx + rotation[1][2] * dy + rotation[2][2] * dz,
    )


def add3(
    lhs: tuple[float, float, float],
    rhs: tuple[float, float, float],
) -> tuple[float, float, float]:
    return lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]


def matvec(
    matrix: Matrix3,
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0]
        + matrix[0][1] * vector[1]
        + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0]
        + matrix[1][1] * vector[1]
        + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0]
        + matrix[2][1] * vector[1]
        + matrix[2][2] * vector[2],
    )


def matmul(lhs: Matrix3, rhs: Matrix3) -> Matrix3:
    return (
        (
            lhs[0][0] * rhs[0][0]
            + lhs[0][1] * rhs[1][0]
            + lhs[0][2] * rhs[2][0],
            lhs[0][0] * rhs[0][1]
            + lhs[0][1] * rhs[1][1]
            + lhs[0][2] * rhs[2][1],
            lhs[0][0] * rhs[0][2]
            + lhs[0][1] * rhs[1][2]
            + lhs[0][2] * rhs[2][2],
        ),
        (
            lhs[1][0] * rhs[0][0]
            + lhs[1][1] * rhs[1][0]
            + lhs[1][2] * rhs[2][0],
            lhs[1][0] * rhs[0][1]
            + lhs[1][1] * rhs[1][1]
            + lhs[1][2] * rhs[2][1],
            lhs[1][0] * rhs[0][2]
            + lhs[1][1] * rhs[1][2]
            + lhs[1][2] * rhs[2][2],
        ),
        (
            lhs[2][0] * rhs[0][0]
            + lhs[2][1] * rhs[1][0]
            + lhs[2][2] * rhs[2][0],
            lhs[2][0] * rhs[0][1]
            + lhs[2][1] * rhs[1][1]
            + lhs[2][2] * rhs[2][1],
            lhs[2][0] * rhs[0][2]
            + lhs[2][1] * rhs[1][2]
            + lhs[2][2] * rhs[2][2],
        ),
    )


def load_collision_geometries(world_file: Path) -> list[CollisionGeometry]:
    try:
        root = ET.parse(world_file).getroot()
    except (ET.ParseError, FileNotFoundError):
        return []

    world = root.find('world')
    if world is None:
        return []

    geometries: list[CollisionGeometry] = []
    for model in world.findall('model'):
        model_name = model.get('name', '')
        model_transform = parse_pose(model.findtext('pose'))
        static = (model.findtext('static') or 'false').lower() == 'true'
        for link in model.findall('link'):
            link_name = link.get('name', '')
            link_transform = compose_transform(
                model_transform,
                parse_pose(link.findtext('pose')),
            )
            for collision in link.findall('collision'):
                collision_name = collision.get('name', '')
                geometry = collision.find('geometry')
                if geometry is None:
                    continue
                collision_transform = compose_transform(
                    link_transform,
                    parse_pose(collision.findtext('pose')),
                )
                box = geometry.find('box')
                if box is not None:
                    size = _parse_xyz(box.findtext('size'))
                    geometries.append(
                        BoxCollision(
                            model_name=model_name,
                            link_name=link_name,
                            collision_name=collision_name,
                            transform=collision_transform,
                            size=size,
                            static=static,
                        )
                    )
                    continue
                cylinder = geometry.find('cylinder')
                if cylinder is None:
                    continue
                radius = cylinder.findtext('radius')
                length = cylinder.findtext('length')
                if radius is None or length is None:
                    continue
                geometries.append(
                    CylinderCollision(
                        model_name=model_name,
                        link_name=link_name,
                        collision_name=collision_name,
                        transform=collision_transform,
                        radius=float(radius),
                        length=float(length),
                        static=static,
                    )
                )
    return geometries


def is_traversable_box(box: BoxCollision) -> bool:
    label = box.label
    blocked_terms = (
        'wall',
        'partition',
        'glass',
        'rack',
        'shelf',
        'bench',
        'crate',
        'dock',
        'column',
        'pedestrian',
        'cart',
        'robot',
    )
    if any(term in label for term in blocked_terms):
        return False
    traversable_terms = (
        'floor',
        'ground',
        'ramp',
        'stair',
        'step',
        'deck',
        'platform',
        'landing',
        'walkway',
        'bridge',
    )
    return any(term in label for term in traversable_terms)


def terrain_intensity(label: str) -> float:
    if 'stair' in label or 'step' in label:
        return 150.0
    if 'ramp' in label:
        return 135.0
    if 'deck' in label or 'platform' in label or 'landing' in label:
        return 120.0
    if 'floor' in label or 'ground' in label:
        return 45.0
    return 80.0


def sample_world_cloud(
    world_file: Path,
    spacing: float,
    include_dynamic: bool = True,
    include_traversable_sides: bool = False,
) -> list[CloudPoint]:
    geometries = load_collision_geometries(world_file)
    points: list[CloudPoint] = []
    for geometry in geometries:
        if not include_dynamic and is_dynamic_geometry(geometry):
            continue
        if isinstance(geometry, BoxCollision):
            traversable = is_traversable_box(geometry)
            if traversable:
                points.extend(sample_box_top(geometry, spacing))
                if include_traversable_sides:
                    points.extend(sample_box_sides(geometry, spacing * 1.5))
            else:
                points.extend(sample_box_sides(geometry, spacing))
                points.extend(sample_box_top(geometry, spacing))
            continue
        points.extend(sample_cylinder_surface(geometry, spacing))
    return points


def sample_box_top(
    box: BoxCollision,
    spacing: float,
    margin: float = 0.0,
) -> list[CloudPoint]:
    size_x, size_y, size_z = box.size
    half_x = max(size_x / 2.0 - margin, 0.0)
    half_y = max(size_y / 2.0 - margin, 0.0)
    if half_x <= 0.0 or half_y <= 0.0:
        return []
    label = box.label
    intensity = terrain_intensity(label)
    points: list[CloudPoint] = []
    for x_local in axis_values(-half_x, half_x, spacing):
        for y_local in axis_values(-half_y, half_y, spacing):
            x, y, z = transform_point(
                box.transform,
                (x_local, y_local, size_z / 2.0),
            )
            points.append((x, y, z, intensity))
    return points


def sample_box_sides(box: BoxCollision, spacing: float) -> list[CloudPoint]:
    size_x, size_y, size_z = box.size
    half_x = size_x / 2.0
    half_y = size_y / 2.0
    label = box.label
    intensity = terrain_intensity(label) if is_traversable_box(box) else 90.0
    points: list[CloudPoint] = []
    z_values = axis_values(-size_z / 2.0, size_z / 2.0, spacing)
    traversable = is_traversable_box(box)
    for z_local in z_values:
        for x_local in axis_values(-half_x, half_x, spacing):
            for y_local in (-half_y, half_y):
                x, y, z = transform_point(
                    box.transform,
                    (x_local, y_local, z_local),
                )
                if traversable and z < 0.0:
                    continue
                points.append((x, y, z, intensity))
        for y_local in axis_values(-half_y, half_y, spacing):
            for x_local in (-half_x, half_x):
                x, y, z = transform_point(
                    box.transform,
                    (x_local, y_local, z_local),
                )
                if traversable and z < 0.0:
                    continue
                points.append((x, y, z, intensity))
    return points


def sample_cylinder_surface(
    cylinder: CylinderCollision,
    spacing: float,
) -> list[CloudPoint]:
    label = cylinder.label
    intensity = 95.0 if not is_dynamic_geometry(cylinder) else 115.0
    circumference = max(2.0 * math.pi * cylinder.radius, spacing)
    angle_count = max(int(math.ceil(circumference / spacing)), 12)
    z_values = axis_values(-cylinder.length / 2.0, cylinder.length / 2.0, spacing)
    points: list[CloudPoint] = []
    for z_local in z_values:
        for index in range(angle_count):
            angle = 2.0 * math.pi * index / angle_count
            x, y, z = transform_point(
                cylinder.transform,
                (
                    math.cos(angle) * cylinder.radius,
                    math.sin(angle) * cylinder.radius,
                    z_local,
                ),
            )
            points.append((x, y, z, intensity))
    top_z = cylinder.length / 2.0
    radius_values = axis_values(0.0, cylinder.radius, spacing)
    for radius in radius_values:
        for index in range(angle_count):
            angle = 2.0 * math.pi * index / angle_count
            x, y, z = transform_point(
                cylinder.transform,
                (math.cos(angle) * radius, math.sin(angle) * radius, top_z),
            )
            points.append((x, y, z, intensity))
    if label:
        return points
    return points


def is_dynamic_geometry(geometry: CollisionGeometry) -> bool:
    return not geometry.static or 'dynamic' in geometry.label


def axis_values(start: float, stop: float, spacing: float) -> list[float]:
    if spacing <= 0.0:
        raise ValueError('spacing must be positive')
    length = max(stop - start, 0.0)
    count = max(int(math.ceil(length / spacing)), 1)
    return [start + min(index * spacing, length) for index in range(count + 1)]


def iter_traversable_boxes(
    geometries: Iterable[CollisionGeometry],
) -> Iterable[BoxCollision]:
    for geometry in geometries:
        if isinstance(geometry, BoxCollision) and is_traversable_box(geometry):
            yield geometry


def iter_obstacle_geometries(
    geometries: Iterable[CollisionGeometry],
) -> Iterable[CollisionGeometry]:
    for geometry in geometries:
        if isinstance(geometry, BoxCollision) and is_traversable_box(geometry):
            continue
        yield geometry


def _parse_xyz(text: Optional[str]) -> tuple[float, float, float]:
    parts = [float(part) for part in (text or '').split()]
    while len(parts) < 3:
        parts.append(0.0)
    return parts[0], parts[1], parts[2]


def _joined_label(*parts: str) -> str:
    return '/'.join(part.lower() for part in parts if part)
