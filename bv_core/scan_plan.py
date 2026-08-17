"""Camera-aware snake scan planning."""

import math
import os
from dataclasses import dataclass

import yaml

from .mission_config import mission_config_path
from .stitch_geometry import compute_step_m, distance_m


@dataclass(frozen=True)
class ScanPlan:
    waypoints: list[list[float]]
    altitude_m: float
    overlap: float
    capture_spacing_m: float
    row_spacing_m: float
    row_count: int
    cross_footprint_m: float = 0.0


def build_scan_plan(mission, vision, camera) -> ScanPlan:
    altitude = float(mission.get('scan_altitude', mission['takeoff_alt']))
    overlap = float(vision.get('stitch_overlap', 0.35))
    width = int(camera['image_width_px'])
    height = int(camera['image_height_px'])
    intrinsics = camera['c_matrix']
    fx = float(intrinsics[0])
    fy = float(intrinsics[4])

    if altitude <= 0.0:
        raise ValueError("scan_altitude must be positive")
    if width <= 0 or height <= 0 or fx <= 0.0 or fy <= 0.0:
        raise ValueError(
            "camera dimensions and focal lengths must be positive"
        )

    # A forward-aligned nadir camera uses image width across the flight path.
    cross_footprint_m = width * altitude / fx
    capture_spacing_m = compute_step_m(height, fy, altitude, overlap)

    boundary = mission.get('scan_boundary')
    explicit_points = mission.get('scan_points', [])
    if boundary is None:
        return ScanPlan(
            waypoints=[list(point) for point in explicit_points],
            altitude_m=altitude,
            overlap=overlap,
            capture_spacing_m=capture_spacing_m,
            row_spacing_m=0.0,
            row_count=len(explicit_points) // 2,
            cross_footprint_m=cross_footprint_m,
        )
    if explicit_points:
        raise ValueError("use scan_boundary or scan_points, not both")

    waypoints, row_spacing_m = _snake_waypoints(
        boundary,
        altitude,
        cross_footprint_m,
        overlap,
    )
    return ScanPlan(
        waypoints=waypoints,
        altitude_m=altitude,
        overlap=overlap,
        capture_spacing_m=capture_spacing_m,
        row_spacing_m=row_spacing_m,
        row_count=len(waypoints) // 2,
        cross_footprint_m=cross_footprint_m,
    )


def load_scan_plan() -> ScanPlan:
    from ament_index_python.packages import get_package_share_directory

    config_dir = os.path.join(get_package_share_directory('bv_core'), 'config')
    with open(mission_config_path(), 'r') as stream:
        mission = yaml.safe_load(stream)
    with open(os.path.join(config_dir, 'vision_params.yaml'), 'r') as stream:
        vision = yaml.safe_load(stream)
    camera_path = os.path.join(config_dir, 'filtering_params.yaml')
    with open(camera_path, 'r') as stream:
        camera = yaml.safe_load(stream)
    return build_scan_plan(mission, vision, camera)


def _snake_waypoints(boundary, altitude, footprint_m, overlap):
    points = [_lat_lon(point) for point in boundary]
    if len(points) != 4:
        raise ValueError("scan_boundary must contain four ordered corners")

    pair_a = (
        distance_m(points[0], points[1])
        + distance_m(points[2], points[3])
    ) / 2.0
    pair_b = (
        distance_m(points[1], points[2])
        + distance_m(points[3], points[0])
    ) / 2.0
    if pair_a >= pair_b:
        starts = (points[0], points[3])
        ends = (points[1], points[2])
    else:
        starts = (points[1], points[0])
        ends = (points[2], points[3])

    cross_span_m = (
        distance_m(starts[0], starts[1]) + distance_m(ends[0], ends[1])
    ) / 2.0
    offsets_m, row_spacing_m = _row_offsets(cross_span_m, footprint_m, overlap)

    waypoints = []
    for index, offset_m in enumerate(offsets_m):
        fraction = 0.5 if cross_span_m == 0.0 else offset_m / cross_span_m
        start = _interpolate(starts[0], starts[1], fraction)
        end = _interpolate(ends[0], ends[1], fraction)
        if index % 2:
            start, end = end, start
        waypoints.extend([
            [start[0], start[1], altitude],
            [end[0], end[1], altitude],
        ])

    return waypoints, row_spacing_m


def _row_offsets(span_m, footprint_m, overlap):
    if not 0.0 <= overlap < 1.0:
        raise ValueError("stitch_overlap must be in [0, 1)")
    if span_m <= footprint_m:
        return [span_m / 2.0], 0.0

    center_span_m = span_m - footprint_m
    maximum_spacing_m = footprint_m * (1.0 - overlap)
    intervals = max(1, math.ceil(center_span_m / maximum_spacing_m))
    spacing_m = center_span_m / intervals
    first_center_m = footprint_m / 2.0
    offsets = [
        first_center_m + index * spacing_m
        for index in range(intervals + 1)
    ]
    return offsets, spacing_m


def _lat_lon(point):
    if len(point) != 2:
        raise ValueError(
            "scan boundary corners must contain latitude and longitude"
        )
    return float(point[0]), float(point[1])


def _interpolate(start, end, fraction):
    return (
        start[0] + (end[0] - start[0]) * fraction,
        start[1] + (end[1] - start[1]) * fraction,
    )
