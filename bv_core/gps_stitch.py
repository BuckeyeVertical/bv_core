"""Geotag-driven stitching.

Places every frame by its own EXIF GPS fix rather than by its index in the
scan plan. `naive_stitch.py` assumes the aircraft flew the plan exactly;
this module asks each frame where it actually was. Where the two disagree
-- a row flown long, a capture that fired late, wind pushing the aircraft
off the line -- the geotag is right and the plan is not.

Still no feature matching, so like the naive path it cannot fail to
converge. It is a second floor, a little above the first.

Pure numpy and piexif on purpose. No cv2, no ROS, so it unit-tests in a
bare venv. `stitching.py` owns the warping and blending, which is the only
part that needs OpenCV.
"""

import math
import os
import re
from dataclasses import dataclass

import numpy as np
import piexif

# WGS84. The local radii of curvature below are exact to well under a
# millimetre over the few hundred metres a scan row spans, which is far
# inside GPS noise, so no projected CRS is worth the dependency.
_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3


@dataclass(frozen=True)
class FrameGeo:
    """One frame's measured position in a local tangent plane.

    east_m/north_m are metres from the survey centroid, which is an
    arbitrary origin: only differences matter, and fixing it at the centroid
    keeps the numbers small and symmetric.
    """

    row: int
    column: int
    path: str
    east_m: float
    north_m: float
    alt_m: float


def read_geotag(path: str) -> tuple[float, float, float] | None:
    """(lat, lon, alt) in degrees/metres, or None if the frame has no fix.

    Returns None rather than raising. A survey where some frames missed a
    GPS lock should still mosaic the frames that have one.
    """
    try:
        exif = piexif.load(path)
    except Exception:
        return None

    gps = exif.get("GPS") or {}
    try:
        lat = _dms_to_degrees(
            gps[piexif.GPSIFD.GPSLatitude], gps[piexif.GPSIFD.GPSLatitudeRef]
        )
        lon = _dms_to_degrees(
            gps[piexif.GPSIFD.GPSLongitude], gps[piexif.GPSIFD.GPSLongitudeRef]
        )
    except (KeyError, TypeError, ZeroDivisionError, ValueError):
        return None

    alt = 0.0
    try:
        alt = _rational_to_float(gps[piexif.GPSIFD.GPSAltitude])
        if gps.get(piexif.GPSIFD.GPSAltitudeRef) in (1, b"\x01"):
            alt = -alt
    except (KeyError, TypeError, ZeroDivisionError, ValueError):
        pass

    return lat, lon, alt


def _rational_to_float(value) -> float:
    numerator, denominator = value
    if denominator == 0:
        raise ZeroDivisionError("EXIF rational with zero denominator")
    return numerator / denominator


def _dms_to_degrees(dms, ref) -> float:
    degrees, minutes, seconds = (_rational_to_float(part) for part in dms)
    value = degrees + minutes / 60.0 + seconds / 3600.0
    if ref in ("S", "W", b"S", b"W"):
        return -value
    return value


def parse_row_column(path: str) -> tuple[int, int] | None:
    """"row3_7.jpg" -> (3, 7). Mirrors StitchingNode._parse_row_col."""
    basename = os.path.splitext(os.path.basename(path))[0]
    match = re.match(r"^row(\d+)_(\d+)$", basename, re.IGNORECASE)
    if match:
        return int(match.group(1)), int(match.group(2))
    return None


def frames_from_row_groups(row_groups: dict[int, list[str]]) -> list[FrameGeo]:
    """Read every frame's geotag and project to a common tangent plane.

    Frames without a usable fix are dropped. The returned list is sorted by
    (row, column) and shares one origin across all rows, which is what makes
    cross-row placement work without ever comparing image content.
    """
    located = []
    for row in sorted(row_groups):
        for path in row_groups[row]:
            fix = read_geotag(path)
            if fix is None:
                continue
            parsed = parse_row_column(path)
            column = parsed[1] if parsed else len(located) + 1
            located.append((row, column, path, fix))

    if not located:
        return []

    lat0 = sum(fix[0] for _, _, _, fix in located) / len(located)
    lon0 = sum(fix[1] for _, _, _, fix in located) / len(located)
    m_per_deg_lat, m_per_deg_lon = _metres_per_degree(lat0)

    frames = [
        FrameGeo(
            row=row,
            column=column,
            path=path,
            east_m=(fix[1] - lon0) * m_per_deg_lon,
            north_m=(fix[0] - lat0) * m_per_deg_lat,
            alt_m=fix[2],
        )
        for row, column, path, fix in located
    ]
    frames.sort(key=lambda frame: (frame.row, frame.column))
    return frames


def _metres_per_degree(lat_deg: float) -> tuple[float, float]:
    lat = math.radians(lat_deg)
    sin2 = math.sin(lat) ** 2
    m_per_deg_lat = (
        math.pi * _WGS84_A * (1 - _WGS84_E2)
        / (180.0 * (1 - _WGS84_E2 * sin2) ** 1.5)
    )
    m_per_deg_lon = (
        math.pi * _WGS84_A * math.cos(lat)
        / (180.0 * math.sqrt(1 - _WGS84_E2 * sin2))
    )
    return m_per_deg_lat, m_per_deg_lon


def compute_headings(frames: list[FrameGeo]) -> dict[tuple[int, int], tuple[float, float]]:
    """Unit (east, north) direction of travel at each frame.

    Measured from the frame's neighbours in its own row rather than taken
    from the plan, so a snake's alternating row directions fall out of the
    data instead of being hardcoded to row parity. Interior frames use a
    centred difference across both neighbours, which averages out the
    jitter in any single fix; the two end frames have only one neighbour
    and use that.

    A row with one frame has no measurable direction and gets due east.
    That is arbitrary, but a single frame has no neighbour to misalign with.
    """
    by_row: dict[int, list[FrameGeo]] = {}
    for frame in frames:
        by_row.setdefault(frame.row, []).append(frame)

    headings = {}
    for row, group in by_row.items():
        group.sort(key=lambda frame: frame.column)
        count = len(group)
        for index, frame in enumerate(group):
            key = (frame.row, frame.column)
            if count == 1:
                headings[key] = (1.0, 0.0)
                continue
            if index == 0:
                first, second = group[0], group[1]
            elif index == count - 1:
                first, second = group[count - 2], group[count - 1]
            else:
                first, second = group[index - 1], group[index + 1]

            east = second.east_m - first.east_m
            north = second.north_m - first.north_m
            norm = math.hypot(east, north)
            # Two captures at the same fix carry no direction. Reuse the
            # previous frame's rather than dividing by zero; within a row
            # the heading barely changes, so this is a good stand-in.
            if norm < 1e-9:
                headings[key] = headings.get(
                    (frame.row, group[index - 1].column), (1.0, 0.0)
                )
            else:
                headings[key] = (east / norm, north / norm)
    return headings


def px_per_m_from_plan(scan_plan, frame_w: int) -> float:
    """Mosaic scale in pixels per ground metre, at the frames' loaded size.

    `cross_footprint_m` is the ground width the full frame spans, computed
    by `scan_plan` from the pinhole model, so the frame's own pixels per
    metre is just `frame_w / cross_footprint_m`. Deriving it here rather
    than configuring it means the scale follows whatever downscale
    `_load_and_resize` applied, with nothing to keep in sync.
    """
    cross_footprint_m = float(scan_plan.cross_footprint_m)
    if cross_footprint_m <= 0.0:
        raise ValueError("cross_footprint_m must be positive")
    if frame_w <= 0:
        raise ValueError("frame width must be positive")
    return frame_w / cross_footprint_m


def frame_transform(
    frame: FrameGeo,
    heading: tuple[float, float],
    frame_w: int,
    frame_h: int,
    px_per_m: float,
    camera_x_sign: int = -1,
    camera_y_sign: int = -1,
) -> np.ndarray:
    """3x3 transform taking this frame's pixels into north-up mosaic pixels.

    The rotation block is deliberately unit scale: frame content maps one
    pixel to one mosaic pixel, and only the translation is multiplied by
    `px_per_m`. That is what forces `px_per_m` to be the frames' true
    pixels-per-metre -- if it is not, content and placement disagree and
    every seam shears.

    Mosaic +y points south, since a north-up raster counts rows downward,
    so the heading's north component is negated on the way in.

    camera_x_sign/camera_y_sign map the image axes onto (perpendicular,
    heading). Their product must be +1; -1 would be a mirror, not a
    rotation, and would silently produce a flipped mosaic that still looks
    plausible. For the Sony RX0 II nadir mount both are -1, i.e. image +y
    points opposite the direction of travel.
    """
    if camera_x_sign not in (1, -1) or camera_y_sign not in (1, -1):
        raise ValueError("camera signs must be 1 or -1")
    if camera_x_sign * camera_y_sign != 1:
        raise ValueError(
            "camera_x_sign * camera_y_sign must be 1; opposite signs mirror "
            "the mosaic instead of rotating it"
        )

    east, north = heading
    heading_mosaic = (east, -north)
    perpendicular = (heading_mosaic[1], -heading_mosaic[0])

    rotation = np.array(
        [
            [camera_x_sign * perpendicular[0], camera_y_sign * heading_mosaic[0]],
            [camera_x_sign * perpendicular[1], camera_y_sign * heading_mosaic[1]],
        ],
        dtype=np.float64,
    )

    centre = np.array([frame_w / 2.0, frame_h / 2.0])
    origin = np.array([px_per_m * frame.east_m, -px_per_m * frame.north_m])

    transform = np.eye(3)
    transform[:2, :2] = rotation
    transform[:2, 2] = origin - rotation @ centre
    return transform


def build_gps_transforms(
    frames: list[FrameGeo],
    frame_w: int,
    frame_h: int,
    px_per_m: float,
    camera_x_sign: int = -1,
    camera_y_sign: int = -1,
) -> list[tuple[FrameGeo, np.ndarray]]:
    """Every frame paired with its mosaic transform, sorted by (row, column)."""
    if not frames:
        raise ValueError("no geotagged frames to stitch")
    if px_per_m <= 0.0:
        raise ValueError("px_per_m must be positive")

    headings = compute_headings(frames)
    return [
        (
            frame,
            frame_transform(
                frame,
                headings[(frame.row, frame.column)],
                frame_w,
                frame_h,
                px_per_m,
                camera_x_sign,
                camera_y_sign,
            ),
        )
        for frame in frames
    ]


def canvas_bounds(
    transforms: list[np.ndarray],
    frame_w: int,
    frame_h: int,
) -> tuple[np.ndarray, int, int]:
    """Offset, width and height of the canvas holding every warped frame.

    Returns `offset` such that `offset @ transform` lands inside a canvas
    whose top-left corner is (0, 0).
    """
    corners = np.array(
        [[0, 0, 1], [frame_w, 0, 1], [frame_w, frame_h, 1], [0, frame_h, 1]],
        dtype=np.float64,
    ).T

    points = np.concatenate(
        [(transform @ corners)[:2].T for transform in transforms]
    )
    min_x, min_y = points.min(axis=0)
    max_x, max_y = points.max(axis=0)

    offset = np.array(
        [[1.0, 0.0, -min_x], [0.0, 1.0, -min_y], [0.0, 0.0, 1.0]]
    )
    width = int(math.ceil(max_x - min_x))
    height = int(math.ceil(max_y - min_y))
    return offset, width, height
