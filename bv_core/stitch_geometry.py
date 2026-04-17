"""Pure-math helpers for distance-based stitch frame capture.

No ROS dependencies; safe to unit-test standalone.
"""
import math
from typing import Tuple

LatLon = Tuple[float, float]

_EARTH_R = 6378137.0  # WGS-84 semi-major axis (meters)


def compute_step_m(frame_width_px: int, fx: float, altitude_m: float, overlap: float) -> float:
    """Along-track meters between consecutive stitch captures.

    Derived from pinhole camera ground footprint:
        footprint_m = frame_width_px * altitude_m / fx
        step_m      = footprint_m * (1 - overlap)
    """
    if not (0.0 <= overlap < 1.0):
        raise ValueError(f"overlap must be in [0, 1); got {overlap}")
    footprint_m = frame_width_px * altitude_m / fx
    return footprint_m * (1.0 - overlap)


def _ll_to_local_xy(point: LatLon, origin: LatLon) -> Tuple[float, float]:
    """Equirectangular projection relative to origin. Accurate for short segments."""
    lat0 = math.radians(origin[0])
    dlat = math.radians(point[0] - origin[0])
    dlon = math.radians(point[1] - origin[1])
    x = _EARTH_R * dlon * math.cos(lat0)   # east
    y = _EARTH_R * dlat                    # north
    return x, y


def along_track_m(current: LatLon, anchor: LatLon, nxt: LatLon) -> float:
    """Signed along-track distance of `current` along segment `anchor → nxt`, in meters.

    Negative if `current` is behind the anchor.
    Projection is onto the segment direction vector; lateral drift is ignored.
    """
    cx, cy = _ll_to_local_xy(current, anchor)
    nx, ny = _ll_to_local_xy(nxt, anchor)
    seg_len2 = nx * nx + ny * ny
    if seg_len2 == 0.0:
        return 0.0
    seg_len = math.sqrt(seg_len2)
    # dot(current, seg_unit)
    return (cx * nx + cy * ny) / seg_len
