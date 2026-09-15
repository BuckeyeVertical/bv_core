"""Terrain elevation lookup from a DEM GeoTIFF.

The scan route is planned at a single above-ground altitude, but PX4 flies a
mission waypoint at either a fixed offset from the takeoff point or at a true
AMSL altitude.  Over anything but flat ground those are not the same thing, so
this module converts the configured AGL into a per-waypoint AMSL altitude using
a digital elevation model.

Every entry point here fails soft: a missing rasterio, a missing ``dem.tif`` or
a waypoint the raster does not cover leaves the caller with the flat altitudes
it started with, so a bad DEM never grounds the aircraft.
"""

import math
import os
from dataclasses import dataclass

import numpy as np


DEM_FILENAME = 'dem.tif'
DEM_PATH_ENV = 'BV_DEM_PATH'


def dem_path():
    """Return the DEM location, honoring the environment override."""
    override = os.environ.get(DEM_PATH_ENV)
    if override:
        return override

    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory('bv_core'), 'config', DEM_FILENAME)


@dataclass(frozen=True)
class TerrainModel:
    """Elevation samples from a DEM raster, indexed by latitude/longitude.

    ``elevations`` is the full first band held in memory; a field-sized DEM is
    a few megapixels at most, and keeping it resident means a waypoint lookup
    never touches the disk while the mission is running.
    """

    elevations: np.ndarray
    inverse_transform: object
    crs: object
    nodata: float
    path: str

    def _raster_xy(self, lat, lon):
        """Project one latitude/longitude into the raster's own CRS.

        A geographic DEM needs no projection at all, and short-circuiting
        keeps rasterio off this path entirely - which is what lets the lookup
        maths be tested without it installed.
        """
        if self.crs is None or _is_wgs84(self.crs):
            return float(lon), float(lat)

        from rasterio.crs import CRS
        from rasterio.warp import transform

        xs, ys = transform(
            CRS.from_epsg(4326), self.crs, [float(lon)], [float(lat)])
        return float(xs[0]), float(ys[0])

    def _sample(self, row, column):
        """Return one cell's elevation, or ``None`` when it carries no data."""
        height, width = self.elevations.shape
        if not (0 <= row < height and 0 <= column < width):
            return None
        value = float(self.elevations[row, column])
        if np.isnan(value):
            return None
        if self.nodata is not None and value == self.nodata:
            return None
        return value

    def elevation_at(self, lat, lon):
        """Return the ground elevation at this point, or ``None``.

        The four surrounding cells are interpolated bilinearly. Sampling the
        nearest cell instead would quantize the commanded altitude to the DEM
        grid, and on a 30 m model that shows up as altitude steps between
        waypoints on the same row.

        Two edge cases are deliberately forgiving, because the alternative is
        discarding terrain following for the whole route over one point. A
        position inside the raster but within the outer half-cell rim has no
        opposite neighbour, so the interpolation block is clamped to the grid
        and the fractions saturate - the rim reads as its edge cells. And a
        block that overlaps nodata falls back to the nearest valid cell rather
        than interpolating across the hole.
        """
        x, y = self._raster_xy(lat, lon)
        column, row = self.inverse_transform * (x, y)
        height, width = self.elevations.shape
        if not (0.0 <= column <= width and 0.0 <= row <= height):
            return None

        # Cell centers sit at the half-pixel, so shift before interpolating.
        column -= 0.5
        row -= 0.5
        column_0 = _clamp_index(math.floor(column), width)
        row_0 = _clamp_index(math.floor(row), height)
        column_fraction = min(max(column - column_0, 0.0), 1.0)
        row_fraction = min(max(row - row_0, 0.0), 1.0)

        corners = [
            self._sample(row_0, column_0),
            self._sample(row_0, column_0 + 1),
            self._sample(row_0 + 1, column_0),
            self._sample(row_0 + 1, column_0 + 1),
        ]
        if any(corner is None for corner in corners):
            return self._sample(
                min(max(round(row), 0), height - 1),
                min(max(round(column), 0), width - 1))

        top = corners[0] + (corners[1] - corners[0]) * column_fraction
        bottom = corners[2] + (corners[3] - corners[2]) * column_fraction
        return top + (bottom - top) * row_fraction

    def msl_for_agl(self, lat, lon, agl_m):
        """Return the AMSL altitude that flies ``agl_m`` above this point."""
        elevation = self.elevation_at(lat, lon)
        if elevation is None:
            return None
        return elevation + float(agl_m)


def load_terrain_model(logger=None):
    """Load the DEM, or return ``None`` after explaining why it is unusable."""
    try:
        path = dem_path()
    except Exception as error:
        _warn(logger, f"Terrain following disabled, no DEM path: {error}")
        return None

    if not os.path.isfile(path):
        _warn(
            logger,
            f"Terrain following disabled, no DEM at {path} - scan altitudes "
            f"stay relative to the takeoff point")
        return None

    try:
        import rasterio
    except ImportError as error:
        _warn(
            logger,
            f"Terrain following disabled, rasterio is not installed "
            f"({error}) - scan altitudes stay relative to the takeoff point")
        return None

    try:
        with rasterio.open(path) as source:
            elevations = source.read(1)
            model = TerrainModel(
                elevations=elevations,
                inverse_transform=~source.transform,
                crs=source.crs,
                nodata=source.nodata,
                path=path,
            )
    except Exception as error:
        _warn(logger, f"Terrain following disabled, cannot read {path}: "
                      f"{error!r}")
        return None

    _info(
        logger,
        f"Loaded DEM {path}: {elevations.shape[1]}x{elevations.shape[0]} "
        f"cells, crs={model.crs}")
    return model


def apply_terrain_altitudes(waypoints, agl_m, terrain, logger=None):
    """Rewrite waypoint altitudes as AMSL, all of them or none.

    Returns ``(waypoints, applied)``.  A route is pushed to the autopilot under
    one altitude frame, so a partial conversion would leave some waypoints
    AMSL and the rest above the takeoff point inside a single mission - PX4
    cannot express that and would silently fly the wrong altitudes.  One failed
    lookup therefore discards the whole conversion.
    """
    if terrain is None:
        return waypoints, False
    if not waypoints:
        return waypoints, False

    converted = []
    for index, point in enumerate(waypoints):
        lat, lon = float(point[0]), float(point[1])
        msl = terrain.msl_for_agl(lat, lon, agl_m)
        if msl is None:
            _warn(
                logger,
                f"Terrain following disabled: waypoint {index} "
                f"({lat:.6f}, {lon:.6f}) is outside the DEM or has no data - "
                f"the whole scan route stays relative to the takeoff point")
            return waypoints, False
        converted.append([lat, lon, msl])

    altitudes = [point[2] for point in converted]
    _info(
        logger,
        f"Terrain following ON: {len(converted)} scan waypoints at "
        f"{float(agl_m):.1f}m AGL, AMSL {min(altitudes):.1f}-"
        f"{max(altitudes):.1f}m ({max(altitudes) - min(altitudes):.1f}m of "
        f"relief)")
    return converted, True


def _clamp_index(index, size):
    """Clamp a block's first index so the 2x2 block stays on the grid."""
    return min(max(int(index), 0), max(size - 2, 0))


def _is_wgs84(crs):
    """Return whether this CRS is plain lat/lon on WGS84."""
    try:
        return crs.to_epsg() == 4326
    except Exception:
        return False


def _info(logger, message):
    """Log an informational message when a logger was supplied."""
    if logger is not None:
        logger.info(message)


def _warn(logger, message):
    """Log a warning when a logger was supplied."""
    if logger is not None:
        logger.warn(message)
