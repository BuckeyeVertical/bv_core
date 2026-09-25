"""Tests for DEM elevation lookup and terrain-referenced scan altitudes."""

import numpy as np
import pytest

from bv_core.terrain import (
    TerrainModel,
    apply_terrain_altitudes,
)


# A 4x4 grid of 0.001-degree cells whose origin sits at the SUAS field. Row 0
# is the northern edge, so latitude decreases as the row index grows.
ORIGIN_LON = -96.010
ORIGIN_LAT = 36.220
CELL = 0.001
NODATA = -9999.0

ELEVATIONS = np.array([
    [100.0, 110.0, 120.0, 130.0],
    [200.0, 210.0, 220.0, 230.0],
    [300.0, 310.0, 320.0, 330.0],
    [400.0, 410.0, 420.0, NODATA],
], dtype=np.float32)


class FakeAffine:
    """The inverse of a north-up GeoTIFF transform.

    Real rasterio hands back an ``Affine`` supporting ``* (x, y)``; only that
    one operation is exercised here, so reimplementing it keeps these tests
    runnable without rasterio installed.
    """

    def __mul__(self, point):
        """Map a projected coordinate to fractional column/row."""
        x, y = point
        return ((x - ORIGIN_LON) / CELL, (ORIGIN_LAT - y) / CELL)


def model(nodata=NODATA):
    """Build a TerrainModel over the fixture grid."""
    return TerrainModel(
        elevations=ELEVATIONS,
        inverse_transform=FakeAffine(),
        crs=None,
        nodata=nodata,
        path='<fixture>',
    )


def at(row, column):
    """Return the lat/lon of one cell's center."""
    return (
        ORIGIN_LAT - (row + 0.5) * CELL,
        ORIGIN_LON + (column + 0.5) * CELL,
    )


def test_cell_center_returns_that_cells_elevation():
    terrain = model()

    assert terrain.elevation_at(*at(0, 0)) == pytest.approx(100.0)
    assert terrain.elevation_at(*at(1, 2)) == pytest.approx(220.0)
    assert terrain.elevation_at(*at(2, 1)) == pytest.approx(310.0)


def test_midpoint_between_cells_is_interpolated_not_snapped():
    terrain = model()
    lat, lon = at(0, 0)

    # Halfway east toward the 110 m cell, and halfway south toward the 200 m
    # one. Nearest-neighbour sampling would return 100.0 for both.
    assert terrain.elevation_at(lat, lon + CELL / 2) == pytest.approx(105.0)
    assert terrain.elevation_at(lat - CELL / 2, lon) == pytest.approx(150.0)
    assert terrain.elevation_at(
        lat - CELL / 2, lon + CELL / 2) == pytest.approx(155.0)


def test_points_outside_the_raster_report_a_miss():
    terrain = model()

    assert terrain.elevation_at(ORIGIN_LAT + 1.0, ORIGIN_LON) is None
    assert terrain.elevation_at(ORIGIN_LAT, ORIGIN_LON - 1.0) is None
    assert terrain.elevation_at(ORIGIN_LAT - 5 * CELL, ORIGIN_LON) is None


def test_the_outer_half_cell_rim_reads_as_its_edge_cell():
    """The rim has no opposite neighbour; it must not fail the whole route."""
    terrain = model()

    # The exact north-west corner of the raster, half a cell outside the
    # center of cell (0, 0).
    assert terrain.elevation_at(
        ORIGIN_LAT, ORIGIN_LON) == pytest.approx(100.0)
    # Half a cell west of the (1, 0) center, still inside the extent.
    lat, _ = at(1, 0)
    assert terrain.elevation_at(lat, ORIGIN_LON) == pytest.approx(200.0)


def test_nodata_cells_report_a_miss():
    terrain = model()

    assert terrain.elevation_at(*at(3, 3)) is None


def test_a_block_touching_nodata_falls_back_to_the_nearest_cell():
    """Interpolating across a hole would invent an elevation."""
    terrain = model()

    # Midway between cell (3, 2) at 420 m and the nodata cell beside it.
    # Nearest-cell fallback keeps (3, 2); a bilinear blend would not.
    lat, lon = at(3, 2)
    assert terrain.elevation_at(
        lat, lon + CELL * 0.4) == pytest.approx(420.0)
    # Past the midpoint the nearest cell is the nodata one, so it is a miss.
    assert terrain.elevation_at(lat, lon + CELL * 0.6) is None


def test_nan_cells_report_a_miss():
    nan_grid = ELEVATIONS.copy()
    nan_grid[0, 0] = np.nan
    nan_grid[0, 1] = np.nan
    nan_grid[1, 0] = np.nan
    nan_grid[1, 1] = np.nan
    terrain = TerrainModel(
        elevations=nan_grid,
        inverse_transform=FakeAffine(),
        crs=None,
        nodata=None,
        path='<fixture>',
    )
    assert terrain.elevation_at(*at(0, 0)) is None


def test_msl_for_agl_adds_the_requested_height():
    terrain = model()

    assert terrain.msl_for_agl(*at(1, 1), 48.768) == pytest.approx(258.768)
    assert terrain.msl_for_agl(*at(3, 3), 48.768) is None


def test_every_waypoint_is_converted_to_amsl():
    waypoints = [list(at(0, 0)) + [48.768], list(at(2, 2)) + [48.768]]

    converted, applied = apply_terrain_altitudes(waypoints, 48.768, model())

    assert applied is True
    assert converted[0][2] == pytest.approx(148.768)
    assert converted[1][2] == pytest.approx(368.768)
    # Latitude and longitude ride through untouched.
    assert converted[0][:2] == pytest.approx(list(at(0, 0)))


def test_one_missing_waypoint_discards_the_whole_conversion():
    """A route is pushed under a single frame, so a partial fix is unsafe."""
    good = list(at(0, 0)) + [48.768]
    outside = [ORIGIN_LAT + 1.0, ORIGIN_LON, 48.768]
    waypoints = [good, outside, list(at(2, 2)) + [48.768]]

    converted, applied = apply_terrain_altitudes(waypoints, 48.768, model())

    assert applied is False
    assert converted is waypoints
    assert converted[0][2] == pytest.approx(48.768)


def test_no_terrain_model_leaves_waypoints_alone():
    waypoints = [list(at(0, 0)) + [48.768]]

    converted, applied = apply_terrain_altitudes(waypoints, 48.768, None)

    assert applied is False
    assert converted is waypoints


def test_empty_route_is_not_reported_as_terrain_referenced():
    converted, applied = apply_terrain_altitudes([], 48.768, model())

    assert applied is False
    assert converted == []
