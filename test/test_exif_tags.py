#!/usr/bin/env python3
"""EXIF geotagging of stitch frames.

Pure metadata construction plus one real round trip through a JPEG on disk.
No ROS and no exiftool, so this runs anywhere the repo does.

The numbers are checked against the real calibration in
config/filtering_params.yaml: a tag set that disagreed with the matrix
localizer.py projects with would hand ODM a focal length never flown.
"""

import math
import os
import sys
from datetime import datetime

import piexif
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.frame_metadata import (  # noqa: E402
    CameraExifProfile,
    FrameGeotag,
    build_exif_bytes,
    heading_deg_from_quaternion,
)

# config/filtering_params.yaml, the real RX0 II calibration.
FX = 2296.4008369677636
FY = 2312.535785524523
CALIB_W = 3840
CALIB_H = 2160
SENSOR_W_MM = 13.2


def profile():
    return CameraExifProfile.from_calibration(
        make="SONY",
        model="DSC-RX0M2",
        fx=FX,
        fy=FY,
        calib_width_px=CALIB_W,
        calib_height_px=CALIB_H,
        sensor_width_mm=SENSOR_W_MM,
    )


def geotag(**overrides):
    fields = dict(
        latitude=38.3877,
        longitude=-76.4190,
        altitude_m=102.46,
        altitude_is_msl=True,
        captured_at=datetime(2026, 9, 16, 14, 23, 5, 123456),
        horizontal_error_m=1.204,
        heading_deg=87.3,
    )
    fields.update(overrides)
    return FrameGeotag(**fields)


def ratio(value):
    """Collapse an EXIF RATIONAL into a float."""
    numerator, denominator = value
    return numerator / denominator


def dms(value):
    """Collapse an EXIF degree/minute/second triple into decimal degrees."""
    degrees, minutes, seconds = (ratio(part) for part in value)
    return degrees + minutes / 60.0 + seconds / 3600.0


# Static camera tags

def test_focal_length_is_derived_from_fx_and_sensor_width():
    tags = profile().exif_dict(CALIB_W, CALIB_H)['Exif']
    assert ratio(tags[piexif.ExifIFD.FocalLength]) == pytest.approx(
        7.893877, abs=1e-5)


def test_focal_plane_x_resolution_is_pixels_per_inch_of_sensor():
    tags = profile().exif_dict(CALIB_W, CALIB_H)['Exif']
    assert ratio(tags[piexif.ExifIFD.FocalPlaneXResolution]) == pytest.approx(
        7389.0909, abs=1e-3)


def test_focal_plane_ratio_encodes_the_calibrated_fy_over_fx():
    tags = profile().exif_dict(CALIB_W, CALIB_H)['Exif']
    encoded = (ratio(tags[piexif.ExifIFD.FocalPlaneYResolution])
               / ratio(tags[piexif.ExifIFD.FocalPlaneXResolution]))
    assert encoded == pytest.approx(FY / FX, rel=1e-7)


def test_resolution_unit_declares_inches():
    tags = profile().exif_dict(CALIB_W, CALIB_H)['Exif']
    assert tags[piexif.ExifIFD.FocalPlaneResolutionUnit] == 2


def test_focal_plane_resolution_follows_the_actual_image_size():
    half = profile().exif_dict(CALIB_W // 2, CALIB_H // 2)['Exif']
    full = profile().exif_dict(CALIB_W, CALIB_H)['Exif']
    assert (ratio(half[piexif.ExifIFD.FocalPlaneXResolution])
            == pytest.approx(
                ratio(full[piexif.ExifIFD.FocalPlaneXResolution]) / 2.0))


def test_focal_length_is_independent_of_the_actual_image_size():
    half = profile().exif_dict(CALIB_W // 2, CALIB_H // 2)['Exif']
    full = profile().exif_dict(CALIB_W, CALIB_H)['Exif']
    assert (half[piexif.ExifIFD.FocalLength]
            == full[piexif.ExifIFD.FocalLength])


def test_make_and_model_are_written_as_bytes():
    tags = profile().exif_dict(CALIB_W, CALIB_H)['0th']
    assert tags[piexif.ImageIFD.Make] == b'SONY'
    assert tags[piexif.ImageIFD.Model] == b'DSC-RX0M2'


# Heading

def test_identity_quaternion_points_the_body_x_axis_east():
    assert heading_deg_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(90.0)


def test_quarter_turn_about_z_points_the_body_x_axis_north():
    half = math.sqrt(0.5)
    assert heading_deg_from_quaternion(0.0, 0.0, half, half) == pytest.approx(0.0)


def test_heading_stays_in_zero_to_three_sixty():
    half = math.sqrt(0.5)
    assert heading_deg_from_quaternion(
        0.0, 0.0, -half, half) == pytest.approx(180.0)


# Per-frame GPS tags

def test_northern_latitude_is_unsigned_with_an_n_ref():
    gps = geotag().exif_dict()['GPS']
    assert dms(gps[piexif.GPSIFD.GPSLatitude]) == pytest.approx(
        38.3877, abs=1e-7)
    assert gps[piexif.GPSIFD.GPSLatitudeRef] == b'N'


def test_western_longitude_is_unsigned_with_a_w_ref():
    gps = geotag().exif_dict()['GPS']
    assert dms(gps[piexif.GPSIFD.GPSLongitude]) == pytest.approx(
        76.4190, abs=1e-7)
    assert gps[piexif.GPSIFD.GPSLongitudeRef] == b'W'


def test_southern_and_eastern_refs_follow_the_signs():
    gps = geotag(latitude=-33.87, longitude=151.21).exif_dict()['GPS']
    assert dms(gps[piexif.GPSIFD.GPSLatitude]) == pytest.approx(
        33.87, abs=1e-7)
    assert gps[piexif.GPSIFD.GPSLatitudeRef] == b'S'
    assert dms(gps[piexif.GPSIFD.GPSLongitude]) == pytest.approx(
        151.21, abs=1e-7)
    assert gps[piexif.GPSIFD.GPSLongitudeRef] == b'E'


def test_altitude_below_sea_level_is_unsigned_with_ref_one():
    gps = geotag(altitude_m=-12.5).exif_dict()['GPS']
    assert ratio(gps[piexif.GPSIFD.GPSAltitude]) == pytest.approx(
        12.5, abs=1e-3)
    assert gps[piexif.GPSIFD.GPSAltitudeRef] == 1


def test_capture_time_uses_the_exif_datetime_format():
    exif = geotag().exif_dict()['Exif']
    assert exif[piexif.ExifIFD.DateTimeOriginal] == b'2026:09:16 14:23:05'


def test_sub_second_time_is_carried_separately():
    exif = geotag().exif_dict()['Exif']
    assert exif[piexif.ExifIFD.SubSecTimeOriginal] == b'123'


def test_horizontal_error_is_omitted_when_the_fix_reports_none():
    gps = geotag(horizontal_error_m=None).exif_dict()['GPS']
    assert piexif.GPSIFD.GPSHPositioningError not in gps


def test_heading_is_written_as_a_true_north_image_direction():
    gps = geotag().exif_dict()['GPS']
    assert ratio(gps[piexif.GPSIFD.GPSImgDirection]) == pytest.approx(
        87.3, abs=1e-3)
    assert gps[piexif.GPSIFD.GPSImgDirectionRef] == b'T'


def test_image_direction_is_omitted_entirely_without_a_heading():
    gps = geotag(heading_deg=None).exif_dict()['GPS']
    assert piexif.GPSIFD.GPSImgDirection not in gps
    assert piexif.GPSIFD.GPSImgDirectionRef not in gps


# Round trip through a real JPEG on disk

@pytest.fixture
def jpeg(tmp_path):
    """A real JPEG, standing in for a written stitch frame."""
    from PIL import Image
    path = tmp_path / "row3_7.jpg"
    Image.new('RGB', (640, 360), (40, 90, 140)).save(path, quality=95)
    return str(path)


def test_tags_survive_a_write_and_read_back(jpeg):
    blob = build_exif_bytes(profile(), geotag(), 640, 360)
    piexif.insert(blob, jpeg)

    loaded = piexif.load(jpeg)
    assert dms(loaded['GPS'][piexif.GPSIFD.GPSLatitude]) == pytest.approx(
        38.3877, abs=1e-7)
    assert loaded['GPS'][piexif.GPSIFD.GPSLatitudeRef] == b'N'
    assert ratio(loaded['GPS'][piexif.GPSIFD.GPSAltitude]) == pytest.approx(
        102.46, abs=1e-3)
    assert loaded['0th'][piexif.ImageIFD.Model] == b'DSC-RX0M2'
    assert ratio(loaded['Exif'][piexif.ExifIFD.FocalLength]) == pytest.approx(
        7.893877, abs=1e-5)
    assert (loaded['Exif'][piexif.ExifIFD.DateTimeOriginal]
            == b'2026:09:16 14:23:05')


def test_tagging_does_not_re_encode_the_pixels(jpeg):
    """Generation loss would degrade the very frames ODM reconstructs from."""
    import numpy as np
    from PIL import Image
    before = np.asarray(Image.open(jpeg)).copy()

    piexif.insert(build_exif_bytes(profile(), geotag(), 640, 360), jpeg)

    after = np.asarray(Image.open(jpeg))
    assert np.array_equal(before, after)


# Regressions found by independent audit of a real flight's frames

def test_ellipsoidal_altitude_is_not_labelled_above_sea_level():
    """GPSAltitudeRef=0 asserts MSL. Ellipsoidal height is ~30 m from it."""
    gps = geotag(altitude_is_msl=False).exif_dict()['GPS']
    assert piexif.GPSIFD.GPSAltitude not in gps
    assert piexif.GPSIFD.GPSAltitudeRef not in gps


def test_msl_altitude_is_still_written():
    gps = geotag(altitude_is_msl=True).exif_dict()['GPS']
    assert ratio(gps[piexif.GPSIFD.GPSAltitude]) == pytest.approx(102.46, abs=1e-3)
    assert gps[piexif.GPSIFD.GPSAltitudeRef] == 0


def test_seconds_rounding_never_emits_sixty():
    """59.99996 arcsec must carry into minutes, not encode as 60 arcsec."""
    lat = 36.0 + 59.0 / 60.0 + 59.99996 / 3600.0
    gps = geotag(latitude=lat).exif_dict()['GPS']
    deg, minute, sec = gps[piexif.GPSIFD.GPSLatitude]
    assert ratio(sec) < 60.0, f"seconds field encoded as {ratio(sec)}"
    assert ratio(minute) < 60.0
    assert dms(gps[piexif.GPSIFD.GPSLatitude]) == pytest.approx(lat, abs=1e-7)
