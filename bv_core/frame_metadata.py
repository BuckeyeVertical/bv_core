"""Framework-neutral metadata carried with a captured vision frame."""

import math
from dataclasses import dataclass
from typing import Any, Mapping

import piexif


@dataclass(frozen=True)
class SimulationFrameMetadata:
    stream_id: str
    sequence: int
    time_ns: int

    @classmethod
    def from_mapping(cls, metadata: Mapping[str, Any]):
        fields = ("sim_stream_id", "sim_sequence", "sim_time_ns")
        values = tuple(metadata.get(field) for field in fields)
        if all(value is None for value in values):
            return None
        stream_id, sequence, time_ns = values
        if sequence is None or time_ns is None:
            raise ValueError("simulation frame metadata is incomplete")

        if stream_id is None:
            stream_id = ""
        elif not isinstance(stream_id, str) or not stream_id.strip():
            raise ValueError(
                "simulation frame metadata has an invalid stream ID"
            )
        for name, value in (("sequence", sequence), ("time", time_ns)):
            if (
                not isinstance(value, int)
                or isinstance(value, bool)
                or value < 0
            ):
                raise ValueError(
                    f"simulation frame metadata has an invalid {name}"
                )
        return cls(stream_id, sequence, time_ns)


def apply_simulation_metadata(
    message,
    metadata: SimulationFrameMetadata | None,
) -> None:
    if metadata is None:
        return
    message.sim_stream_id = metadata.stream_id
    message.sim_sequence = metadata.sequence
    message.sim_time_ns = metadata.time_ns


# EXIF geotagging for stitch frames
#
# The frames written to raw_frames/ are reconstructed by OpenDroneMap off the
# aircraft, and ODM does far better with frames that carry their own position,
# time and camera model than with bare pixels. All of that exists only at the
# moment of capture, so it is gathered there and encoded here.
#
# These build piexif dictionaries rather than touching files, so the whole
# derivation is testable without ROS or OpenCV on the path.

MM_PER_INCH = 25.4

# EXIF stores every real number as a RATIONAL: a pair of UNSIGNED 32-BIT
# integers. That ceiling of 4294967295 is what picks the scales below - a
# focal-plane resolution of 7389.09 at a millionth would need 7389090909 and
# silently overflow - so each quantity gets the finest scale its own range
# can afford.
_FOCAL_SCALE = 1000000          # mm, to a nanometre
_RESOLUTION_SCALE = 10000       # pixels per inch, to a ten-thousandth
_METRE_SCALE = 1000             # metres, to a millimetre
_DEGREE_SCALE = 1000            # degrees, to a thousandth
_ARCSECOND_SCALE = 10000        # arcseconds, to ~3 mm of ground


def _rational(value, scale):
    """Encode a real number as an EXIF RATIONAL at a fixed denominator."""
    return (int(round(value * scale)), scale)


def _dms_rationals(decimal_degrees):
    """Split a decimal degree into EXIF's degree/minute/second triple.

    EXIF predates decimal coordinates, so a position is three rationals and
    the sign lives in a separate reference tag. The magnitude is taken here;
    the caller supplies the reference.
    """
    remaining = abs(decimal_degrees)
    degrees = int(remaining)
    minutes_total = (remaining - degrees) * 60.0
    minutes = int(minutes_total)
    seconds = (minutes_total - minutes) * 60.0

    # Round the seconds FIRST, then carry. Rounding in place would let a value
    # a hair under a full minute - 59.99996 arcsec at this scale - encode as
    # exactly 60 arcseconds, which is out of range for the field and which no
    # reader is obliged to interpret sensibly.
    ticks = int(round(seconds * _ARCSECOND_SCALE))
    if ticks >= 60 * _ARCSECOND_SCALE:
        ticks -= 60 * _ARCSECOND_SCALE
        minutes += 1
    if minutes >= 60:
        minutes -= 60
        degrees += 1
    return [(degrees, 1), (minutes, 1), (ticks, _ARCSECOND_SCALE)]


@dataclass(frozen=True)
class CameraExifProfile:
    """Static EXIF describing one calibrated camera.

    Derived from the same intrinsics ``localizer.py`` projects with, so a
    recalibration cannot leave the tags describing a camera that never flew.
    """

    make: str
    model: str
    focal_length_mm: float
    sensor_width_mm: float
    sensor_height_mm: float

    @classmethod
    def from_calibration(
        cls,
        make: str,
        model: str,
        fx: float,
        fy: float,
        calib_width_px: int,
        calib_height_px: int,
        sensor_width_mm: float,
    ):
        """Solve the physical camera that produces this intrinsic matrix.

        ``fx`` in pixels and the sensor's physical width give the focal length
        directly.  The sensor HEIGHT is then solved rather than assumed square,
        so the calibrated ``fy`` survives into the tags: a 4K frame off an RX0
        II is a 16:9 crop of a 3:2 sensor, and pretending the pixels are square
        would quietly discard the 0.7% the calibration actually measured.
        """
        if min(fx, fy, sensor_width_mm) <= 0.0:
            raise ValueError("focal lengths and sensor width must be positive")
        if min(calib_width_px, calib_height_px) <= 0:
            raise ValueError("calibration dimensions must be positive")

        focal_length_mm = fx * sensor_width_mm / float(calib_width_px)
        sensor_height_mm = focal_length_mm * float(calib_height_px) / fy
        return cls(
            make=make,
            model=model,
            focal_length_mm=focal_length_mm,
            sensor_width_mm=float(sensor_width_mm),
            sensor_height_mm=sensor_height_mm,
        )

    def exif_dict(self, image_width_px: int, image_height_px: int) -> dict:
        """piexif IFDs for an image of this pixel size.

        ``FocalLength`` is a physical length and does not move with the output
        resolution, but ``FocalPlane*Resolution`` counts pixels per inch of
        sensor and does - so a downscaled stream must be tagged against the
        size actually written, not the size the camera was calibrated at.
        """
        if min(image_width_px, image_height_px) <= 0:
            raise ValueError("image dimensions must be positive")

        x_res = image_width_px * MM_PER_INCH / self.sensor_width_mm
        y_res = image_height_px * MM_PER_INCH / self.sensor_height_mm
        return {
            "0th": {
                piexif.ImageIFD.Make: self.make.encode('ascii', 'replace'),
                piexif.ImageIFD.Model: self.model.encode('ascii', 'replace'),
            },
            "Exif": {
                piexif.ExifIFD.FocalLength:
                    _rational(self.focal_length_mm, _FOCAL_SCALE),
                piexif.ExifIFD.FocalPlaneXResolution:
                    _rational(x_res, _RESOLUTION_SCALE),
                piexif.ExifIFD.FocalPlaneYResolution:
                    _rational(y_res, _RESOLUTION_SCALE),
                # 2 == inches, which is what the resolutions above are in.
                piexif.ExifIFD.FocalPlaneResolutionUnit: 2,
            },
        }


@dataclass(frozen=True)
class FrameGeotag:
    """Where, when and which way the aircraft was when one frame was taken."""

    latitude: float
    longitude: float
    altitude_m: float
    # False means altitude_m is a height above the WGS84 ellipsoid, which has
    # no EXIF representation - see exif_dict.
    altitude_is_msl: bool
    captured_at: Any
    horizontal_error_m: Any = None
    heading_deg: Any = None

    def exif_dict(self) -> dict:
        """piexif IFDs for this frame.

        Camera pitch and roll are deliberately absent. They have no standard
        EXIF tag - only vendor XMP - and for a gimbal that holds the camera
        level they are the constants -90 and 0, so they carry no information
        that config does not already state.
        """
        gps = {
            piexif.GPSIFD.GPSLatitude: _dms_rationals(self.latitude),
            piexif.GPSIFD.GPSLatitudeRef:
                b'N' if self.latitude >= 0.0 else b'S',
            piexif.GPSIFD.GPSLongitude: _dms_rationals(self.longitude),
            piexif.GPSIFD.GPSLongitudeRef:
                b'E' if self.longitude >= 0.0 else b'W',
        }

        # EXIF can only say "above" or "below sea level" - it has no way to
        # express a height above the WGS84 ellipsoid, which is what the GPS
        # reports when no DEM is available. Over the continental US the two
        # differ by around 30 m, so writing an ellipsoidal height under
        # GPSAltitudeRef=0 would be a silent 30 m lie in every frame.
        # Omitting it instead makes the gap visible to the consumer.
        if self.altitude_is_msl:
            gps[piexif.GPSIFD.GPSAltitude] = _rational(
                abs(self.altitude_m), _METRE_SCALE)
            gps[piexif.GPSIFD.GPSAltitudeRef] = (
                0 if self.altitude_m >= 0.0 else 1)

        # ODM weights a position by its reported accuracy, so an unknown
        # accuracy must be absent rather than guessed at.
        if self.horizontal_error_m is not None:
            gps[piexif.GPSIFD.GPSHPositioningError] = _rational(
                self.horizontal_error_m, _METRE_SCALE)

        # GPSImgDirection is the standard home for a camera heading, which is
        # why this needs no vendor XMP. 'T' declares it against true north
        # rather than magnetic - the pose is already in ENU, so there is no
        # declination in it.
        if self.heading_deg is not None:
            gps[piexif.GPSIFD.GPSImgDirection] = _rational(
                self.heading_deg, _DEGREE_SCALE)
            gps[piexif.GPSIFD.GPSImgDirectionRef] = b'T'

        return {
            "Exif": {
                piexif.ExifIFD.DateTimeOriginal:
                    self.captured_at.strftime('%Y:%m:%d %H:%M:%S').encode(
                        'ascii'),
                piexif.ExifIFD.SubSecTimeOriginal:
                    f"{self.captured_at.microsecond // 1000:03d}".encode(
                        'ascii'),
            },
            "GPS": gps,
        }


def build_exif_bytes(
    profile: CameraExifProfile,
    geotag: FrameGeotag,
    image_width_px: int,
    image_height_px: int,
) -> bytes:
    """Merge the camera and frame metadata into one EXIF block."""
    camera = profile.exif_dict(image_width_px, image_height_px)
    frame = geotag.exif_dict()
    return piexif.dump({
        "0th": dict(camera["0th"]),
        "Exif": {**camera["Exif"], **frame["Exif"]},
        "GPS": dict(frame["GPS"]),
        "1st": {},
        "thumbnail": None,
    })


def heading_deg_from_quaternion(qx, qy, qz, qw) -> float:
    """Compass heading of the body x-axis, degrees clockwise from true north.

    MAVROS publishes attitude as a body-to-ENU quaternion, so rotating the
    body x-axis gives a forward vector measured counter-clockwise from EAST,
    while every consumer of a heading wants clockwise from NORTH.

    Only the first column of the rotation matrix is needed, so it is written
    out directly rather than pulling in scipy - this module stays importable
    wherever the tests run.
    """
    east = 1.0 - 2.0 * (qy * qy + qz * qz)
    north = 2.0 * (qx * qy + qz * qw)
    heading = (90.0 - math.degrees(math.atan2(north, east))) % 360.0
    # Due north rounds to a hair BELOW zero before the modulo - the quaternion
    # for a quarter turn puts `east` at -2e-16 rather than 0 - and the modulo
    # then returns exactly 360.0, which is outside the range promised above.
    # Snapping first folds it.
    return round(heading, 6) % 360.0
