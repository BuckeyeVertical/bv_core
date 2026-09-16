"""Framework-neutral metadata carried with a captured vision frame."""

import math
from dataclasses import dataclass
from typing import Any, Mapping


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


# EXIF/XMP geotagging for stitch frames
#
# The frames written to raw_frames/ are fed to OpenDroneMap off the aircraft,
# and ODM reconstructs far better from frames that carry their own position,
# time and camera model than from bare pixels. Everything needed exists only
# at the moment of capture, so it is gathered there and written here.
#
# These build exiftool argument lists rather than touching files, so the whole
# derivation is testable without exiftool, OpenCV or ROS on the path.

MM_PER_INCH = 25.4


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

    def tags(self, image_width_px: int, image_height_px: int) -> list:
        """exiftool arguments for an image of this pixel size.

        ``FocalLength`` is a physical length and does not move with the output
        resolution, but ``FocalPlane*Resolution`` counts pixels per inch of
        sensor and does — so a downscaled stream must be tagged against the
        size actually written, not the size the camera was calibrated at.
        """
        if min(image_width_px, image_height_px) <= 0:
            raise ValueError("image dimensions must be positive")

        x_res = image_width_px * MM_PER_INCH / self.sensor_width_mm
        y_res = image_height_px * MM_PER_INCH / self.sensor_height_mm
        return [
            f"-Make={self.make}",
            f"-Model={self.model}",
            f"-FocalLength={self.focal_length_mm:.4f}",
            f"-FocalPlaneXResolution={x_res:.4f}",
            f"-FocalPlaneYResolution={y_res:.4f}",
            # The '#' forces the numeric value. Without it exiftool expects the
            # word 'inches' and rejects the 2.
            "-FocalPlaneResolutionUnit#=2",
        ]


@dataclass(frozen=True)
class FrameGeotag:
    """Where, when and which way the aircraft was when one frame was taken.

    ``pitch_deg``/``roll_deg`` default to nadir because the camera rides a
    gimbal that holds it level - the same fact ``localizer.py`` relies on when
    it discards drone roll and pitch and keeps only yaw.
    """

    latitude: float
    longitude: float
    altitude_m: float
    altitude_is_msl: bool
    captured_at: Any
    horizontal_error_m: Any = None
    heading_deg: Any = None
    pitch_deg: float = -90.0
    roll_deg: float = 0.0

    def tags(self) -> list:
        """exiftool arguments for this frame.

        Signed coordinates are split into an unsigned magnitude and a
        reference: handing exiftool a negative value AND a reference lets the
        reference win silently, so a southern latitude written as -33.87 with
        an 'S' ref would land in the northern hemisphere.
        """
        tags = [
            f"-GPSLatitude={abs(self.latitude):.8f}",
            f"-GPSLatitudeRef={'N' if self.latitude >= 0.0 else 'S'}",
            f"-GPSLongitude={abs(self.longitude):.8f}",
            f"-GPSLongitudeRef={'E' if self.longitude >= 0.0 else 'W'}",
            f"-GPSAltitude={abs(self.altitude_m):.3f}",
            f"-GPSAltitudeRef={0 if self.altitude_m >= 0.0 else 1}",
            f"-DateTimeOriginal={self.captured_at.strftime('%Y:%m:%d %H:%M:%S')}",
            f"-SubSecTimeOriginal={self.captured_at.microsecond // 1000:03d}",
        ]

        # ODM weights a position by its reported accuracy, so an unknown
        # accuracy must be absent rather than guessed at.
        if self.horizontal_error_m is not None:
            tags.append(
                f"-GPSHPositioningError={self.horizontal_error_m:.3f}")

        # A wrong orientation prior is worse than none: it steers ODM's
        # matcher instead of merely failing to help it.
        if self.heading_deg is not None:
            tags.extend([
                f"-XMP-Camera:Yaw={self.heading_deg:.2f}",
                f"-XMP-Camera:Pitch={self.pitch_deg:.2f}",
                f"-XMP-Camera:Roll={self.roll_deg:.2f}",
            ])

        return tags


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
    # then returns exactly 360.0, which is outside the range promised above
    # and would be written to the tag as Yaw=360.00. Snapping first folds it.
    return round(heading, 6) % 360.0
