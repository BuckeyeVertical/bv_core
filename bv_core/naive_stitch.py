"""Dead-reckoned fallback stitching.

Places captured frames by planned geometry alone: no features, no
homography, nothing that can fail to converge. The result is a coarse
mosaic that always exists, as a floor under the feature-based stitcher in
`stitching.py`.

Pure numpy on purpose. No cv2, no ROS, so it unit-tests in a bare venv.
"""

from dataclasses import dataclass

import numpy as np

from .stitch_geometry import cross_track_m, distance_m


@dataclass(frozen=True)
class NaiveLayout:
    """Integer pixel geometry for one dead-reckoned mosaic.

    along_step_px: pixels between consecutive frames within a row.
    row_length_px: pixel distance from a row's anchor to its endpoint. The
                   scheduler's final capture fires there rather than at an
                   index multiple, so placement needs it.
    row_x_px:      canvas column offset per row, index 0 == row 1,
                   normalized so the minimum is 0.
    """

    along_step_px: int
    row_length_px: int
    row_x_px: tuple[int, ...]
    frame_w: int
    frame_h: int


def layout_from_plan(
    scan_plan,
    frame_w: int,
    frame_h: int,
    camera_x_sign: int = -1,
) -> NaiveLayout:
    """Derive pixel offsets from planned scan geometry.

    Along-track spacing is exactly `(1 - overlap)` of the frame height,
    because `compute_step_m` applies that factor with no rounding.

    Cross-track spacing is NOT `(1 - overlap)`. `scan_plan._row_offsets`
    rounds the interval count up, so realized row overlap always exceeds
    the configured value (56.6% against a configured 35% in
    `test_scan_plan.py`), and the `scan_points` path skips that function
    entirely and reports `row_spacing_m = 0.0`. Row offsets are therefore
    always measured from the waypoints; see `_row_offsets_m`.

    Args:
        scan_plan: a `ScanPlan` from `bv_core.scan_plan`.
        frame_w: width of the frames as they will be pasted (post-resize).
        frame_h: height of the frames as they will be pasted.
        camera_x_sign: maps `cross_track_m`'s left-positive convention onto
            the image x axis. -1 for a nadir camera whose image +x points
            right of travel.
    """
    if frame_w <= 0 or frame_h <= 0:
        raise ValueError("frame dimensions must be positive")
    if camera_x_sign not in (1, -1):
        raise ValueError("camera_x_sign must be 1 or -1")

    along_step_px = max(1, round((1.0 - scan_plan.overlap) * frame_h))

    offsets_m = _row_offsets_m(scan_plan)
    if not offsets_m:
        raise ValueError("scan plan has no rows")

    cross_footprint_m = float(scan_plan.cross_footprint_m)
    if cross_footprint_m <= 0.0:
        raise ValueError("cross_footprint_m must be positive")

    raw_px = [
        camera_x_sign * offset_m / cross_footprint_m * frame_w
        for offset_m in offsets_m
    ]
    shift = min(raw_px)
    row_x_px = tuple(round(value - shift) for value in raw_px)

    return NaiveLayout(
        along_step_px=along_step_px,
        row_length_px=_row_length_px(scan_plan, along_step_px),
        row_x_px=row_x_px,
        frame_w=frame_w,
        frame_h=frame_h,
    )


def _row_length_px(scan_plan, along_step_px: int) -> int:
    """Anchor-to-endpoint distance of a row, in pixels.

    The scale is fixed by the pinhole model and needs no extra inputs:
    `capture_spacing_m` of ground is exactly `along_step_px` of image, so
    `px_per_m = along_step_px / capture_spacing_m`.
    """
    waypoints = scan_plan.waypoints
    if len(waypoints) < 2:
        return 0
    spacing_m = float(scan_plan.capture_spacing_m)
    if spacing_m <= 0.0:
        return 0
    length_m = distance_m(
        (waypoints[0][0], waypoints[0][1]),
        (waypoints[1][0], waypoints[1][1]),
    )
    return int(round(length_m * along_step_px / spacing_m))


def _row_offsets_m(scan_plan) -> list[float]:
    """Signed cross-track offset of each row, all measured in row 1's frame.

    Measuring row R+1 against row R's line would flip sign every row: each
    row's travel direction alternates in a snake, and `cross_track_m` is
    signed relative to direction of travel. Measuring every row against row
    1's line is monotonic and handles unequal row spacing for free.

    `ScanPlan.row_spacing_m` is deliberately not used, even when it is set.
    It is a magnitude with no direction, so mixing it in would mirror the
    row order relative to the measured path. It is also not independent
    truth: `_snake_waypoints` generates the waypoints from that spacing, so
    measuring them back reproduces it. And it is 0.0 on the `scan_points`
    path, which has to be measured regardless. One code path, self-consistent.
    """
    waypoints = scan_plan.waypoints
    row_count = len(waypoints) // 2
    if row_count == 0:
        return []
    if row_count == 1:
        return [0.0]

    anchor = (waypoints[0][0], waypoints[0][1])
    endpoint = (waypoints[1][0], waypoints[1][1])
    return [
        cross_track_m(
            (waypoints[2 * index][0], waypoints[2 * index][1]),
            anchor,
            endpoint,
        )
        for index in range(row_count)
    ]


def build_naive_mosaic(
    row_groups: dict[int, list[np.ndarray]],
    layout: NaiveLayout,
) -> np.ndarray:
    """Paste frames onto one canvas at positions derived from capture index.

    Frame N of a row sits `N * along_step_px` from that row's start. The
    canvas is oriented forward-is-up, so odd rows (which fly +D) stack in
    reverse: their last frame is the most forward and lands at the top.
    Even rows fly -D, so their anchor is at the far end and they stack
    forward. Even-row frames are rotated 180 degrees first, which flips
    both image axes at once and re-aligns the tile to ground on both.

    Overlap is a hard paste, later frame wins. Dead-reckoned placement does
    not register to the pixel, so averaging would ghost.
    """
    if not row_groups:
        raise ValueError("no rows to stitch")

    populated = {row: frames for row, frames in row_groups.items() if frames}
    if not populated:
        raise ValueError("no frames to stitch")

    ground = {row: _ground_positions_px(len(frames), row, layout)
              for row, frames in populated.items()}
    top = max(max(positions) for positions in ground.values())
    tops = {row: [int(round(top - value)) for value in positions]
            for row, positions in ground.items()}
    lift = min(min(values) for values in tops.values())

    canvas_h = max(max(values) for values in tops.values()) - lift + layout.frame_h
    canvas_w = max(layout.row_x_px) + layout.frame_w
    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

    for row in sorted(populated):
        frames = populated[row]
        x = layout.row_x_px[min(row - 1, len(layout.row_x_px) - 1)]

        for index, frame in enumerate(frames):
            if frame.shape[:2] != (layout.frame_h, layout.frame_w):
                raise ValueError(
                    f"row{row} frame {index + 1} is "
                    f"{frame.shape[1]}x{frame.shape[0]}, "
                    f"expected {layout.frame_w}x{layout.frame_h}"
                )
            if row % 2 == 0:
                frame = frame[::-1, ::-1]

            y = tops[row][index] - lift
            canvas[y:y + layout.frame_h, x:x + layout.frame_w] = frame

    return canvas


def _ground_positions_px(count: int, row: int, layout: NaiveLayout) -> list[float]:
    """Along-track position of each capture, in row 1's frame, in pixels.

    `StitchCaptureScheduler` fires its first capture at half a spacing and
    then every spacing after, so capture i sits at `(i + 0.5) * step`. Its
    LAST capture is different: `_capture(..., "endpoint")` fires at the row
    end regardless of spacing. On the 2026-09-14 sim run that put the final
    frame 10.5 m past its predecessor against a 38.49 m spacing, so treating
    it as another index multiple over-advances it by ~450 px (28 m). Clamp
    it to the row end instead.

    Even rows fly the reverse direction, so their anchor is at the far end
    and their positions mirror about the row length.
    """
    step = layout.along_step_px
    positions = [(index + 0.5) * step for index in range(count)]

    length = float(layout.row_length_px)
    if length > 0.0 and count >= 2 and positions[-1] > length:
        positions[-1] = length

    if row % 2 == 0:
        positions = [length - value for value in positions]
    return positions
