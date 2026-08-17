"""Frame selection for distance-based stitch capture."""

from dataclasses import dataclass

from .stitch_geometry import along_track_m, cross_track_m, distance_m


@dataclass(frozen=True)
class StitchCapture:
    row: int
    column: int
    target_m: float
    distance_m: float
    kind: str
    skipped_targets: int = 0


class StitchCaptureScheduler:
    def __init__(self, spacing_m: float, max_cross_track_m: float | None = None):
        """
        Configure frame selection for one scan row.

        Args:
            spacing_m: along-track meters between captures.
            max_cross_track_m: reject frames further than this from the row line.
                None disables the check, which is the pre-existing behavior.
        """
        if spacing_m <= 0.0:
            raise ValueError("capture spacing must be positive")
        if max_cross_track_m is not None and max_cross_track_m <= 0.0:
            raise ValueError("max cross-track distance must be positive")

        self.spacing_m = spacing_m
        self.max_cross_track_m = max_cross_track_m
        self.active = False
        self._anchor = None
        self._endpoint = None
        self._row = 0
        self._column = 1
        self._next_target_m = 0.0
        self._row_length_m = 0.0
        self._last_frame_id = None

    def start_row(self, row: int, anchor, endpoint) -> None:
        self.active = True
        self._anchor = anchor
        self._endpoint = endpoint
        self._row = row
        self._column = 1
        self._next_target_m = 0.0
        self._row_length_m = distance_m(anchor, endpoint)
        self._last_frame_id = None

    def consider(self, position, frame_id: int):
        if not self.active:
            return None

        progress_m = along_track_m(position, self._anchor, self._endpoint)
        if progress_m < self._next_target_m:
            return None

        # Drop frames taken off the row. A delivery deviation flies the aircraft
        # far to the side while the row is still active, and along_track_m
        # projects onto the row axis only — so a frame from 100 m off-path reads
        # as perfectly good progress and gets written as row content.
        #
        # Returning None leaves _next_target_m untouched, so the real frame at
        # this distance is still captured once the aircraft is back on the row.
        # Capturing instead would be worse than skipping: the target is consumed
        # either way, so the off-path frame REPLACES the correct one.
        #
        # This must stay above the endpoint branch below. A deviation whose
        # projection runs past the row end would otherwise fire an endpoint
        # capture and deactivate the row, silently losing the rest of it.
        if self.max_cross_track_m is not None:
            offset_m = abs(cross_track_m(position, self._anchor, self._endpoint))
            if offset_m > self.max_cross_track_m:
                return None

        if progress_m >= self._row_length_m:
            return self._capture(
                frame_id,
                self._row_length_m,
                progress_m,
                "endpoint",
            )

        target_m = self._next_target_m
        skipped_targets = 0
        self._next_target_m += self.spacing_m
        while self._next_target_m <= progress_m:
            self._next_target_m += self.spacing_m
            skipped_targets += 1

        return self._capture(
            frame_id,
            target_m,
            progress_m,
            "start" if target_m == 0.0 else "spacing",
            skipped_targets,
        )

    def finish_row(self, position, frame_id: int):
        if not self.active:
            return None

        progress_m = along_track_m(position, self._anchor, self._endpoint)
        if frame_id == self._last_frame_id:
            self.active = False
            return None

        return self._capture(
            frame_id,
            self._row_length_m,
            progress_m,
            "endpoint",
        )

    def cancel_row(self) -> None:
        self.active = False

    def _capture(
        self,
        frame_id: int,
        target_m: float,
        progress_m: float,
        kind: str,
        skipped_targets: int = 0,
    ) -> StitchCapture:
        capture = StitchCapture(
            row=self._row,
            column=self._column,
            target_m=target_m,
            distance_m=progress_m,
            kind=kind,
            skipped_targets=skipped_targets,
        )
        self._column += 1
        self._last_frame_id = frame_id
        if kind == "endpoint":
            self.active = False
        return capture
