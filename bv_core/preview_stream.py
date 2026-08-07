#!/usr/bin/env python3
"""Low-bitrate camera preview for the ground station.

This module is deliberately split so the part that runs on the capture thread —
frame gating — has no GStreamer dependency and can be tested exhaustively. The
encoder lives alongside it but is only exercised in integration.

The governing rule is that the capture thread is never delayed. `FrameGate.offer`
rate-limits, keeps at most one frame, and returns immediately. When the encoder is
slow the preview loses framerate; capture and detection are untouched.
"""

import threading
import time

import cv2


def decimation_factor(src_width, target_width):
    """Integer stride that gets `src_width` close to `target_width`.

    Strided slicing is far cheaper than resizing a full frame: it copies every
    nth pixel instead of reading and interpolating all of them. Returns 1 when
    the source is already at or below the target.
    """
    if target_width <= 0 or src_width <= target_width:
        return 1
    return max(1, int(src_width) // int(target_width))


def downscale(frame, target_width):
    """Reduce `frame` to `target_width`, preserving aspect ratio.

    Decimates first, then resizes the much smaller result. Output dimensions are
    forced even because H.264 encoders reject odd ones. Never mutates the input.
    """
    height, width = frame.shape[:2]
    if width <= target_width:
        return frame

    stride = decimation_factor(width, target_width)
    if stride > 1:
        frame = frame[::stride, ::stride]

    dec_h, dec_w = frame.shape[:2]
    out_w = int(target_width) - (int(target_width) % 2)
    out_h = int(round(dec_h * out_w / dec_w))
    out_h -= out_h % 2

    return cv2.resize(frame, (max(2, out_w), max(2, out_h)),
                      interpolation=cv2.INTER_AREA)


class FrameGate:
    """Rate-limited, latest-wins, single-slot handoff to the encoder thread.

    `offer` is called from the capture thread and must never block or raise.
    `take` is called from the encoder thread.
    """

    def __init__(self, fps):
        """Build a gate that accepts at most `fps` frames per second.

        Args:
            fps: accept at most this many frames per second. 0 disables limiting.
        """
        self._min_interval = 1.0 / fps if fps and fps > 0 else 0.0
        self._lock = threading.Lock()
        self._slot = None
        self._last_accept = 0.0
        self._counts = {'offered': 0, 'accepted': 0, 'dropped': 0}

    @property
    def stats(self):
        """Return a snapshot of the offered/accepted/dropped counters."""
        with self._lock:
            return dict(self._counts)

    def offer(self, frame):
        """Present a frame. Returns True if it was accepted.

        Rejects when it arrives sooner than the rate limit allows. If a previously
        accepted frame has not been taken yet, it is overwritten — the newest frame
        is always the useful one for a live view.
        """
        try:
            with self._lock:
                self._counts['offered'] += 1

                if frame is None:
                    self._counts['dropped'] += 1
                    return False

                now = time.monotonic()
                if self._min_interval and (now - self._last_accept) < self._min_interval:
                    self._counts['dropped'] += 1
                    return False

                if self._slot is not None:
                    # Never encoded; the encoder was still busy.
                    self._counts['dropped'] += 1

                self._last_accept = now
                self._slot = frame
                self._counts['accepted'] += 1
                return True
        except Exception:       # noqa: BLE001 - the capture thread must survive
            return False

    def take(self):
        """Remove and return the pending frame, or None."""
        with self._lock:
            frame, self._slot = self._slot, None
            return frame
