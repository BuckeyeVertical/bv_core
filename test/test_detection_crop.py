#!/usr/bin/env python3
"""Tests for detection_crop — pure numpy/cv2, no ROS required."""

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.detection_crop import CropConfig, build_annotated_crop, crop_window  # noqa: E402


CFG = CropConfig(margin_factor=2.5, min_px=320, max_px=1024, jpeg_quality=85)


class TestCropWindow:
    """Geometry of the crop window before any pixels are touched."""

    def test_centred_box_is_expanded_by_margin(self):
        # 200x200 box in the middle of a 4640x3480 frame.
        x1, y1, x2, y2 = crop_window((2220, 1640, 2420, 1840), 4640, 3480, CFG)
        assert (x2 - x1) == 500      # 200 * 2.5
        assert (y2 - y1) == 500
        # Still centred on the box centre (2320, 1740).
        assert (x1 + x2) // 2 == 2320
        assert (y1 + y2) // 2 == 1740

    def test_small_box_widens_to_min_px(self):
        # 20x49 box — a person in sim. 2.5x is only 50x122, below the 320 floor.
        x1, y1, x2, y2 = crop_window((630, 330, 650, 379), 1280, 720, CFG)
        assert (x2 - x1) == 320
        assert (y2 - y1) == 320

    def test_box_at_left_edge_shifts_rather_than_shrinks(self):
        # Box hard against x=0. Window must stay full size and stay in bounds.
        x1, y1, x2, y2 = crop_window((0, 1640, 200, 1840), 4640, 3480, CFG)
        assert x1 == 0
        assert (x2 - x1) == 500
        assert x2 <= 4640

    def test_box_at_bottom_right_corner_shifts_rather_than_shrinks(self):
        x1, y1, x2, y2 = crop_window((4440, 3280, 4640, 3480), 4640, 3480, CFG)
        assert (x2 - x1) == 500
        assert (y2 - y1) == 500
        assert x2 == 4640
        assert y2 == 3480
        assert x1 >= 0 and y1 >= 0

    def test_window_never_exceeds_frame(self):
        # Requested window (min_px=320) is larger than this tiny frame.
        x1, y1, x2, y2 = crop_window((10, 10, 20, 20), 100, 80, CFG)
        assert x1 == 0 and y1 == 0
        assert x2 == 100 and y2 == 80

    def test_degenerate_zero_area_box_does_not_crash(self):
        x1, y1, x2, y2 = crop_window((500, 500, 500, 500), 4640, 3480, CFG)
        assert (x2 - x1) == 320      # falls back to the min window
        assert (y2 - y1) == 320


def _frame(w=4640, h=3480):
    """A frame with non-uniform content so JPEG size is realistic."""
    rng = np.random.default_rng(seed=1)
    return rng.integers(0, 255, (h, w, 3), dtype=np.uint8)


class TestBuildAnnotatedCrop:
    """The encoded JPEG handed to the operator."""

    def test_returns_decodable_jpeg(self):
        data = build_annotated_crop(_frame(), (2220, 1640, 2420, 1840),
                                    'person 0.94', CFG)
        assert data[:2] == b'\xff\xd8'          # JPEG SOI marker
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert decoded is not None

    def test_output_respects_max_px(self):
        # 2000x2000 box -> 5000x5000 window, must be capped at 1024.
        data = build_annotated_crop(_frame(), (1320, 740, 3320, 2740),
                                    'tent 0.80', CFG)
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert max(decoded.shape[:2]) == 1024

    def test_small_detection_keeps_native_resolution(self):
        # 20x49 box widens to the 320 floor, which is under max_px -> no rescale.
        data = build_annotated_crop(_frame(1280, 720), (630, 330, 650, 379),
                                    'person 0.91', CFG)
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape[0] == 320
        assert decoded.shape[1] == 320

    def test_edge_detection_does_not_crash(self):
        data = build_annotated_crop(_frame(), (0, 0, 120, 120), 'person 0.70', CFG)
        assert len(data) > 0

    def test_grayscale_frame_is_accepted(self):
        rng = np.random.default_rng(seed=2)
        gray = rng.integers(0, 255, (720, 1280), dtype=np.uint8)
        data = build_annotated_crop(gray, (630, 330, 650, 379), 'person 0.60', CFG)
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert decoded is not None

    def test_payload_stays_link_friendly(self):
        # Random noise is close to the worst case for JPEG; real imagery is smaller.
        data = build_annotated_crop(_frame(), (2220, 1640, 2420, 1840),
                                    'person 0.94', CFG)
        assert len(data) < 400_000
