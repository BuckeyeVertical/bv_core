"""Tests for normalizing alternating snake-row image orientation."""

import cv2
import numpy as np

from bv_core.stitching import normalize_row_orientation


def test_even_rows_are_rotated_and_odd_rows_are_unchanged():
    image = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)

    odd = normalize_row_orientation([image], 1)[0]
    even = normalize_row_orientation([image], 2)[0]

    assert odd is image
    assert np.array_equal(even, cv2.rotate(image, cv2.ROTATE_180))
