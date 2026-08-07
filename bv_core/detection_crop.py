#!/usr/bin/env python3
"""Build an annotated, native-resolution crop around a single detection.

The operator judges a detection from this image, so it has to stay legible. A
whole-frame downscale will not do: at 15 m AGL the real camera (fx=3582.9,
4640 px wide) puts a 1.8 m person at roughly 430 px tall, while the Gazebo sim
(fx=410.9, 1280 px) puts the same person at roughly 49 px. One fixed crop size
cannot serve both, so the window adapts to the bounding box.

Pure numpy/cv2 — no ROS — so the geometry is testable without a camera.
"""

# MUST STAY FIRST, above cv2: pins libgcc's unwinder before OpenCV can map
# libunwind and break Fast-DDS exception handling. See bv_core/_unwinder.py.
import bv_core._unwinder  # noqa: F401  # isort: skip  (side-effect; keep first)

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class CropConfig:
    """Tunables for the crop, mirrored in config/vision_params.yaml."""

    margin_factor: float = 2.5   # how much context around the box
    min_px: int = 320            # floor, so a tiny detection still gets context
    max_px: int = 1024           # ceiling on the longest output side
    jpeg_quality: int = 85


def crop_window(bbox, frame_w, frame_h, cfg):
    """Compute the crop window for one bounding box.

    Args:
        bbox: (x1, y1, x2, y2) in frame pixel coordinates.
        frame_w, frame_h: frame dimensions.
        cfg: CropConfig.

    Returns:
        (x1, y1, x2, y2) integer window, guaranteed inside the frame.

    Near a frame edge the window is *shifted* rather than shrunk, so an edge
    detection still gets its full share of context instead of a sliver.
    """
    x1, y1, x2, y2 = (float(v) for v in bbox)
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0

    # max(..., 1.0) keeps a degenerate zero-area box from collapsing the window.
    width = max(x2 - x1, 1.0) * cfg.margin_factor
    height = max(y2 - y1, 1.0) * cfg.margin_factor

    width = max(width, float(cfg.min_px))
    height = max(height, float(cfg.min_px))

    # Can never be bigger than the frame it came from.
    width = min(width, float(frame_w))
    height = min(height, float(frame_h))

    left = min(max(cx - width / 2.0, 0.0), frame_w - width)
    top = min(max(cy - height / 2.0, 0.0), frame_h - height)

    return (
        int(round(left)),
        int(round(top)),
        int(round(left + width)),
        int(round(top + height)),
    )


_BOX_BGR = (84, 220, 64)      # same green supervision draws in vision_node
_TEXT_BGR = (12, 24, 12)


def build_annotated_crop(frame, bbox, label, cfg):
    """Crop around `bbox`, draw the box and `label`, and JPEG-encode.

    Args:
        frame: HxWx3 BGR or HxW grayscale numpy array.
        bbox: (x1, y1, x2, y2) detection box in frame coordinates.
        label: text drawn above the box, e.g. "person 0.94".
        cfg: CropConfig.

    Returns:
        JPEG bytes.

    Raises:
        RuntimeError: if cv2 fails to encode.

    The box is drawn *after* the downscale on purpose. Annotating at native
    resolution and then resizing 4640 -> 1024 thins the lines to near
    invisibility; this way line weight is specified in the pixels the operator
    actually sees.
    """
    if frame.ndim == 2:
        frame = np.repeat(frame[:, :, None], 3, axis=2)

    frame_h, frame_w = frame.shape[:2]
    wx1, wy1, wx2, wy2 = crop_window(bbox, frame_w, frame_h, cfg)
    crop = frame[wy1:wy2, wx1:wx2].copy()

    # Detection box relative to the crop origin.
    bx1, by1, bx2, by2 = (float(v) for v in bbox)
    box = [bx1 - wx1, by1 - wy1, bx2 - wx1, by2 - wy1]

    crop_h, crop_w = crop.shape[:2]
    scale = min(1.0, float(cfg.max_px) / float(max(crop_h, crop_w)))
    if scale < 1.0:
        crop = cv2.resize(
            crop,
            (max(1, int(round(crop_w * scale))), max(1, int(round(crop_h * scale)))),
            interpolation=cv2.INTER_AREA,
        )
        box = [v * scale for v in box]

    _annotate(crop, box, label)

    ok, buf = cv2.imencode(
        '.jpg', crop, [int(cv2.IMWRITE_JPEG_QUALITY), int(cfg.jpeg_quality)])
    if not ok:
        raise RuntimeError('cv2.imencode failed for annotated crop')
    return buf.tobytes()


def _annotate(image, box, label):
    """Draw the detection box and its label, in `image` pixel coordinates."""
    h, w = image.shape[:2]
    x1 = int(round(max(0, min(box[0], w - 1))))
    y1 = int(round(max(0, min(box[1], h - 1))))
    x2 = int(round(max(0, min(box[2], w - 1))))
    y2 = int(round(max(0, min(box[3], h - 1))))

    cv2.rectangle(image, (x1, y1), (x2, y2), _BOX_BGR, 2)

    (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    # Put the label inside the image when the box is flush against the top.
    label_bottom = y1 if y1 - text_h - 8 >= 0 else min(h, y2 + text_h + 8)
    label_top = max(0, label_bottom - text_h - 8)

    # Slide the chip left rather than let OpenCV clip it: a chip running past the
    # right edge silently truncates the text, so the operator reads "pers".
    chip_w = text_w + 8
    chip_left = int(max(0, min(x1, w - chip_w)))

    cv2.rectangle(image, (chip_left, label_top), (chip_left + chip_w, label_bottom),
                  _BOX_BGR, -1)
    cv2.putText(image, label, (chip_left + 4, label_bottom - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, _TEXT_BGR, 2, cv2.LINE_AA)
