"""Class-specific confidence configuration and filtering."""

import math

import numpy as np


def load_thresholds(config, class_names):
    """Return per-class thresholds, falling back to the legacy global value."""
    default = float(config.get('detection_threshold', 0.5))
    overrides = config.get('detection_thresholds', {})
    unknown = set(overrides) - set(class_names)
    if unknown:
        raise ValueError(f'Unknown detection classes: {sorted(unknown)}')
    thresholds = {
        index: float(overrides.get(name, default))
        for index, name in enumerate(class_names)
    }
    if any(not math.isfinite(value) or not 0 <= value <= 1
           for value in [default, *thresholds.values()]):
        raise ValueError('Detection thresholds must be finite values from 0 to 1')
    return thresholds


def filter_detections(detections, thresholds):
    """Keep recognized classes whose confidence meets their own threshold."""
    if not thresholds or len(detections) == 0:
        return detections
    keep = np.array([
        int(class_id) in thresholds and score >= thresholds[int(class_id)]
        for score, class_id in zip(detections.confidence, detections.class_id)
    ], dtype=bool)
    return detections[keep]
