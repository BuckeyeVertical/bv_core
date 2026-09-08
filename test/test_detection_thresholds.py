"""Regression checks for independent class confidence cutoffs."""

import numpy as np
import pytest

from bv_core.detection_thresholds import filter_detections, load_thresholds


class Detections:
    def __init__(self, classes, scores):
        self.class_id = np.array(classes)
        self.confidence = np.array(scores)

    def __len__(self):
        return len(self.class_id)

    def __getitem__(self, mask):
        return Detections(self.class_id[mask], self.confidence[mask])


def test_lower_threshold_class_survives_and_higher_threshold_class_is_filtered():
    thresholds = load_thresholds(
        {'detection_thresholds': {'person': 0.3, 'tent': 0.8}},
        ('person', 'tent'))
    assert min(thresholds.values()) == 0.3
    result = filter_detections(
        Detections([0, 1, 1, 0, 9], [0.4, 0.7, 0.8, 0.2, 0.99]), thresholds)
    assert result.class_id.tolist() == [0, 1]
    assert result.confidence.tolist() == [0.4, 0.8]


def test_legacy_and_partial_configuration():
    assert load_thresholds({'detection_threshold': 0.6}, ('person', 'tent')) == {
        0: 0.6, 1: 0.6}
    assert load_thresholds(
        {'detection_thresholds': {'tent': 0.7}}, ('person', 'tent')) == {
            0: 0.5, 1: 0.7}


@pytest.mark.parametrize('value', [-0.1, 1.1, float('nan'), float('inf')])
def test_invalid_threshold_is_rejected(value):
    with pytest.raises(ValueError):
        load_thresholds({'detection_thresholds': {'person': value}}, ('person', 'tent'))


def test_misspelled_class_is_rejected():
    with pytest.raises(ValueError, match='Unknown detection classes'):
        load_thresholds({'detection_thresholds': {'tents': 0.7}}, ('person', 'tent'))
