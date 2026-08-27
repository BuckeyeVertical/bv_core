import numpy as np
import pytest
from PIL import Image

from bv_core.detectors.ml_detector import MLDetector


class FakeModel:
    image_size = (640, 640)

    def __init__(self):
        self.predict_call = None
        self.sahi_call = None

    def predict(self, image, threshold):
        self.predict_call = (image.size, threshold)
        return {"bboxes": np.empty((0, 4), dtype=np.float32)}

    def predict_sahi(self, image, threshold, overlap):
        self.sahi_call = (image.size, threshold, overlap)
        return {
            "bboxes": np.array([[100.0, 200.0, 300.0, 400.0]], dtype=np.float32)
        }


def detector_with_fake_model(local_slices=4, progress_callback=None):
    detector = MLDetector(
        "unused.pt",
        local_slices,
        overlap=0.2,
        progress_callback=progress_callback,
    )
    detector.model = FakeModel()
    return detector


def test_bevy_and_real_frames_use_the_same_two_by_two_grid():
    bevy_detector = detector_with_fake_model()
    real_detector = detector_with_fake_model()

    bevy_results = bevy_detector._predict_sahi(
        Image.new("RGB", (1280, 960)), threshold=0.5
    )
    real_results = real_detector._predict_sahi(
        Image.new("RGB", (4640, 3480)), threshold=0.5
    )

    assert bevy_detector.model.predict_call is None
    assert real_detector.model.predict_call is None
    assert bevy_detector.model.sahi_call == ((1003, 752), 0.5, 0.2)
    assert real_detector.model.sahi_call == ((1003, 752), 0.5, 0.2)
    np.testing.assert_allclose(
        bevy_results["bboxes"],
        [[100.0 * 1280 / 1003, 200.0 * 960 / 752,
          300.0 * 1280 / 1003, 400.0 * 960 / 752]],
    )
    np.testing.assert_allclose(
        real_results["bboxes"],
        [[100.0 * 4640 / 1003, 200.0 * 3480 / 752,
          300.0 * 4640 / 1003, 400.0 * 3480 / 752]],
    )


def test_progress_reports_the_full_sahi_batch():
    progress = []
    detector = detector_with_fake_model(progress_callback=progress.append)

    detector._predict_sahi(Image.new("RGB", (1280, 960)), threshold=0.5)

    assert len(progress) == 2
    assert progress[0]["status"] == "running"
    assert progress[0]["local_slices"] == 4
    assert progress[0]["total"] == 5
    assert progress[1]["status"] == "complete"
    assert progress[1]["completed"] == 5


def test_local_slice_count_must_form_a_square_grid():
    with pytest.raises(ValueError, match="square grid"):
        detector_with_fake_model(local_slices=3)
