import numpy as np
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


def detector_with_fake_model(source_tile_size, progress_callback=None):
    detector = MLDetector(
        "unused.pt",
        source_tile_size,
        overlap=0.2,
        progress_callback=progress_callback,
    )
    detector.model = FakeModel()
    return detector


def test_bevy_frame_uses_same_three_by_three_grid_as_real_camera():
    detector = detector_with_fake_model((530, 530))

    results = detector._predict_sahi(Image.new("RGB", (1280, 960)), threshold=0.5)

    assert detector.model.predict_call is None
    assert detector.model.sahi_call == ((1546, 1159), 0.5, 0.2)
    np.testing.assert_allclose(
        results["bboxes"],
        [[100.0 * 1280 / 1546, 200.0 * 960 / 1159,
          300.0 * 1280 / 1546, 400.0 * 960 / 1159]],
    )


def test_large_frame_uses_1920_pixel_sahi_tiles_and_restores_coordinates():
    detector = detector_with_fake_model((1920, 1920))

    results = detector._predict_sahi(Image.new("RGB", (4640, 3480)), threshold=0.5)

    assert detector.model.sahi_call == ((1547, 1160), 0.5, 0.2)
    np.testing.assert_allclose(
        results["bboxes"],
        [[100.0 * 4640 / 1547, 600.0, 300.0 * 4640 / 1547, 1200.0]],
    )


def test_progress_reports_the_full_sahi_batch():
    progress = []
    detector = detector_with_fake_model((530, 530), progress.append)

    detector._predict_sahi(Image.new("RGB", (1280, 960)), threshold=0.5)

    assert len(progress) == 2
    assert progress[0]["status"] == "running"
    assert progress[0]["local_slices"] == 9
    assert progress[0]["total"] == 10
    assert progress[1]["status"] == "complete"
    assert progress[1]["completed"] == 10
