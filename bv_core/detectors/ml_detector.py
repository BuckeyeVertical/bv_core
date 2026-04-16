"""ML-based object detector using LightlyTrain LTDETR."""

from pathlib import Path

import numpy as np
import rclpy.logging
import supervision as sv

from .base_detector import BaseDetector


class MLDetector(BaseDetector):
    """Object detector using a LightlyTrain LTDETR checkpoint."""

    def __init__(self, model_path: str):
        """Initialize the ML detector.

        Args:
            model_path: Filesystem path to the LTDETR checkpoint.
        """
        self.model_path = Path(model_path).expanduser()
        self.model = None
        self._logger = rclpy.logging.get_logger("ml_detector")

    def start(self) -> None:
        """Load the LTDETR model."""
        if self.model is None:
            self.model = self._create_model()

    def stop(self) -> None:
        """Release model resources."""
        self.model = None

    def process_frame(
        self,
        frame: np.ndarray,
        threshold: float = 0.5,
        **kwargs,
    ) -> sv.Detections:
        """Run LTDETR inference on a single frame.

        Args:
            frame: Input image as numpy array (H, W, C) in BGR format.
            threshold: Minimum confidence required to keep a detection.

        Returns:
            Detections surviving confidence filtering.
        """
        if self.model is None:
            self.model = self._create_model()

        from PIL import Image

        image = Image.fromarray(frame[:, :, ::-1])
        results = self.model.predict(image)

        labels = self._to_numpy(results.get("labels"), np.int32)
        boxes = self._to_numpy(results.get("bboxes"), np.float32).reshape((-1, 4))
        scores = self._to_numpy(results.get("scores"), np.float32)

        if len(labels) == 0 or len(boxes) == 0 or len(scores) == 0:
            return sv.Detections.empty()

        keep = scores >= float(threshold)
        if not np.any(keep):
            return sv.Detections.empty()

        return sv.Detections(
            xyxy=boxes[keep],
            confidence=scores[keep],
            class_id=labels[keep],
        )

    def _create_model(self):
        """Initialize the LTDETR model."""
        if not self.model_path.is_file():
            raise RuntimeError(f"LTDETR weights not found: {self.model_path}")

        try:
            import lightly_train
        except ImportError as exc:
            raise RuntimeError(
                "Missing dependency 'lightly_train'. Install it with "
                "`python3 -m pip install lightly-train`."
            ) from exc

        self._logger.info(f"Loading LTDETR model from {self.model_path}")
        return lightly_train.load_model(str(self.model_path))

    @staticmethod
    def _to_numpy(value, dtype) -> np.ndarray:
        """Convert LTDETR outputs to numpy arrays without hard-coding torch."""
        if value is None:
            return np.array([], dtype=dtype)
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        if hasattr(value, "numpy"):
            value = value.numpy()
        return np.asarray(value, dtype=dtype)
