"""ML-based object detector using LightlyTrain LTDETR."""

import time
from collections.abc import Callable
from pathlib import Path

import numpy as np
import rclpy.logging
import supervision as sv
from PIL import Image

from .base_detector import BaseDetector


class MLDetector(BaseDetector):
    """Object detector using a LightlyTrain LTDETR checkpoint."""

    def __init__(
        self,
        model_path: str,
        source_tile_size: tuple[int, int],
        overlap: float,
        progress_callback: Callable[[dict], None] | None = None,
    ):
        """Initialize the ML detector.

        Args:
            model_path: Filesystem path to the LTDETR checkpoint.
            source_tile_size: Native camera crop width and height for SAHI.
            overlap: Fractional overlap between adjacent slices.
            progress_callback: Optional observer for inference progress.
        """
        if len(source_tile_size) != 2 or min(source_tile_size) <= 0:
            raise ValueError("source_tile_size must contain two positive values")
        if not 0.0 <= overlap < 1.0:
            raise ValueError("overlap must be in the range [0, 1)")

        self.model_path = Path(model_path).expanduser()
        self.source_tile_size = source_tile_size
        self.overlap = overlap
        self.progress_callback = progress_callback
        self._run_id = 0
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

        image = Image.fromarray(frame[:, :, ::-1])
        results = self.model.predict(image, threshold=threshold)
        # results = self._predict_sahi(image, threshold)

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

    def _predict_sahi(self, image: Image.Image, threshold: float):
        source_tile_width, source_tile_height = self.source_tile_size
        if image.width <= source_tile_width and image.height <= source_tile_height:
            resized = image
            local_slices = 1
            total_views = 1
            use_sahi = False
        else:
            model_height, model_width = self.model.image_size
            resized_width = max(
                model_width,
                round(image.width * model_width / source_tile_width),
            )
            resized_height = max(
                model_height,
                round(image.height * model_height / source_tile_height),
            )
            resized = image.resize(
                (resized_width, resized_height),
                resample=Image.Resampling.LANCZOS,
            )
            local_slices = (
                self._axis_slice_count(resized_width, model_width)
                * self._axis_slice_count(resized_height, model_height)
            )
            total_views = local_slices + 1
            use_sahi = True

        self._run_id += 1
        progress = {
            "run_id": self._run_id,
            "active": True,
            "status": "running",
            "completed": 0,
            "total": total_views,
            "local_slices": local_slices,
            "slice_width": source_tile_width,
            "slice_height": source_tile_height,
            "overlap": self.overlap,
            "elapsed_sec": 0.0,
        }
        self._notify_progress(progress)
        started = time.monotonic()

        try:
            if use_sahi:
                results = self.model.predict_sahi(
                    resized,
                    threshold=threshold,
                    overlap=self.overlap,
                )
            else:
                results = self.model.predict(image, threshold=threshold)

            boxes = results.get("bboxes")
            if use_sahi and boxes is not None and len(boxes) > 0:
                boxes[:, [0, 2]] *= image.width / resized_width
                boxes[:, [1, 3]] *= image.height / resized_height
        except Exception as exc:
            self._notify_progress({
                **progress,
                "active": False,
                "status": "error",
                "elapsed_sec": time.monotonic() - started,
                "error": str(exc),
            })
            raise

        self._notify_progress({
            **progress,
            "active": False,
            "status": "complete",
            "completed": total_views,
            "elapsed_sec": time.monotonic() - started,
        })
        return results

    def _axis_slice_count(self, image_size: int, tile_size: int) -> int:
        if image_size <= tile_size:
            return 1
        step = max(1, int((1.0 - self.overlap) * tile_size))
        last_start = image_size - tile_size
        count = (last_start // step) + 1
        if (count - 1) * step != last_start:
            count += 1
        return count

    def _notify_progress(self, progress: dict) -> None:
        if self.progress_callback is None:
            return
        try:
            self.progress_callback(progress)
        except Exception as exc:
            self._logger.warning(f"SAHI progress callback failed: {exc}")

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
