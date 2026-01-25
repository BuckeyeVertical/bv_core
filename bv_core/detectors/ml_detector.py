"""ML-based object detector using RF-DETR."""

import os

import numpy as np
import rclpy.logging
import supervision as sv
import torch
from PIL import Image
from rfdetr import RFDETRBase

from .base_detector import BaseDetector


class MLDetector(BaseDetector):
    """Object detector using RF-DETR model with tiled inference."""

    def __init__(self, batch_size: int = 16, resolution: int = 728):
        """Initialize the ML detector.

        Args:
            batch_size: Number of tiles to process in parallel.
            resolution: Input resolution for the model (tile size).
        """
        self.batch_size = batch_size
        self.resolution = resolution
        self.model = None
        self.frame_save_cnt = 0

    def start(self) -> None:
        """Load the RF-DETR model."""
        if self.model is None:
            self.model = self._create_model()

    def stop(self) -> None:
        """Release model resources."""
        self.model = None
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    def process_frame(
        self,
        frame: np.ndarray,
        overlap: int = 100,
        threshold: float = 0.5,
        **kwargs,
    ) -> sv.Detections:
        """Process an image by tiling, batched inference, and global NMS.

        Args:
            frame: Input image as numpy array (H, W, C).
            overlap: Overlap between adjacent tiles in pixels.
            threshold: Detection confidence threshold.

        Returns:
            Merged detections with NMS applied.
        """
        if self.model is None:
            rclpy.logging.get_logger("ml_detector").info("Creating model")
            self.model = self._create_model()

        tiles_with_positions = self._create_tiles(frame, overlap)
        tiles, offsets = zip(*tiles_with_positions)

        detections_list = []
        for i in range(0, len(tiles), self.batch_size):
            batch = list(tiles[i : i + self.batch_size])
            dets = self._pad_and_predict(batch, threshold)
            detections_list.extend(dets)

        all_dets = []
        for dets, (x_off, y_off) in zip(detections_list, offsets):
            if len(dets.xyxy) == 0:
                continue

            boxes = dets.xyxy.copy()
            boxes[:, [0, 2]] += x_off
            boxes[:, [1, 3]] += y_off

            all_dets.append(
                sv.Detections(
                    xyxy=boxes,
                    confidence=dets.confidence,
                    class_id=dets.class_id,
                )
            )

        if not all_dets:
            return sv.Detections.empty()

        merged = sv.Detections.merge(all_dets)
        return merged.with_nms(threshold=0.45)

    def annotate_frame(
        self,
        frame: np.ndarray,
        detections: sv.Detections,
        labels: list,
    ) -> np.ndarray:
        """Draw bounding boxes and labels on frame."""
        annotated_frame = frame.copy()
        annotated_frame = sv.BoxAnnotator().annotate(annotated_frame, detections)
        annotated_frame = sv.LabelAnnotator().annotate(
            annotated_frame, detections, labels
        )
        return annotated_frame

    def save_frame(self, frame: np.ndarray, output_dir: str) -> None:
        """Save annotated frame to disk."""
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, f"{self.frame_save_cnt}_annotated.jpg")
        Image.fromarray(frame).save(output_path)
        self.frame_save_cnt += 1

    def _create_tiles(self, image: np.ndarray, overlap: int = 100) -> list:
        """Split an image into overlapping tiles.

        Args:
            image: Input image as numpy array (H, W, C).
            overlap: Overlap between adjacent tiles in pixels.

        Returns:
            List of (tile, (x_offset, y_offset)) tuples.
        """
        tile_size = self.resolution
        width, height = image.shape[1], image.shape[0]
        tiles = []

        x_positions = list(range(0, width - overlap, tile_size - overlap))
        if width not in x_positions and x_positions:
            x_positions.append(max(0, width - tile_size))
        if not x_positions:
            x_positions = [0]

        y_positions = list(range(0, height - overlap, tile_size - overlap))
        if height not in y_positions and y_positions:
            y_positions.append(max(0, height - tile_size))
        if not y_positions:
            y_positions = [0]

        for y in y_positions:
            for x in x_positions:
                x_end = min(x + tile_size, width)
                y_end = min(y + tile_size, height)
                x_start = max(0, x_end - tile_size)
                y_start = max(0, y_end - tile_size)

                tile = image[y_start:y_end, x_start:x_end, :]
                tiles.append((tile, (x_start, y_start)))

        return tiles

    def _pad_and_predict(self, tiles: list, threshold: float) -> list:
        """Run batched inference, padding if needed.

        Args:
            tiles: List of tile images as numpy arrays.
            threshold: Detection confidence threshold.

        Returns:
            List of detections for each tile.
        """
        B = len(tiles)
        if B < self.batch_size:
            dummy = np.zeros_like(tiles[0])
            tiles = tiles + [dummy] * (self.batch_size - B)
            trim_to = B
        else:
            trim_to = None

        outputs = self.model.predict(tiles, threshold=threshold)

        if trim_to is not None:
            outputs = outputs[:trim_to]

        return outputs

    def _create_model(self, dtype=None):
        """Initialize the RF-DETR model."""
        if dtype is None:
            if torch.cuda.is_available():
                dtype = torch.float16
            else:
                dtype = torch.float32

        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            device_info = "cuda"
        else:
            device_info = "cpu"

        model = RFDETRBase(resolution=self.resolution)
        model.optimize_for_inference(batch_size=self.batch_size, dtype=dtype)

        rclpy.logging.get_logger("ml_detector").info(
            f"Model created with batch size: {self.batch_size} "
            f"(dtype={dtype}, device={device_info})"
        )

        return model
