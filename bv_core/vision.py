#!/usr/bin/env python3

import sys
print(f"[DEBUG] Running with interpreter: {sys.executable}")

import os
import glob
import supervision as sv
import numpy as np
from PIL import Image
from rfdetr import RFDETRBase
from rfdetr.util.coco_classes import COCO_CLASSES
import torch
import cv2


class Localizer():
    def __init__(self, k):
        self.k = k
        pass

    def estimate_locations(self, bounding_boxes: list):
        # Return the estimated lat lon from the bounding boxes
        # Use the compute_center, project_lat_lon, cv2 solvePNP, and DBSCAN functions
        pass

    def compute_center(self, bounding_boxes: list):
        # Compute the center of the bounding boxes and return a list of centroids
        pass

    def project_lat_lon(self, drone_global_pos, object_rel_pos):
        # Find the object global position given the drone's global position and objects relative position
        pass

class Detector():
    def __init__(self, batch_size=16, resolution=728):
        self.batch_size = batch_size
        self.resolution = resolution
        self.model = self.create_model()
        self.frame_save_cnt = 0

    def create_tiles(self, image, overlap=100):
        """
        Split an image into tiles with overlap.
        
        Args:
            image: PIL Image to be tiled
            tile_size: Size of each square tile
            overlap: Overlap between adjacent tiles in pixels
        
        Returns:
            List of (tile, (x_offset, y_offset)) where tile is a PIL Image and
            (x_offset, y_offset) is the position of the tile in the original image
        """
        tile_size = self.resolution
        width, height = image.shape[1], image.shape[0]
        tiles = []
        
        # Calculate positions where tiles should start
        x_positions = list(range(0, width - overlap, tile_size - overlap))
        if width not in x_positions and x_positions:
            x_positions.append(max(0, width - tile_size))
        if not x_positions:  # Image is smaller than tile_size
            x_positions = [0]
            
        y_positions = list(range(0, height - overlap, tile_size - overlap))
        if height not in y_positions and y_positions:
            y_positions.append(max(0, height - tile_size))
        if not y_positions:  # Image is smaller than tile_size
            y_positions = [0]
        
        # Create tiles
        for y in y_positions:
            for x in x_positions:
                # Adjust position if we're at the edge of the image
                x_end = min(x + tile_size, width)
                y_end = min(y + tile_size, height)
                x_start = max(0, x_end - tile_size)
                y_start = max(0, y_end - tile_size)
                
                # Extract tile
                tile = image[y_start:y_end, x_start:x_end, :]
                tiles.append((tile, (x_start, y_start)))
        
        return tiles

    def pad_and_predict(self, tiles, threshold):
        # tiles: list of H×W×C numpy arrays (or torch tensors)
        # model.predict wants exactly `batch_size` inputs.

        # 1) prepare batch
        B = len(tiles)
        if B < self.batch_size:
            # assume tiles[0] exists; make dummy using its shape
            dummy = np.zeros_like(tiles[0])
            tiles = tiles + [dummy] * (self.batch_size - B)
            trim_to = B
        else:
            trim_to = None

        # 2) run inference
        outputs = self.model.predict(tiles, threshold=threshold)

        # 3) if we padded, slice off the extra outputs
        if trim_to is not None:
            # assume outputs is a list of detections per‐tile
            outputs = outputs[:trim_to]

        return outputs

    def process_frame(self, frame, overlap: int = 100, threshold: float = 0.5):
        """
        Process an image by tiling, using RF-DETR's batch API for inference,
        then combine results and apply global NMS.
        """
        # 1) Tile the image and unzip into lists of tiles + offsets
        tiles_with_positions = self.create_tiles(frame, overlap)
        tiles, offsets = zip(*tiles_with_positions)

        if self.model == None:
            print("Creating model")
            self.model = self.create_model()

        detections_list = []

        for i in range(0, len(tiles), self.batch_size):
            batch = list(tiles[i:i + self.batch_size])
            dets = self.pad_and_predict(batch, threshold)
            detections_list.extend(dets)

        # 3) Adjust boxes back to full-image coords
        all_dets = []
        for dets, (x_off, y_off) in zip(detections_list, offsets):
            if len(dets.xyxy) == 0:
                continue

            boxes = dets.xyxy.copy()               # NumPy array copy
            boxes[:, [0, 2]] += x_off              # x_min, x_max
            boxes[:, [1, 3]] += y_off              # y_min, y_max

            all_dets.append(sv.Detections(
                xyxy=boxes,
                confidence=dets.confidence,
                class_id=dets.class_id
            ))

        # 4) Merge & run global NMS
        if not all_dets:
            return sv.Detections.empty()

        merged = sv.Detections.merge(all_dets)
        return merged.with_nms(threshold=0.45)
    
    def annotate_frame(self, frame, detections, labels):
        annotated_frame = frame.copy()
        annotated_frame = sv.BoxAnnotator().annotate(annotated_frame, detections)
        annotated_frame = sv.LabelAnnotator().annotate(annotated_frame, detections, labels)
        return annotated_frame

    def save_frame(self, frame, output_dir):
        output_path = os.path.join(output_dir, f"{self.frame_save_cnt}_annotated.jpg")
        Image.fromarray(frame).save(output_path)
        self.frame_save_cnt += 1

    def create_model(self, dtype=torch.float16):
        # Initialize the model
        torch.cuda.empty_cache()
        model = RFDETRBase(resolution=self.resolution)

        model.optimize_for_inference(batch_size=self.batch_size, dtype=dtype)

        print(f"Model created with batch size: {self.batch_size}")

        return model