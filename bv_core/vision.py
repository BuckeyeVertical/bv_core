#!/usr/bin/env python3

import sys

import os
import glob
import supervision as sv
import numpy as np
from PIL import Image
from rfdetr import RFDETRBase
from rfdetr.util.coco_classes import COCO_CLASSES
import torch

import cv2
from sklearn.cluster import DBSCAN
from geographiclib.geodesic import Geodesic
import math
from scipy.spatial.transform import Rotation as R
import rclpy
import rclpy.logging
import yaml
from ament_index_python.packages import get_package_share_directory

class Localizer:
    def __init__(self,
                 camera_matrix: np.ndarray,
                 dist_coeffs: np.ndarray,
                 eps: float = 0.0005,
                 min_samples: int = 1):
        """
        Args:
            camera_matrix: 3x3 intrinsic matrix
            dist_coeffs: distortion coefficients
            eps: clustering radius in degrees (approx ~50m)
            min_samples: minimum points to form a cluster
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.eps = eps
        self.min_samples = min_samples
        self.geod = Geodesic.WGS84

        filtering_yaml = os.path.join(
            get_package_share_directory('bv_core'),
            'config',
            'filtering_params.yaml'
        )

        with open(filtering_yaml, 'r') as f:
            cfg = yaml.safe_load(f)

        self.q_mount = cfg.get('camera_orientation', [0.0]*4)

    def get_lat_lon(self,
                pixel_centers: list,
                drone_global_pos: tuple,
                drone_orientation: tuple  # (qx, qy, qz, qw)
               ) -> list:
        """
        Convert pixel detections to global lat/lon using pinhole geometry,
        GeographicLib for geodesic projection, and the drone's orientation.

        Args:
            pixel_centers: list of (u, v, class_id)
            drone_global_pos: (lat, lon, alt) in degrees and meters
            drone_orientation: (qx, qy, qz, qw) quaternion giving rotation
                            from CAMERA (or body) frame into ENU frame

        Returns:
            List of (lat, lon, class_id) detections.
        """

        if not pixel_centers:
            return []

        # 1) Undistort to normalized image plane
        pts = np.array([(u, v) for u, v, _ in pixel_centers], dtype=np.float32)
        pts = pts.reshape(-1, 1, 2)
        norm = cv2.undistortPoints(
            pts,
            self.camera_matrix,
            self.dist_coeffs,
            # P=self.camera_matrix
        ).reshape(-1, 2)

        # 2) Build camera‐frame rays (assuming z forward = optical axis)
        dirs_cam = np.hstack([norm, np.ones((len(norm), 1), dtype=np.float32)])
        # (optionally normalize to unit length):
        dirs_cam /= np.linalg.norm(dirs_cam, axis=1, keepdims=True)

        r_mount = R.from_quat(self.q_mount)
        r_drone = R.from_quat(drone_orientation, scalar_first=False)
        r_total = r_drone * r_mount

        # 3) Get rotation from camera → ENU
        #    Assumes quaternion is [qx, qy, qz, qw] in scipy convention
        R_cam2enu = r_total.as_matrix()

        # 4) For each ray, find intersection with ground plane z=0
        lat0, lon0, alt = drone_global_pos
        results = []
        for dir_cam, (_, _, class_id) in zip(dirs_cam, pixel_centers):
            # rotate into ENU
            dir_enu = R_cam2enu.dot(dir_cam)          # [east, north, up]
            # compute scale t such that drone_pos + t·dir_enu lies on z = 0
            # (drone at altitude `alt` above ground)
            t = alt / -dir_enu[2]
            east_offset  = dir_enu[0] * t
            north_offset = dir_enu[1] * t

            # 5) project via GeographicLib
            az   = math.degrees(math.atan2(east_offset, north_offset))
            dist = math.hypot(east_offset, north_offset)
            geo  = self.geod.Direct(lat0, lon0, az, dist)

            results.append((geo['lat2'], geo['lon2'], class_id))

        i = 0  # or any index
        rclpy.logging.get_logger("filtering_node").info(f"pixel: {pixel_centers[i]}")
        rclpy.logging.get_logger("filtering_node").info(f"norm: {norm[i]}")
        rclpy.logging.get_logger("filtering_node").info(f"dir_cam: {dirs_cam[i]}")
        rclpy.logging.get_logger("filtering_node").info(f"dir_enu: {dir_enu}")
        rclpy.logging.get_logger("filtering_node").info(f"t: {t}, → E,N: {east_offset}, {north_offset}")

        for res in results:
            rclpy.logging.get_logger("filtering_node").info(f"Result: {res[0]}, {res[1]}, {COCO_CLASSES[int(res[2])]}")

        return results

    def estimate_locations(self,
                           detections: list) -> list:
        """
        Cluster lat/lon detections and return unique object locations.

        Args:
            detections: list of (lat, lon, class_id)
        Returns:
            List of (lat, lon, class_id) cluster centers
        """
        if not detections:
            return []

        coords = np.array([[lat, lon] for lat, lon, _ in detections])
        classes = [cls for _, _, cls in detections]

        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(coords)
        labels = clustering.labels_

        cluster_centers = []
        for lbl in set(labels) - {-1}:
            mask = labels == lbl
            pts = coords[mask]
            cls_ids = [classes[i] for i, m in enumerate(mask) if m]
            # choose most frequent class in cluster
            class_id = max(set(cls_ids), key=cls_ids.count)
            center_lat, center_lon = pts.mean(axis=0)
            cluster_centers.append((center_lat, center_lon, class_id))

        return cluster_centers

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
            rclpy.logging.get_logger("filtering_node").info("Creating model")
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

        rclpy.logging.get_logger("filtering_node").info(f"Model created with batch size: {self.batch_size}")

        return model