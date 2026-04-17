#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

import cv2
import numpy as np
import glob
import os
import re
import shutil
import time
from collections import defaultdict
from datetime import datetime


class StitchingNode(Node):
    def __init__(self):
        super().__init__('stitching_node')

        # configurable settings with defaults
        self.declare_parameter('input_dir', 'raw_frames')
        self.declare_parameter('output_dir', 'stitching_results')
        self.declare_parameter('max_width', 1500)
        self.declare_parameter('max_images_per_row', 50)
        self.declare_parameter('save_intermediate_rows', False)
        self.declare_parameter('feature_count', 5000)
        self.declare_parameter('match_confidence', 0.7)  # lowe's ratio threshold
        self.declare_parameter('gap_width', 50)
        self.declare_parameter('save_failed_frames', True)
        self._has_stitched = False # stops from stitching on every return publish


        # subscribe to mission_state
        self.subscription = self.create_subscription(
            String,                      
            '/mission_state',            
            self.mission_state_callback, 
            10                           
        )

        self.get_logger().info(
            "Stitching Service Node Ready. "
            "Waiting for return state..."
        )

        self._log_hardware_info()

    def mission_state_callback(self, msg):
        if msg.data == "return" and not self._has_stitched:
            self._has_stitched = True
            self.get_logger().info("Mission state is 'return'. Starting stitch...")
        
            try:
                success, message = self._perform_stitching()
                if success:
                    self.get_logger().info(f"Stitching complete: {message}")
                else:
                    self.get_logger().error(f"Stitching failed: {message}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

    def _log_hardware_info(self):
        try:
            cuda_devices = cv2.cuda.getCudaEnabledDeviceCount()
            if cuda_devices > 0:
                self.get_logger().info(
                    f"CUDA available: {cuda_devices} device(s)."
                )
            else:
                self.get_logger().info(
                    "CUDA not available. Using CPU for stitching."
                )
        except AttributeError:
            self.get_logger().info(
                "OpenCV CUDA module not available. Using CPU."
            )


    def _parse_row_col(self, filepath):
        # "row3_7.jpg" -> (3, 7)
        basename = os.path.splitext(os.path.basename(filepath))[0]
        match = re.match(r'^row(\d+)_(\d+)$', basename, re.IGNORECASE)
        if match:
            return int(match.group(1)), int(match.group(2))
        return None

    def _discover_and_group_images(self, input_dir):
        # scan for all image files in the input directory
        extensions = ('*.jpg', '*.JPG', '*.jpeg', '*.JPEG', '*.png', '*.PNG')
        all_paths = []
        for ext in extensions:
            all_paths.extend(glob.glob(os.path.join(input_dir, ext)))
        all_paths = sorted(set(all_paths))

        # skip any images with "annotated" in the name
        all_paths = [
            p for p in all_paths
            if "annotated" not in os.path.basename(p).lower()
        ]

        # group images into a dict: {row_num: [(col, path), ...]}
        rows = defaultdict(list)
        unmatched = []

        for p in all_paths:
            parsed = self._parse_row_col(p)
            if parsed:
                row_num, col_num = parsed
                rows[row_num].append((col_num, p))
            else:
                unmatched.append(p)

        if unmatched:
            self.get_logger().warning(
                f"Skipping {len(unmatched)} images that don't match "
                f"'row{{N}}_{{M}}' naming convention."
            )

        # sort each row by column number so images are left to right
        grouped = {}
        for row_num, items in sorted(rows.items()):
            items.sort(key=lambda x: x[0])
            grouped[row_num] = [path for _, path in items]

        return grouped

    def _load_and_resize(self, paths, max_width):
        images = []
        for p in paths:
            img = cv2.imread(p)
            if img is None:
                self.get_logger().warning(f"Failed to load image: {p}")
                continue

            # downscale if too wide
            h, w = img.shape[:2]
            if w > max_width:
                scale = max_width / w
                new_dim = (max_width, int(h * scale))
                img = cv2.resize(img, new_dim, interpolation=cv2.INTER_AREA)

            images.append(img)

        return images

    def _find_homography(self, img1, img2, feature_count, match_confidence):
        # feature detection needs grayscale
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        # find distinctive points and their descriptors in each image
        orb = cv2.ORB_create(nfeatures=feature_count)
        kp1, des1 = orb.detectAndCompute(gray1, None)
        kp2, des2 = orb.detectAndCompute(gray2, None)

        if des1 is None or des2 is None:
            return False, "No features detected in one or both images"

        if len(kp1) < 10 or len(kp2) < 10:
            return False, (
                f"Too few features: img1={len(kp1)}, img2={len(kp2)}"
            )

        # for each feature in img1, find the 2 closest matches in img2
        bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        try:
            matches = bf.knnMatch(des1, des2, k=2)
        except cv2.error as e:
            return False, f"Feature matching failed: {e}"

        # keep match only if it is clearly best option
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < match_confidence * n.distance:
                    good_matches.append(m)

        self.get_logger().info(
            f"    Features: {len(kp1)}/{len(kp2)}, "
            f"Good matches: {len(good_matches)}"
        )

        if len(good_matches) < 10:
            return False, (
                f"Not enough good matches: {len(good_matches)} "
                f"(need at least 10)"
            )

        # extract the coordinates of matched points
        pts1 = np.float32(
            [kp1[m.queryIdx].pt for m in good_matches]
        ).reshape(-1, 1, 2)
        pts2 = np.float32(
            [kp2[m.trainIdx].pt for m in good_matches]
        ).reshape(-1, 1, 2)

        # compute the 3x3 matrix that maps img2 coords to img1 coords
        # RANSAC finds transformation that works for the most points
        H, mask = cv2.findHomography(pts2, pts1, cv2.RANSAC, 5.0)

        if H is None:
            return False, "Homography estimation failed (RANSAC)"

        # determinant should be close to 1 cause aerial translations
        det = np.linalg.det(H[:2, :2])
        if det < 0.5 or det > 2.0:
            self.get_logger().warning(
                f"    Suspicious homography determinant: {det:.3f} "
                f"(expected ~1.0 for aerial imagery)"
            )

        inliers = np.sum(mask) if mask is not None else 0
        self.get_logger().info(
            f"    Homography: {inliers} inliers, det={det:.3f}"
        )

        return True, H

    def _stitch_pair(self, img1, img2, H):
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]

        # figure out where img2's corners land after applying the homography
        corners_img2 = np.float32([
            [0, 0], [w2, 0], [w2, h2], [0, h2]
        ]).reshape(-1, 1, 2)
        warped_corners = cv2.perspectiveTransform(corners_img2, H)

        corners_img1 = np.float32([
            [0, 0], [w1, 0], [w1, h1], [0, h1]
        ]).reshape(-1, 1, 2)

        # bounding box of both images = size of output canvas
        all_corners = np.concatenate(
            [corners_img1, warped_corners], axis=0
        )
        x_min = int(np.floor(all_corners[:, 0, 0].min()))
        y_min = int(np.floor(all_corners[:, 0, 1].min()))
        x_max = int(np.ceil(all_corners[:, 0, 0].max()))
        y_max = int(np.ceil(all_corners[:, 0, 1].max()))

        # shift everything so all coordinates are positive
        translation = np.array([
            [1, 0, -x_min],
            [0, 1, -y_min],
            [0, 0, 1]
        ], dtype=np.float64)

        canvas_w = x_max - x_min
        canvas_h = y_max - y_min

        # if homography messes up, shouldn't destroy memory with giant mosaic
        max_canvas = max(w1, w2) * 6
        if canvas_w > max_canvas or canvas_h > max_canvas:
            self.get_logger().warning(
                f"Canvas too large ({canvas_w}x{canvas_h}), "
                f"clamping to {max_canvas}"
            )
            canvas_w = min(canvas_w, max_canvas)
            canvas_h = min(canvas_h, max_canvas)

        # warp img2 onto the canvas
        result = cv2.warpPerspective(
            img2, translation @ H, (canvas_w, canvas_h)
        )

        # now place img1 onto the canvas
        tx = -x_min
        ty = -y_min

        # fit to canvas
        y_start = max(0, ty)
        y_end = min(canvas_h, ty + h1)
        x_start = max(0, tx)
        x_end = min(canvas_w, tx + w1)

        img1_y_start = y_start - ty
        img1_y_end = img1_y_start + (y_end - y_start)
        img1_x_start = x_start - tx
        img1_x_end = img1_x_start + (x_end - x_start)

        roi = img1[img1_y_start:img1_y_end, img1_x_start:img1_x_end]
        canvas_roi = result[y_start:y_end, x_start:x_end]

        # where images overlap, blend 50/50
        img1_mask = np.any(roi > 0, axis=2)
        img2_mask = np.any(canvas_roi > 0, axis=2)
        overlap = img1_mask & img2_mask

        # like a venn diagram, where only image one just place
        canvas_roi[img1_mask & ~overlap] = roi[img1_mask & ~overlap]

        if np.any(overlap):
            canvas_roi[overlap] = (
                roi[overlap].astype(np.float32) * 0.5 +
                canvas_roi[overlap].astype(np.float32) * 0.5
            ).astype(np.uint8)

        result[y_start:y_end, x_start:x_end] = canvas_roi

        return result

    def _concatenate_segments(self, segments):
        # join multiple mosaic segments horizontally with red gap bars
        gap_width = self.get_parameter('gap_width').value
        max_h = max(seg.shape[0] for seg in segments)

        # calculate total canvas width including gaps
        total_w = (
            sum(seg.shape[1] for seg in segments)
            + (len(segments) - 1) * gap_width
        )

        canvas = np.zeros((max_h, total_w, 3), dtype=np.uint8)
        x_offset = 0

        for i, seg in enumerate(segments):
            h, w = seg.shape[:2]
            y_offset = (max_h - h) // 2  # vertically center if heights differ
            canvas[y_offset:y_offset + h, x_offset:x_offset + w] = seg
            x_offset += w

            # draw red gap bar between segments
            if i < len(segments) - 1:
                canvas[:, x_offset:x_offset + gap_width] = (0, 0, 255)
                x_offset += gap_width

        return canvas

    def _stitch_row_with_fallback(self, images, label=""):
        # stitch images with skip-and-continue fallback for bad frames
        feature_count = self.get_parameter('feature_count').value
        match_confidence = self.get_parameter('match_confidence').value

        if len(images) == 0:
            return False, f"No images for {label}", {}
        if len(images) == 1:
            return True, images[0], {
                'segments': [(0, 0)],
                'failed_frames': [],
                'gap_count': 0,
            }

        segments = []
        failed_frames = []
        current_indices = []
        current_mosaic = None

        i = 0
        while i < len(images):
            # start a new segment
            if not current_indices:
                current_indices = [i]
                current_mosaic = images[i]
                i += 1
                continue

            self.get_logger().info(
                f"  {label}: Stitching image {i+1}/{len(images)}..."
            )

            # try to stitch frame i to the current mosaic
            success, result = self._find_homography(
                current_mosaic, images[i], feature_count, match_confidence
            )

            if success:
                current_mosaic = self._stitch_pair(
                    current_mosaic, images[i], result
                )
                current_indices.append(i)
                self.get_logger().info(
                    f"  {label}: Mosaic now "
                    f"{current_mosaic.shape[1]}x{current_mosaic.shape[0]}px"
                )
                i += 1
                continue

            self.get_logger().warning(
                f"  {label}: Frame {i+1} failed: {result}"
            )

            # frame i failed, try skipping to i+1
            if i + 1 < len(images):
                self.get_logger().info(
                    f"  {label}: Trying frame {i+2} instead..."
                )
                skip_ok, skip_res = self._find_homography(
                    current_mosaic, images[i + 1],
                    feature_count, match_confidence
                )

                if skip_ok:
                    # skip worked, continue from i+1
                    current_mosaic = self._stitch_pair(
                        current_mosaic, images[i + 1], skip_res
                    )
                    failed_frames.append(i)
                    current_indices.append(i + 1)
                    self.get_logger().info(
                        f"  {label}: Skipped frame {i+1}, "
                        f"continued with {i+2}. "
                        f"Mosaic now "
                        f"{current_mosaic.shape[1]}x"
                        f"{current_mosaic.shape[0]}px"
                    )
                    i += 2
                    continue

            # both i and i+1 failed, save segment and start new one
            segments.append((current_mosaic, list(current_indices)))
            failed_frames.append(i)
            self.get_logger().info(
                f"  {label}: Segment break after frame {i}. "
                f"Saved segment [{current_indices[0]+1}..{current_indices[-1]+1}]"
            )
            current_indices = []
            current_mosaic = None
            i += 1

        # save final segment
        if current_indices:
            segments.append((current_mosaic, list(current_indices)))

        if not segments:
            return False, f"{label}: All frames failed to stitch", {}

        metadata = {
            'segments': [
                (idxs[0], idxs[-1]) for _, idxs in segments
            ],
            'failed_frames': failed_frames,
            'gap_count': len(segments) - 1,
        }

        # single segment means no gaps, multiple means we need to concatenate
        if len(segments) == 1:
            mosaic = segments[0][0]
        else:
            self.get_logger().info(
                f"  {label}: Concatenating {len(segments)} segments "
                f"with {len(segments)-1} gap(s)"
            )
            mosaic = self._concatenate_segments(
                [seg for seg, _ in segments]
            )

        return True, mosaic, metadata

    def _stitch_sequential(self, images, label=""):
        # stitch images one at a time
        feature_count = self.get_parameter('feature_count').value
        match_confidence = self.get_parameter('match_confidence').value

        if len(images) == 0:
            return False, f"No images for {label}"
        if len(images) == 1:
            return True, images[0]

        mosaic = images[0]

        for i in range(1, len(images)):
            self.get_logger().info(
                f"  {label}: Stitching image {i+1}/{len(images)}..."
            )

            success, result = self._find_homography(
                mosaic, images[i], feature_count, match_confidence
            )

            if not success:
                self.get_logger().error(
                    f"  {label}: Failed at image {i+1}: {result}"
                )
                return False, (
                    f"{label}: Failed at image {i+1}/{len(images)}"
                    f" - {result}"
                )

            H = result
            mosaic = self._stitch_pair(mosaic, images[i], H)

            self.get_logger().info(
                f"  {label}: Mosaic now "
                f"{mosaic.shape[1]}x{mosaic.shape[0]}px"
            )

        return True, mosaic

    def _crop_black_borders(self, img):
        # get rid of black pixels from stitching/blending
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 5, 255, cv2.THRESH_BINARY)

        coords = cv2.findNonZero(thresh)
        if coords is None:
            return img

        x, y, w, h = cv2.boundingRect(coords)

        margin = 5
        x = max(0, x - margin)
        y = max(0, y - margin)
        w = min(img.shape[1] - x, w + 2 * margin)
        h = min(img.shape[0] - y, h + 2 * margin)

        return img[y:y+h, x:x+w]

    def _perform_stitching(self):
        input_dir = self.get_parameter('input_dir').value
        output_dir = self.get_parameter('output_dir').value
        max_width = max(100, self.get_parameter('max_width').value)
        max_per_row = max(2, self.get_parameter('max_images_per_row').value)
        save_intermediate = self.get_parameter(
            'save_intermediate_rows').value

        if not os.path.exists(input_dir):
            return False, f"Input directory not found: {input_dir}"

        # build dict of rownum -> images
        row_groups = self._discover_and_group_images(input_dir)

        if not row_groups:
            return False, "No images matching 'row{N}_{M}' pattern found."

        total_images = sum(len(paths) for paths in row_groups.values())
        self.get_logger().info(
            f"Discovered {total_images} images across "
            f"{len(row_groups)} rows: "
            f"{{{', '.join(f'row{k}: {len(v)} imgs' for k, v in row_groups.items())}}}"
        )

        # move files to temp just incase images get added. prevents random errors
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        work_dir = os.path.join(input_dir, f".work_{timestamp}")

        try:
            os.makedirs(work_dir)
        except OSError as e:
            return False, f"Failed to create working directory: {e}"

        all_work_paths = []
        work_row_groups = defaultdict(list)

        try:
            for row_num, paths in row_groups.items():
                for p in paths:
                    dest = os.path.join(work_dir, os.path.basename(p))
                    shutil.move(p, dest)
                    all_work_paths.append(dest)
                    work_row_groups[row_num].append(dest)
        except Exception as e:
            self._restore_files(all_work_paths, input_dir)
            shutil.rmtree(work_dir, ignore_errors=True)
            return False, f"Failed to lock input files: {e}"

        start_time = time.time()

        # stitch each row into a horizontal strip
        self.get_logger().info(
            "=== Phase 1: Stitching individual rows ==="
        )

        row_strips = {}
        failed_rows = []

        for row_num in sorted(work_row_groups.keys()):
            row_paths = work_row_groups[row_num]

            if len(row_paths) > max_per_row:
                self.get_logger().warning(
                    f"Row {row_num}: Limiting to {max_per_row} images."
                )
                row_paths = row_paths[:max_per_row]

            self.get_logger().info(
                f"Row {row_num}: Stitching {len(row_paths)} images..."
            )

            images = self._load_and_resize(row_paths, max_width)

            if len(images) < 1:
                failed_rows.append((row_num, "No valid images loaded"))
                continue

            success, result, metadata = self._stitch_row_with_fallback(
                images, label=f"Row {row_num}"
            )

            del images  # free memory

            if success:
                result = self._crop_black_borders(result)
                row_strips[row_num] = result

                if metadata['gap_count'] > 0:
                    self.get_logger().warning(
                        f"Row {row_num}: Strip created with "
                        f"{metadata['gap_count']} gap(s), "
                        f"failed frames: {metadata['failed_frames']} "
                        f"({result.shape[1]}x{result.shape[0]}px)"
                    )
                else:
                    self.get_logger().info(
                        f"Row {row_num}: Strip created "
                        f"({result.shape[1]}x{result.shape[0]}px)"
                    )

                if (metadata['failed_frames']
                        and self.get_parameter('save_failed_frames').value):
                    self._save_failed_frames(
                        row_num,
                        metadata['failed_frames'],
                        work_row_groups[row_num]
                    )
            else:
                failed_rows.append((row_num, result))
                self.get_logger().error(f"Row {row_num}: {result}")

        if not row_strips:
            self._restore_files(all_work_paths, input_dir)
            shutil.rmtree(work_dir, ignore_errors=True)
            failures = "; ".join(
                f"Row {r}: {msg}" for r, msg in failed_rows
            )
            return False, f"All rows failed. {failures}"

        if save_intermediate:
            os.makedirs(output_dir, exist_ok=True)
            for row_num, strip in row_strips.items():
                strip_path = os.path.join(
                    output_dir,
                    f"row_strip_{row_num}_{timestamp}.jpg"
                )
                cv2.imwrite(strip_path, strip)
                self.get_logger().info(
                    f"Saved intermediate strip: {strip_path}"
                )

        # stitch row strips together 
        if len(row_strips) == 1:
            only_row = list(row_strips.keys())[0]
            mosaic = row_strips[only_row]
            self.get_logger().info(
                f"Only row {only_row} succeeded. Using as final mosaic."
            )
        else:
            self.get_logger().info(
                f"=== Phase 2: Stitching {len(row_strips)} "
                f"row strips vertically ==="
            )

            # order strips top to bottom by row number
            ordered_strips = [
                row_strips[k] for k in sorted(row_strips.keys())
            ]

            success, result = self._stitch_sequential(
                ordered_strips, label="Vertical merge"
            )

            del ordered_strips
            del row_strips

            if not success:
                self._restore_files(all_work_paths, input_dir)
                shutil.rmtree(work_dir, ignore_errors=True)
                return False, (
                    f"Phase 2 failed: {result}. "
                    "Row strips OK but vertical merge failed."
                )

            mosaic = result

        mosaic = self._crop_black_borders(mosaic)
        duration = time.time() - start_time

        # save the final mosaic
        os.makedirs(output_dir, exist_ok=True)
        save_path = os.path.join(output_dir, f"mosaic_{timestamp}.jpg")
        cv2.imwrite(save_path, mosaic)

        mosaic_h, mosaic_w = mosaic.shape[:2]
        del mosaic

        self.get_logger().info(f"Final mosaic saved: {save_path}")

        # move processed files to backup so they aren't re-stitched
        backup_dir = os.path.join(input_dir, "backup", timestamp)
        os.makedirs(backup_dir, exist_ok=True)

        for p in all_work_paths:
            try:
                shutil.move(
                    p, os.path.join(backup_dir, os.path.basename(p))
                )
            except Exception as e:
                self.get_logger().warning(f"Failed to archive {p}: {e}")

        shutil.rmtree(work_dir, ignore_errors=True)

        fail_note = ""
        if failed_rows:
            fail_note = (
                f" ({len(failed_rows)} row(s) failed: "
                f"{[r for r, _ in failed_rows]})"
            )

        return True, (
            f"Mosaic created in {duration:.2f}s "
            f"({mosaic_w}x{mosaic_h}px). "
            f"Saved to: {save_path}{fail_note}"
        )

    def _save_failed_frames(self, row_num, failed_indices, row_paths):
        # copy failed frames to a separate folder for manual inspection
        failed_dir = os.path.join(
            self.get_parameter('output_dir').value,
            'failed_frames',
            f'row_{row_num}'
        )
        os.makedirs(failed_dir, exist_ok=True)

        for idx in failed_indices:
            if idx < len(row_paths):
                src = row_paths[idx]
                dst = os.path.join(failed_dir, os.path.basename(src))
                shutil.copy2(src, dst)

        self.get_logger().info(
            f"Saved {len(failed_indices)} failed frame(s) to {failed_dir}"
        )

    def _restore_files(self, file_paths, dest_dir):
        # on failure, move files back to input dir so nothing is lost
        for p in file_paths:
            if os.path.exists(p):
                try:
                    shutil.move(
                        p, os.path.join(dest_dir, os.path.basename(p))
                    )
                except Exception as e:
                    self.get_logger().warning(
                        f"Failed to restore {p}: {e}"
                    )


def main(args=None):
    rclpy.init(args=args)
    node = StitchingNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
