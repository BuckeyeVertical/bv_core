#!/usr/bin/env python3
# ----------------------------------------------------
# Image Stitcher Node (ROS2 + OpenCV in Python)
# ----------------------------------------------------
# - Subscribes to camera images (sensor_msgs/Image)
# - Subscribes to mission state (String)
# - Subscribes to GPS (NavSatFix)
# - Stores images with poses
# - Mosaics using pose-based placement + local overlap refinement
#   (ECC rigid tweak with ORB+RANSAC fallback)
# - Saves the stitched result periodically
# ----------------------------------------------------

import os
import time
import math
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, NavSatFix, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import WaypointReached
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


# =====================
# Linear algebra helpers (affine composition)
# =====================

def _to33(M):
    M = np.asarray(M, dtype=np.float32)
    if M.shape == (2, 3):
        return np.vstack([M, [0., 0., 1.]]).astype(np.float32)
    elif M.shape == (3, 3):
        return M.astype(np.float32)
    else:
        raise ValueError(f"Expected 2x3 or 3x3, got {M.shape}")

def _to23(M):
    M = np.asarray(M, dtype=np.float32)
    if M.shape == (3, 3):
        return M[:2, :]
    elif M.shape == (2, 3):
        return M
    else:
        raise ValueError(f"Expected 2x3 or 3x3, got {M.shape}")

def _compose(Ma, Mb):
    """
    Return Ma ∘ Mb (apply Mb first, then Ma), as a 2x3 affine.
    Works if inputs are 2x3 or 3x3.
    """
    A = _to33(Ma)
    B = _to33(Mb)
    return _to23(A @ B)


# =====================
# Pose helper
# =====================

def _pose_xy_yaw(pose_msg):
    """Return (x, y, yaw_rad) from a geometry_msgs Pose or PoseStamped (ENU)."""
    if hasattr(pose_msg, "pose"):  # PoseStamped
        p = pose_msg.pose
    else:  # Pose
        p = pose_msg
    x = float(p.position.x)
    y = float(p.position.y)
    qx = float(p.orientation.x)
    qy = float(p.orientation.y)
    qz = float(p.orientation.z)
    qw = float(p.orientation.w)
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
    return x, y, yaw


# =====================
# Mosaic algorithm (pose-initialized + local overlap refinement)
# =====================

def mosaic_from_images_and_poses(
    frames,
    background_color=(0, 0, 0),
    base_alt_m=9.0,
    base_gsd_m_per_px=0.004,   # meters per pixel at base_alt_m
    neighbor_radius_m=12.0,    # consider overlaps within ~footprint size
    ecc_passes=2,              # relaxation passes over neighbor graph
    do_feather=True
):
    """
    Pose-initialized, overlap-refined mosaic (no intrinsics needed).

    Args:
        frames: list[(img_bgr, pose_msg)]
        background_color: BGR canvas color
        base_alt_m: assumed average altitude (m) (informational; we fix scale via base_gsd)
        base_gsd_m_per_px: meters per pixel at base_alt_m
        neighbor_radius_m: neighbor distance for overlap checks
        ecc_passes: number of refinement passes
        do_feather: enable simple linear feather on composites
    Returns:
        canvas_bgr, origin_m(x_min, y_min), px_per_m
    """
    if not frames:
        raise ValueError("frames is empty")

    # ---- 0) consistent (global) scale ----
    gsd = float(base_gsd_m_per_px)
    PX_PER_M = 1.0 / gsd

    # ---- 1) Predict canvas bounds (using constant GSD) ----
    centers = []   # [(x,y,yaw)]
    sizes   = []   # [(w,h)]
    world_corners_per_img = []

    for img, pose in frames:
        h, w = img.shape[:2]
        x_m, y_m, yaw = _pose_xy_yaw(pose)
        w_m, h_m = w * gsd, h * gsd

        local = np.array([
            [-w_m/2, -h_m/2],
            [ w_m/2, -h_m/2],
            [ w_m/2,  h_m/2],
            [-w_m/2,  h_m/2],
        ], dtype=np.float32)

        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s],
                      [s,  c]], dtype=np.float32)
        corners_world = (local @ R.T) + np.array([x_m, y_m], dtype=np.float32)

        centers.append((x_m, y_m, yaw))
        sizes.append((w, h))
        world_corners_per_img.append(corners_world)

    all_pts = np.vstack(world_corners_per_img)
    min_x_m = float(all_pts[:, 0].min())
    max_x_m = float(all_pts[:, 0].max())
    min_y_m = float(all_pts[:, 1].min())
    max_y_m = float(all_pts[:, 1].max())

    # small ~10 px padding
    pad_m = 10.0 * gsd
    min_x_m -= pad_m
    min_y_m -= pad_m
    max_x_m += pad_m
    max_y_m += pad_m

    width_px  = int(math.ceil((max_x_m - min_x_m) * PX_PER_M))
    height_px = int(math.ceil((max_y_m - min_y_m) * PX_PER_M))
    if width_px <= 0 or height_px <= 0:
        raise RuntimeError("Computed non-positive canvas size")
    origin_m = (min_x_m, min_y_m)

    # ---- 2) Initial image->canvas transforms (2x3) ----
    A = []      # list of 2x3 affines (image -> canvas)
    polys = []  # transformed image quads in canvas px

    for (img, _pose), (x_m, y_m, yaw), (w, h) in zip(frames, centers, sizes):
        # center in canvas px
        cx = (x_m - min_x_m) * PX_PER_M
        cy = (y_m - min_y_m) * PX_PER_M

        angle_deg = -math.degrees(yaw)  # rotate so world-x goes right
        M = cv2.getRotationMatrix2D(center=(w/2, h/2), angle=angle_deg, scale=1.0)
        M[0, 2] += (cx - w/2)
        M[1, 2] += (cy - h/2)
        A.append(M.astype(np.float32))

        # polygon for overlap checks
        corners_px = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32).reshape(-1, 1, 2)
        poly = cv2.transform(corners_px, M).reshape(-1, 2)
        polys.append(poly)

    # ---- 3) Neighbor graph by XY distance ----
    N = len(frames)
    nbrs = [[] for _ in range(N)]
    xy = np.array([[x, y] for (x, y, _yaw) in centers], dtype=np.float32)
    for i in range(N):
        for j in range(i+1, N):
            d = float(np.linalg.norm(xy[i] - xy[j]))
            if d <= neighbor_radius_m:
                nbrs[i].append(j)
                nbrs[j].append(i)

    # ---- 4) ECC overlap refinement (+ ORB fallback) ----
    def _roi_from_polys(pi, pj, W, H):
        xi0, yi0, wi, hi = cv2.boundingRect(pi.astype(np.float32))
        xj0, yj0, wj, hj = cv2.boundingRect(pj.astype(np.float32))
        x0 = max(0, max(xi0, xj0))
        y0 = max(0, max(yi0, yj0))
        x1 = min(W, min(xi0+wi, xj0+wj))
        y1 = min(H, min(yi0+hi, yj0+hj))
        if x1 <= x0 or y1 <= y0:
            return None
        return (x0, y0, x1, y1)

    def _update_poly(j):
        wj, hj = sizes[j]
        corners_px = np.array([[0, 0], [wj, 0], [wj, hj], [0, hj]], dtype=np.float32).reshape(-1, 1, 2)
        polys[j] = cv2.transform(corners_px, A[j]).reshape(-1, 2)

    def _orb_ransac_fallback(i, j):
        img_i = frames[i][0]
        img_j = frames[j][0]

        gray_i = cv2.cvtColor(img_i, cv2.COLOR_BGR2GRAY)
        gray_j = cv2.cvtColor(img_j, cv2.COLOR_BGR2GRAY)
        orb = cv2.ORB_create(nfeatures=2000)
        ki, di = orb.detectAndCompute(gray_i, None)
        kj, dj = orb.detectAndCompute(gray_j, None)
        if di is None or dj is None or len(ki) < 20 or len(kj) < 20:
            return False

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        raw = bf.knnMatch(dj, di, k=2)  # j -> i
        good = []
        for pair in raw:
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < 0.75 * n.distance:
                good.append(m)
        if len(good) < 20:
            return False

        pts_j = np.float32([kj[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        pts_i = np.float32([ki[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        # map into canvas with current A
        pts_j_c = cv2.transform(pts_j, A[j])
        pts_i_c = cv2.transform(pts_i, A[i])

        M, inliers = cv2.estimateAffinePartial2D(
            pts_j_c, pts_i_c,
            method=cv2.RANSAC,
            ransacReprojThreshold=3.0,
            confidence=0.995
        )
        if M is None:
            return False

        A[j] = _compose(M, A[j])
        _update_poly(j)
        return True

    def _ecc_refine_pair(i, j):
        wj, hj = sizes[j]
        roi = _roi_from_polys(polys[i], polys[j], width_px, height_px)
        if roi is None:
            return False
        x0, y0, x1, y1 = roi
        rw, rh = x1 - x0, y1 - y0
        if rw < 40 or rh < 40:
            return False

        img_i = frames[i][0]
        img_j = frames[j][0]
        Ti = A[i].copy()
        Tj = A[j].copy()

        # ROI shift transforms
        To = np.array([[1, 0, -x0],
                       [0, 1, -y0]], dtype=np.float32)
        Tpos = np.array([[1, 0,  x0],
                         [0, 1,  y0]], dtype=np.float32)

        # Warp both into ROI
        Ii = cv2.warpAffine(img_i, _compose(To, Ti), (rw, rh),
                            flags=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        Ij = cv2.warpAffine(img_j, _compose(To, Tj), (rw, rh),
                            flags=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=0)

        gi = cv2.cvtColor(Ii, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
        gj = cv2.cvtColor(Ij, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0

        # quick content check
        if gi.var() < 1e-4 or gj.var() < 1e-4:
            return _orb_ransac_fallback(i, j)

        warp_mode = cv2.MOTION_EUCLIDEAN  # rotation + translation
        W = np.eye(2, 3, dtype=np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 1e-4)
        try:
            cc, W = cv2.findTransformECC(gi, gj, W, warp_mode, criteria, None, 1)
            # Lift ROI-local W back to canvas: C = Tpos ∘ W ∘ To
            C = _compose(_compose(Tpos, W), To)
            A[j] = _compose(C, A[j])
            _update_poly(j)
            return True
        except cv2.error:
            # ECC failed; try ORB fallback
            return _orb_ransac_fallback(i, j)

    # Relaxation passes over neighbor edges
    for _ in range(max(1, ecc_passes)):
        improved = False
        for i in range(N):
            for j in nbrs[i]:
                if j <= i:
                    continue
                ok = _ecc_refine_pair(i, j)
                improved = improved or ok
        if not improved:
            break

    # ---- 5) Render final canvas ----
    canvas = np.full((height_px, width_px, 3), background_color, dtype=np.uint8)

    if do_feather:
        def feather_mask(w, h):
            m = np.zeros((h, w), np.float32)
            m[5:-5, 5:-5] = 1.0
            m = cv2.GaussianBlur(m, (0, 0), 5.0)
            return m[..., None]
    else:
        def feather_mask(w, h):
            return np.ones((h, w, 1), np.float32)

    acc = canvas.astype(np.float32)
    weight = np.zeros((height_px, width_px, 1), np.float32)

    for k, ((img, _pose), M) in enumerate(zip(frames, A)):
        w, h = img.shape[1], img.shape[0]
        m = feather_mask(w, h)
        imgf = img.astype(np.float32) * m

        tmp = np.zeros_like(acc)
        wtmp = np.zeros_like(weight)
        cv2.warpAffine(imgf, M, (width_px, height_px), tmp,
                       flags=cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_TRANSPARENT)
        cv2.warpAffine(m,   M, (width_px, height_px), wtmp,
                       flags=cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_TRANSPARENT)

        acc += tmp
        weight += wtmp

    weight[weight == 0] = 1.0
    canvas = (acc / weight).astype(np.uint8)

    return canvas, origin_m, PX_PER_M


# =====================
# ROS2 Node
# =====================

class ImageStitcherNode(Node):
    def __init__(self):
        super().__init__('image_stitcher_node')

        # State variables
        self.transition_in_progress = False
        self.waypoint_current = 0
        self.waypoint_prev = -1
        self.latest_frame = None
        self.latest_gps = None
        self.state = "lap"
        self.reached = False
        self.pic_counter = 0

        # Parameters
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_path', 'images')
        self.declare_parameter('crop', True)
        self.declare_parameter('preprocessing', False)
        self.declare_parameter('stitch_interval_sec', 30.0)
        self.declare_parameter('minimum_stitch_distance', 8.0)
        os.makedirs("images", exist_ok=True)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.crop = self.get_parameter('crop').get_parameter_value().bool_value
        self.preprocessing = self.get_parameter('preprocessing').get_parameter_value().bool_value
        self.stitch_interval_sec = self.get_parameter('stitch_interval_sec').get_parameter_value().double_value
        self.minimum_stitch_distance = self.get_parameter('minimum_stitch_distance').get_parameter_value().double_value

        # Subscriptions
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.mission_state_sub = self.create_subscription(String, '/mission_state', self.state_callback, 10)
        self.reached_sub = self.create_subscription(WaypointReached, '/mavros/mission/reached', self.reached_callback, 10)

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_callback, qos_profile=pose_qos)

        gps_sub_QoS = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, gps_sub_QoS)

        self.stitch_timer = self.create_timer(self.stitch_interval_sec, self.timer_callback)

        # Helpers
        self.bridge = CvBridge()
        self.received_images = []  # stores (image, pose)

        self.get_logger().info(f"Subscribed to {image_topic}; stitch every {self.stitch_interval_sec}s")

    # -------------------------
    # Callbacks
    # -------------------------
    def rosimg_to_ndarray(self, msg: Image) -> np.ndarray:
        """Convert sensor_msgs/Image to H×W×C NumPy array (BGR8)."""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return img

    def crop_image(self, img):
        """Square center crop (optional)."""
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        crop_w, crop_h = h, h
        x0 = max(0, cx - crop_w // 2)
        y0 = max(0, cy - crop_h // 2)
        x1 = min(w, x0 + crop_w)
        y1 = min(h, y0 + crop_h)
        return img[y0:y1, x0:x1]

    def deblur_image(self, img):
        # placeholder if you want to add a quick deblur/USM pass later
        return img

    def state_callback(self, msg: String):
        self.state = msg.data

    def reached_callback(self, msg: WaypointReached):
        if self.transition_in_progress:
            return
        idx = msg.wp_seq
        self.get_logger().info(f'Reached waypoint {idx} (state={self.state})')
        self.waypoint_current = idx
        if self.state == "stitching":
            self.reached = True
            self.get_logger().info(f'Reached waypoint in {self.state}')
        else:
            self.reached = False
            self.get_logger().info(f'Reached waypoint, NOT in stitching')

    def gps_callback(self, msg: NavSatFix):
        # not used by the stitcher right now, but available if needed
        self.latest_gps = msg

    def timer_callback(self):
        self.get_logger().info('callback enter')
        try:
            if len(self.received_images) < 2:
                self.get_logger().info("Need at least 2 frames; skipping stitch.")
                return

            canvas, origin_m, px_per_m = mosaic_from_images_and_poses(
                self.received_images,
                base_alt_m=9.0,
                base_gsd_m_per_px=0.004,
                neighbor_radius_m=12.0,
                ecc_passes=2,
                do_feather=True
            )

            os.makedirs(self.output_path, exist_ok=True)
            path = os.path.join(self.output_path, f"stitch -- {datetime.now():%Y%m%d_%H%M%S}.jpg")
            cv2.imwrite(path, canvas)
            self.get_logger().info(f"Saved mosaic: {path}  origin_m={origin_m}  px_per_m={px_per_m:.2f}")
        except Exception as e:
            self.get_logger().error(f"Stitch failed: {e}")

    def image_callback(self, msg: Image):
        try:
            frame = self.rosimg_to_ndarray(msg)
            if self.crop:
                frame = self.crop_image(frame)
            if self.preprocessing:
                frame = self.deblur_image(frame)
            self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw image: {e}")

    def pose_callback(self, msg: PoseStamped):
        # Only append when we have an image to pair
        if self.latest_frame is None:
            return

        pose = msg.pose
        if not self.received_images:
            self.received_images.append((self.latest_frame, pose))
            self.get_logger().info("Appended first image/pose (seed).")
            # Optional: avoid re-using the exact same frame for multiple poses
            # self.latest_frame = None
            return

        # Compare to most recently appended pose (XY distance)
        last_pose = self.received_images[-1][1]
        dx = pose.position.x - last_pose.position.x
        dy = pose.position.y - last_pose.position.y
        dist_xy = math.hypot(dx, dy)

        if dist_xy >= self.minimum_stitch_distance:
            self.received_images.append((self.latest_frame, pose))
            self.get_logger().info(f"Appended (Δ= {dist_xy:.2f} m ≥ {self.minimum_stitch_distance} m).")
            # Optional: self.latest_frame = None
        else:
            self.get_logger().info(f"Skipped (Δ= {dist_xy:.2f} m < {self.minimum_stitch_distance} m).")


# =====================

def main(args=None):
    rclpy.init(args=args)
    node = ImageStitcherNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
