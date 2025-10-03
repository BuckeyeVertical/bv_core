#!/usr/bin/env python3
# ----------------------------------------------------
# Image Stitcher Node (ROS2 + OpenCV in Python)
# ----------------------------------------------------
# - Subscribes to camera images (sensor_msgs/Image)
# - Subscribes to mission state (String)
# - Subscribes to GPS (NavSatFix)
# - Stores images with GPS coords when waypoints are reached
# - After N images, stitches them with GPS + ORB keypoints
# - Saves and displays the stitched result
# ----------------------------------------------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
import os
from mavros_msgs.msg import WaypointReached
from datetime import datetime
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from enum import Enum


# indexing into image gps tuple
import math
import numpy as np
import cv2

# Works with geometry_msgs.msg.Pose or PoseStamped
def _pose_xy_yaw(pose_msg):
    """Return (x, y, yaw_rad) from a geometry_msgs Pose or PoseStamped."""
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
    # yaw from quaternion (ROS uses xyzw; ENU assumed)
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
    return x, y, yaw

def mosaic_from_images_and_poses(
    frames,
    gsd_m_per_px=0.004,
    background_color=(0, 0, 0)
):
    """
    Build a 2D mosaic from a list of (image_bgr, pose_msg) tuples.

    Args:
        frames: list of (img_bgr, pose) where img_bgr is HxWx3 uint8 (OpenCV BGR)
                and pose is geometry_msgs.msg.Pose or PoseStamped in a common world frame (meters).
        gsd_m_per_px: ground sampling distance (meters per pixel) for the images.
                      We use the same value for the canvas, so scale=1 by default.
        background_color: canvas background (B,G,R).
    Returns:
        canvas_bgr: the stitched mosaic as a numpy array (uint8 BGR).
        origin_m:  (min_x_m, min_y_m) world coords of canvas pixel (0,0)
        px_per_m:  pixels per meter used for the canvas
    """
    if not frames:
        raise ValueError("frames is empty")

    px_per_m = 1.0 / float(gsd_m_per_px)

    # --- 1) Predict canvas bounds in world meters by transforming each image's corners ---
    world_corners = []  # list of Nx2 arrays of transformed corners for each image

    centers_xy_yaw = []
    sizes_px = []
    for img, pose in frames:
        h, w = img.shape[:2]
        x_m, y_m, yaw = _pose_xy_yaw(pose)
        # image size in meters on ground (assuming nadir camera & constant GSD)
        w_m = w * gsd_m_per_px
        h_m = h * gsd_m_per_px

        # image-local corners (meters), centered at (0,0), +y up in world math
        # We'll treat image's +y (down in pixels) as negative in math space, then rotate by yaw.
        local = np.array([
            [-w_m/2, -h_m/2],
            [ w_m/2, -h_m/2],
            [ w_m/2,  h_m/2],
            [-w_m/2,  h_m/2],
        ], dtype=np.float32)

        # 2D rotation by yaw (world frame Z-up): image-to-world
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s],
                      [s,  c]], dtype=np.float32)

        corners_world = (local @ R.T) + np.array([x_m, y_m], dtype=np.float32)
        world_corners.append(corners_world)

        centers_xy_yaw.append((x_m, y_m, yaw))
        sizes_px.append((w, h))

    all_pts = np.vstack(world_corners)  # (4*N, 2)
    min_x_m = float(all_pts[:, 0].min())
    max_x_m = float(all_pts[:, 0].max())
    min_y_m = float(all_pts[:, 1].min())
    max_y_m = float(all_pts[:, 1].max())

    # Add a small border (in meters)
    pad_m = 2.0 * gsd_m_per_px * 10  # ~10px padding
    min_x_m -= pad_m
    min_y_m -= pad_m
    max_x_m += pad_m
    max_y_m += pad_m

    # --- 2) Canvas size in pixels ---
    width_px  = int(math.ceil((max_x_m - min_x_m) * px_per_m))
    height_px = int(math.ceil((max_y_m - min_y_m) * px_per_m))
    if width_px <= 0 or height_px <= 0:
        raise RuntimeError("Computed non-positive canvas size")

    canvas = np.full((height_px, width_px, 3), background_color, dtype=np.uint8)

    # --- 3) Paste each image onto canvas using a 2D SE(2) (rotate about center, then translate) ---
    for (img, _pose), (x_m, y_m, yaw), (w, h) in zip(frames, centers_xy_yaw, sizes_px):
        # Where should the *center* land on the canvas (pixels)?
        cx = (x_m - min_x_m) * px_per_m
        cy = (y_m - min_y_m) * px_per_m

        # Rotate image by -yaw so that world axes align (east→right).
        angle_deg = -math.degrees(yaw)
        M = cv2.getRotationMatrix2D(center=(w/2, h/2), angle=angle_deg, scale=1.0)

        # Shift so that the rotated image center goes to (cx, cy) on the canvas
        M[0, 2] += (cx - w/2)
        M[1, 2] += (cy - h/2)

        # Warp directly into the big canvas; simple overwrite compositing
        cv2.warpAffine(
            img, M,
            dsize=(canvas.shape[1], canvas.shape[0]),
            dst=canvas,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_TRANSPARENT  # leave background as-is where img is out of bounds
        )

        # If you prefer alpha blending instead of overwrite:
        # tmp = np.zeros_like(canvas)
        # cv2.warpAffine(img, M, (canvas.shape[1], canvas.shape[0]), tmp, flags=cv2.INTER_LINEAR,
        #                borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
        # mask = (tmp.sum(axis=2) > 0).astype(np.uint8)[..., None]
        # canvas = (tmp * mask + canvas * (1 - mask)).astype(np.uint8)

    origin_m = (min_x_m, min_y_m)  # world meters corresponding to canvas (0,0)
    return canvas, origin_m, px_per_m


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
        self.declare_parameter('stitch_interval_sec', 60.0)
        os.makedirs("images", exist_ok=True)

        image_topic = self.get_parameter(
            'image_topic').get_parameter_value().string_value
        self.output_path = self.get_parameter(
            'output_path').get_parameter_value().string_value
        self.crop = self.get_parameter('crop').get_parameter_value().bool_value
        self.preprocessing = self.get_parameter(
            'preprocessing').get_parameter_value().bool_value
        self.stitch_interval_sec = self.get_parameter(
            'stitch_interval_sec').get_parameter_value().double_value

        # Subscriptions

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.mission_state_sub = self.create_subscription(
            String, '/mission_state', self.state_callback, 10)
        self.reached_sub = self.create_subscription(
            WaypointReached, '/mavros/mission/reached', self.reached_callback, 10)

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_callback, qos_profile=pose_qos)

        gps_sub_QoS = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, gps_sub_QoS)

        self.stitch_timer = self.create_timer(
            self.stitch_interval_sec, self.timer_callback)

        # Helpers
        self.bridge = CvBridge()
        self.received_images = []  # will store (image, pose)

        self.get_logger().info(
            f"Subscribed to {image_topic}; stitch every {self.stitch_interval_sec}s")

    # -------------------------
    # Callbacks
    # -------------------------
    def rosimg_to_ndarray(self, msg: Image) -> np.ndarray:
        """Convert any sensor_msgs/Image to an H×W×C NumPy array."""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return img

    def crop_image(self, img):
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        crop_w, crop_h = h, h

        cropped = img[cy-crop_h//2: cy+crop_h//2,
                      cx-crop_w//2: cx+crop_w//2]
        return cropped

    def deblur_image(self, img):
        pass

    def state_callback(self, msg: String):
        # self.get_logger().info(f"Received State: state={msg.data}")
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
        pass

    def timer_callback(self):
        self.get_logger().info(f'callback enter')
        canvas, origin_m, px_per_m = mosaic_from_images_and_poses(self.received_images)
        path = os.path.join(self.output_path, f"stitch -- stamp -- {datetime.now()}.jpg")
        cv2.imwrite(path, canvas)
        self.get_logger().info(f'saved at: {path}')

    def image_callback(self, msg: Image):
        try:
            frame = self.rosimg_to_ndarray(msg)
            cropped_frame = self.crop_image(frame)
            self.latest_frame = cropped_frame
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw image: {e}")

    def pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"(msg.pose)")
        pose = msg.pose
        self.received_images.append((self.latest_frame, pose))


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
