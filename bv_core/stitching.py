#!/usr/bin/env python3

# ----------------------------------------------------
# Image Stitcher Node (ROS2 + OpenCV in Python)
# ----------------------------------------------------
# - Subscribes to a ROS2 raw image topic
# - Buffers incoming frames (decoded to H×W×C NumPy arrays)
# - Every N seconds, stitches them
# - Saves and displays the result (no publishing)
# ----------------------------------------------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
import os
# for stitching
from mavros_msgs.msg import Waypoint, State as MavState, WaypointReached, CommandCode

from datetime import datetime

MAX_PICS = 2

class ImageStitcherNode(Node):
    def __init__(self):
        super().__init__('image_stitcher_node')

        # Parameters

        #default to not transitioning
        self.transition_in_progress = False

        #default to 0th waypoint
        self.waypoint_current = 0

        #default to no previous waypoint
        self.waypoint_prev = -1

        #default say no frame stored
        self.latest_frame = None

        #default to lap state at the beginning
        self.state = "lap"

        #default reached mode
        self.reached = False

        #counter for how many total pictures to take
        self.pic_counter = 0

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_path', '/home/bvorinnano/bv_ws/stitched_image')
        self.declare_parameter('crop', True)
        self.declare_parameter('preprocessing', False)
        self.declare_parameter('stitch_interval_sec', 5.0)

        # Fetch parameters
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.crop = self.get_parameter('crop').get_parameter_value().bool_value
        self.preprocessing = self.get_parameter('preprocessing').get_parameter_value().bool_value
        self.stitch_interval_sec = self.get_parameter('stitch_interval_sec').get_parameter_value().double_value

        # Subscription & Timer
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10 #what is this 10
        )

        # Subscribe to the state the drone is in
        self.mission_state_sub = self.create_subscription(
            String,
            '/mission_state',
            self.state_callback,
            10
        )

        # Subscribe to when drone reaches position
        self.reached_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.reached_callback,
            10
        )

        self.stitch_timer = self.create_timer(self.stitch_interval_sec, self.timer_callback)

        # Helpers & buffer
        self.bridge = CvBridge()
        self.received_images = []
        self.get_logger().info(f"Subscribed to {image_topic}`; stitch every {self.stitch_interval_sec}s")

    def rosimg_to_ndarray(self, msg: Image) -> np.ndarray:
        """Convert any sensor_msgs/Image to an H×W×C uint8 NumPy array."""
        enc = msg.encoding.lower()
        # Choose dtype by bit-depth
        if enc.endswith('16'):
            dtype = np.uint16
        else:
            dtype = np.uint8

        # View the raw buffer
        flat = np.frombuffer(msg.data, dtype=dtype)

        # msg.step = total bytes per row; convert to elements per row
        elems_per_row = msg.step // dtype().itemsize
        mat = flat.reshape((msg.height, elems_per_row))

        # Handle multi-channel layouts
        if enc in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            nch = 3 if enc.endswith('8') else 4
            # keep only real pixel data, discard padding
            mat = mat[:, : msg.width * nch ]
            mat = mat.reshape((msg.height, msg.width, nch))
            # drop alpha if present
            if nch == 4:
                mat = mat[:, :, :3]
        else:
            # mono8 or mono16: single-channel image already
            # mat is H×W
            pass

        # If needed, convert colors for OpenCV
        if enc == 'rgb8':
            mat = cv2.cvtColor(mat, cv2.COLOR_RGB2BGR)

        # If 16-bit, scale down to 8-bit
        if mat.dtype == np.uint16:
            mat = cv2.convertScaleAbs(mat, alpha=(255.0 / 65535.0))

        return mat

    def image_callback(self, msg: Image):
        #self.get_logger().warn(f"Image_callback is running")
        try:
            frame = self.rosimg_to_ndarray(msg)
            self.latest_frame = frame
            #self.get_logger().info(f"got pic")
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw image: {e}")

    def state_callback(self, msg: String):
        try:
            self.get_logger().info(f"Received State: state={msg.data}")
            self.state = msg.data
        except Exception as e:
            self.get_logger().error(f"Failed to determine state")

    def reached_callback(self, msg: WaypointReached):
        if self.transition_in_progress:
            return
        idx = msg.wp_seq
        self.get_logger().info(f'Reached waypoint {idx} (state={self.state})')

        #setting waypoint number
        self.waypoint_current = idx


        # if idx == self.expected_final_wp:
        #     self.handle_mission_completion()

        # store an image if mission state is 
        #ask shankar if self.state can equal WaypointReached
        if (self.state == "stitching"): #or self.state == "lap" or self.state == "scan"):
            self.reached = True
            self.get_logger().info(f'Reached waypoint in {self.state}')
        else: 
            self.reached = False
            self.get_logger().info(f'Reached waypoint, NOT in stitching')

    def timer_callback(self):
        
        if (self.latest_frame is not None and self.reached == True and self.pic_counter < MAX_PICS and self.waypoint_current != self.waypoint_prev):

                self.get_logger().info(f"Took picture number: {self.pic_counter}")
                self.waypoint_prev = self.waypoint_current
                self.received_images.append(self.latest_frame)
                self.get_logger().info(f"Received frame: shape={self.latest_frame.shape} dtype={self.latest_frame.dtype} (buffer={len(self.received_images)})")
                
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                out_file = os.path.join(os.path.dirname(self.output_path), f"pic_{self.waypoint_current}.jpg")
                cv2.imwrite(out_file, self.latest_frame)
                self.get_logger().info(f"Saved: {out_file}")
                self.pic_counter = self.pic_counter + 1

        elif self.pic_counter >= MAX_PICS:
                    self.get_logger().info(f"Got all pictures: {self.pic_counter}, starting stitching")
                    imgs = list(self.received_images)
                    self.received_images.clear()
                    self.get_logger().info(f"Stitching {len(imgs)} frames…")

                    status, pano = self.stitch_images(imgs)
                    if status == cv2.Stitcher_OK:
                        self.get_logger().info("Stitch successful")
                        if self.crop:
                            pano = self.crop_image(pano)

                        # save with timestamp
                        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                        out_file = os.path.join(os.path.dirname(self.output_path), f"stitched_{ts}.jpg")
                        cv2.imwrite(out_file, pano)
                        self.get_logger().info(f"Saved: {out_file}")

                        # display
                        cv2.imshow("Stitched", pano)
                        cv2.waitKey(1)
                    else:
                        self.get_logger().error(f"Stitch failed (code={status})")
                        # optional: handle specific errors here…
        else:
                return
        # if len(self.received_images) < 2:
        #     self.get_logger().warn("Not enough images to stitch yet.")
        #     return

        # imgs = list(self.received_images)
        # self.received_images.clear()
        # self.get_logger().info(f"Stitching {len(imgs)} frames…")

        # status, pano = self.stitch_images(imgs)
        # if status == cv2.Stitcher_OK:
        #     self.get_logger().info("Stitch successful")
        #     if self.crop:
        #         pano = self.crop_image(pano)

        #     # save with timestamp
        #     ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        #     out_file = os.path.join(os.path.dirname(self.output_path), f"stitched_{ts}.jpg")
        #     cv2.imwrite(out_file, pano)
        #     self.get_logger().info(f"Saved: {out_file}")

        #     # display
        #     cv2.imshow("Stitched", pano)
        #     cv2.waitKey(1)
        # else:
        #     self.get_logger().error(f"Stitch failed (code={status})")
        #     # optional: handle specific errors here…

    def resize_images(self, images, widthThreshold=1500):
        resized = []
        for img in images:
            h, w = img.shape[:2]
            if w > widthThreshold:
                r = widthThreshold / w
                resized.append(cv2.resize(img, (widthThreshold, int(h*r))))
            else:
                resized.append(img)
        return resized

    def preprocess_images(self, images):
        if not self.preprocessing:
            return images
        self.get_logger().info("Preprocessing (CLAHE)…")
        out = []
        for img in images:
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            l = clahe.apply(l)
            lab = cv2.merge((l,a,b))
            out.append(cv2.cvtColor(lab, cv2.COLOR_LAB2BGR))
        return out

    def stitch_images(self, images):
        images = self.resize_images(images)
        images = self.preprocess_images(images)
        stitcher = cv2.Stitcher.create(cv2.Stitcher_SCANS)
        return stitcher.stitch(images)

    def crop_image(self, stitched):
        self.get_logger().info("Cropping black borders…")
        img = cv2.copyMakeBorder(stitched,10,10,10,10,cv2.BORDER_CONSTANT,value=(0,0,0))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
        cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            img = img[y:y+h, x:x+w]
        return img

def main(args=None):
    rclpy.init(args=args)
    node = ImageStitcherNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
