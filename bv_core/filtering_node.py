#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, Float64, Int8
from .localizer import Localizer
from rclpy.executors import MultiThreadedExecutor
from bv_msgs.msg import ObjectDetections
from geometry_msgs.msg import PoseStamped
import numpy as np
from rfdetr.util.coco_classes import COCO_CLASSES

# Create 0-indexed list from COCO_CLASSES (detector outputs 0-79, not original COCO IDs)
COCO_CLASS_NAMES = [COCO_CLASSES[k] for k in sorted(COCO_CLASSES.keys())]

# Only two target classes for the competition: one mannequin, one tent
MANNEQUIN_CLASS_ID = COCO_CLASS_NAMES.index('person')
TENT_CLASS_ID = COCO_CLASS_NAMES.index('umbrella')  # closest COCO class to tent
TARGET_CLASS_IDS = {MANNEQUIN_CLASS_ID, TENT_CLASS_ID}

from sensor_msgs.msg import NavSatFix

from bv_msgs.msg import ObjectLocations         # your custom msg
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from collections import deque
from rclpy.time import Time
import math

class FilteringNode(Node):
    def __init__(self):
        super().__init__('filtering_node')

        # === subscriptions ===
        # Match vision_node's qos_best_effort exactly
        best_effort_qos = QoSProfile(depth=10)
        best_effort_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # Match vision_node's qos_reliable exactly for /obj_dets
        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE

        self.object_det_sub = self.create_subscription(
            ObjectDetections,
            '/obj_dets',
            self.handle_detections,
            reliable_qos
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.handle_gps,
            best_effort_qos
        )
        
        self.rel_alt_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.handle_rel_alt,
            best_effort_qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.handle_local_pose,
            best_effort_qos
        )

        self.mission_state_sub = self.create_subscription(
            String,
            '/mission_state',
            self.mission_state_callback,
            best_effort_qos
        )
        self.deployed_location_sub = self.create_subscription(
            ObjectLocations,
            '/deployed_object_locations',
            self.deployed_location_callback,
            reliable_qos
        )

        self.gps_buffer  = deque(maxlen=200)
        self.pose_buffer = deque(maxlen=200)

        # === internal state ===
        self.prev_state = None
        self.state = None
        
        # === target tracking ===
        # One entry per competition target: mannequin and tent
        self.targets = {
            MANNEQUIN_CLASS_ID: {"state": "undetected", "lat": None, "lon": None},
            TENT_CLASS_ID:      {"state": "undetected", "lat": None, "lon": None},
        }
        
        # === 3-frame confirmation tracking ===
        self.frame_history = []  # list of recent detection sets [(lat, lon, class_id), ...]
        
        # === publisher for confirmed detections ===
        # Use same reliable QoS pattern for /global_obj_dets
        global_dets_qos = QoSProfile(depth=10)
        global_dets_qos.reliability = ReliabilityPolicy.RELIABLE
        self.confirmed_pub = self.create_publisher(Int8, '/global_obj_dets', global_dets_qos)

        filtering_yaml = os.path.join(
            get_package_share_directory('bv_core'),
            'config',
            'filtering_params.yaml'
        )

        with open(filtering_yaml, 'r') as f:
            cfg = yaml.safe_load(f)

        c_mat_list = cfg.get('c_matrix', [1.0]*9)
        dist_coeff_list = cfg.get('dist_coefficients', [0.0]*5)

        camera_matrix = np.array(c_mat_list, dtype=np.float64).reshape((3, 3))
        dist_coeffs = np.array(dist_coeff_list, dtype=np.float64)

        self.localizer = Localizer(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs
        )

        self.last_rel_alt = None

    def handle_gps(self, msg: NavSatFix):
        self.gps_buffer.append(msg)

    def handle_rel_alt(self, msg: Float64):
        self.last_rel_alt = msg

    def handle_local_pose(self, msg: PoseStamped):
        self.pose_buffer.append(msg)

    def handle_detections(self, msg: ObjectDetections):
        # Debug: Log incoming detections
        num_dets = len(msg.dets) if msg.dets else 0
        # self.get_logger().info(f"[DEBUG] Received {num_dets} detections from /obj_dets")
        
        if len(self.pose_buffer) == 0 or len(self.gps_buffer) == 0 or self.last_rel_alt == None:
            return
        
        # convert detection stamp into a float or ns
        t_det = Time.from_msg(msg.header.stamp).nanoseconds

        # find the GPS msg whose stamp is closest
        best_gps = min(
            self.gps_buffer,
            key=lambda m: abs(Time.from_msg(m.header.stamp).nanoseconds - t_det),
            default=None
        )
        # same for pose
        best_pose = min(
            self.pose_buffer,
            key=lambda m: abs(Time.from_msg(m.header.stamp).nanoseconds - t_det),
            default=None
        )

        if not best_gps or not best_pose:
            return
        
        drone_pose = (
            best_gps.latitude,
            best_gps.longitude,
            self.last_rel_alt.data
        )

        drone_orientation = (
            best_pose.pose.orientation.x,
            best_pose.pose.orientation.y,
            best_pose.pose.orientation.z,
            best_pose.pose.orientation.w
        )
        
        # msg.dets is a sequence of Vectors: x, y, class_id
        pixel_centers = [
            (det.x, det.y, int(det.z))
            for det in msg.dets
        ]

        # convert to global lat/lon
        detections_global = self.localizer.get_lat_lon(
            pixel_centers,
            drone_global_pos=drone_pose,
            drone_orientation=drone_orientation
            # drone_orientation=(1.0, 0.0, 0.0, 0.0)
        )

        # filter to target classes that haven't been confirmed or deployed yet
        filtered_detections = [
            (lat, lon, cls)
            for lat, lon, cls in detections_global
            if cls in self.targets and self.targets[cls]["state"] == "undetected"
        ]
        
        # Only run 3-frame confirmation during scan state
        if self.state != 'scan':
            return

        # === 3-frame confirmation logic ===
        # Add current frame's detections to history
        self.frame_history.append(filtered_detections)
        if len(self.frame_history) > 3:
            self.frame_history.pop(0)

        # Check for consistent detection across 3 frames
        confirmed_class = self._check_3frame_confirmation()
        
        # Guard: confirm class is a valid target and still undetected
        if confirmed_class is not None and confirmed_class in self.targets and self.targets[confirmed_class]["state"] == "undetected":
            self.targets[confirmed_class]["state"] = "confirmed"
            self.confirmed_pub.publish(Int8(data=int(confirmed_class)))

    def _check_3frame_confirmation(self):
        """
        Check if any class appears in all 3 recent frames within spatial proximity.
        
        Returns:
            class_id if confirmed, None otherwise
        """
        # if len(self.frame_history) < 3:
            # self.get_logger().debug(f"[DEBUG] Not enough frames yet: {len(self.frame_history)}/3")
            # return None
        
        # Get classes present in each frame
        frame_classes = []
        for frame_dets in self.frame_history:
            classes_in_frame = set(int(cls) for _, _, cls in frame_dets)
            frame_classes.append(classes_in_frame)
        
        # Debug: Log classes in each frame
        # self.get_logger().info(f"[DEBUG] Classes per frame: {[list(c) for c in frame_classes]}")

        if len(frame_classes) < 3:
            return
        
        # Find classes present in all 3 frames
        common_classes = frame_classes[0] & frame_classes[1] & frame_classes[2]
        # self.get_logger().info(f"[DEBUG] Common classes across all 3 frames: {list(common_classes)}")
        
        if not common_classes:
            return None
        
        # For each common class, check spatial proximity
        PROXIMITY_THRESHOLD_DEG =  0.00005  #5.5 meters maybe can tighten
        
        for cls in common_classes:
            # Get positions for this class in each frame
            positions = []
            for frame_dets in self.frame_history:
                for lat, lon, c in frame_dets:
                    if int(c) == cls:
                        positions.append((lat, lon))
                        break  # Take first detection of this class per frame
            
            if len(positions) < 3:
                continue
            
            # Check if CONSECUTIVE positions are within proximity
            # Only compare consecutive frames (0->1, 1->2) not all pairs
            # This avoids cumulative drift from failing the check
            all_close = True
            for i in range(len(positions) - 1):
                lat1, lon1 = positions[i]
                lat2, lon2 = positions[i + 1]
                dist = math.sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2)
                if dist > PROXIMITY_THRESHOLD_DEG:
                    all_close = False
                    break
            
            if all_close:
                return cls
        
        return None

    def deployed_location_callback(self, msg: ObjectLocations):
        # Mark target as deployed and store its final location
        cls = msg.class_id
        if cls in self.targets:
            self.targets[cls]["state"] = "deployed"
            self.targets[cls]["lat"] = msg.latitude
            self.targets[cls]["lon"] = msg.longitude

    def mission_state_callback(self, msg: String):
        if msg.data != self.prev_state:
            self.state = msg.data
            # Clear frame history on any state transition to prevent
            # detections from deliver/deploy phases carrying into scan
            if msg.data in ('localize', 'deliver', 'deploy', 'scan'):
                self.frame_history.clear()

            if msg.data == 'scan':
                # Reset non-deployed targets so they can be re-detected
                for cls in self.targets:
                    if self.targets[cls]["state"] != "deployed":
                        self.targets[cls]["state"] = "undetected"

            # once we leave the 'scan' state, write deployed locations
            if self.prev_state == 'scan':
                with open('finalized_object_locations.txt', 'w') as f:
                        for cls, info in self.targets.items():
                            if info["state"] == "deployed":
                                class_name = COCO_CLASS_NAMES[int(cls)] if int(cls) >= 0 else "unknown"
                                f.write(f"{info['lat']:.6f},{info['lon']:.6f},{class_name}\n")

        self.prev_state = msg.data

def main(args=None):
    # Before running, ensure PX4’s yaw mode is set:
    # ros2 param set /px4 MPC_YAW_MODE 0
    rclpy.init(args=args)
    node = FilteringNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
