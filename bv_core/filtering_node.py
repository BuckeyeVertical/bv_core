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

from sensor_msgs.msg import NavSatFix

from bv_msgs.msg import ObjectLocations         # your custom msg
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from collections import deque
from rclpy.time import Time
import math
from .mission_logger import MissionLogger

CLASS_NAMES = ("person", "tent")


def is_within_radius(lat, lon, ref_lat, ref_lon, radius_deg):
    """True when (lat, lon) is inside radius_deg of (ref_lat, ref_lon).

    Euclidean in degrees — the mission area is small enough that treating
    degrees as a flat plane is well within the localization error, and it
    matches how proximity_threshold_deg is already used for 3-frame clustering.
    """
    d_lat = lat - ref_lat
    d_lon = lon - ref_lon
    return (d_lat * d_lat + d_lon * d_lon) <= (radius_deg * radius_deg)


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

        # Locations the operator rejected. Detections of the SAME class near
        # one of these are dropped before 3-frame confirmation, so a rejected
        # false positive cannot immediately re-confirm and re-interrupt.
        self.rejected_location_sub = self.create_subscription(
            ObjectLocations,
            '/rejected_object_locations',
            self.rejected_location_callback,
            reliable_qos
        )
        self.rejected_locations = []   # list of (lat, lon, class_id)

        self.gps_buffer  = deque(maxlen=200)
        self.pose_buffer = deque(maxlen=200)

        # === internal state ===
        self.prev_state = None
        self.state = None
        
        # === target tracking ===
        # One entry per competition target: person and tent
        self.targets = {
            class_id: {"state": "undetected", "lat": None, "lon": None,
                       "confirmed_lat": None, "confirmed_lon": None}
            for class_id in range(len(CLASS_NAMES))
        }
        
        # === 3-frame confirmation tracking ===
        self.frame_history = []  # list of recent detection sets [(lat, lon, class_id), ...]
        self._last_frame_window_summary = None
        
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
        self.log = MissionLogger('filtering')
        self.proximity_threshold_deg = 0.0001
        self.rejected_ignore_radius_deg = float(
            cfg.get('rejected_ignore_radius_deg', 0.0001))

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

        # filter to target classes that haven't been confirmed or deployed yet,
        # and drop anything the operator already rejected at this spot
        filtered_detections = [
            (lat, lon, cls)
            for lat, lon, cls in detections_global
            if cls in self.targets
            and self.targets[cls]["state"] == "undetected"
            and not self._is_rejected(lat, lon, cls)
        ]
        
        # Only run 3-frame confirmation during scan state
        if self.state != 'scan':
            return

        # === 3-frame confirmation logic ===
        # Add current frame's detections to history
        self.frame_history.append(filtered_detections)
        if len(self.frame_history) > 3:
            self.frame_history.pop(0)

        # Print the current 3-frame detection window in plain English.
        frame_summaries = []
        for i, frame_dets in enumerate(self.frame_history):
            class_names = sorted(
                set(
                    CLASS_NAMES[int(cls)] if 0 <= int(cls) < len(CLASS_NAMES) else "unknown"
                    for _, _, cls in frame_dets
                )
            )
            detected_text = ', '.join(class_names) if class_names else 'no objects'
            frame_summaries.append(f"frame {i + 1}: {detected_text}")
        frame_window_summary = "Recent detection window: " + " | ".join(frame_summaries)
        if frame_window_summary != self._last_frame_window_summary:
            self.get_logger().info(frame_window_summary)
            self._last_frame_window_summary = frame_window_summary

        # Log frame window with distances for tuning
        window_data = {}
        for cls_id in set(int(c) for frame in self.frame_history for _, _, c in frame):
            cls_name = CLASS_NAMES[cls_id] if 0 <= cls_id < len(CLASS_NAMES) else "unknown"
            positions = []
            for frame_dets in self.frame_history:
                for lat, lon, c in frame_dets:
                    if int(c) == cls_id:
                        positions.append((lat, lon))
                        break
            dists = []
            for j in range(len(positions) - 1):
                d_deg = math.sqrt((positions[j+1][0] - positions[j][0])**2 +
                                  (positions[j+1][1] - positions[j][1])**2)
                d_m = d_deg * 111320.0
                dists.append(d_m)
            window_data[cls_name] = {
                'dists': dists,
                'thresh': self.proximity_threshold_deg * 111320.0,
            }
        self.log.frame_window(len(self.frame_history), 3, window_data)
        # Check for consistent detection across 3 frames
        confirmed = self._check_3frame_confirmation()
        confirmed_class, confirmed_positions = (
            confirmed if confirmed is not None else (None, None))

        # Guard: confirm class is a valid target and still undetected
        if confirmed_class is not None and confirmed_class in self.targets and self.targets[confirmed_class]["state"] == "undetected":
            self.targets[confirmed_class]["state"] = "confirmed"
            # Record where FILTERING believes the object is, from the three
            # frames that just agreed. Rejection suppression compares future
            # scan-leg detections against this rather than against the
            # loiter-time estimate mission_node publishes: same projection,
            # altitude and angle, so cross-vantage error cancels and the
            # radius only has to cover frame-to-frame jitter.
            centroid = self._centroid(confirmed_positions)
            if centroid is not None:
                self.targets[confirmed_class]["confirmed_lat"] = centroid[0]
                self.targets[confirmed_class]["confirmed_lon"] = centroid[1]
            self.confirmed_pub.publish(Int8(data=int(confirmed_class)))

    @staticmethod
    def _centroid(points):
        """Mean (lat, lon) of the given points, or None if there are none.

        The caller passes the exact per-frame positions that 3-frame
        confirmation matched — one per frame, the first of that class. Averaging
        *every* same-class detection instead would place the stored position
        between a false positive and a real object standing in the same frame,
        so the suppression circle would cover neither.
        """
        if not points:
            return None
        return (sum(p[0] for p in points) / len(points),
                sum(p[1] for p in points) / len(points))

    def _check_3frame_confirmation(self):
        """
        Check if any class appears in all 3 recent frames within spatial proximity.
        
        Returns:
            (class_id, positions) if confirmed, None otherwise. `positions` is
            the one-per-frame list of (lat, lon) that actually satisfied the
            proximity check, so callers store exactly what was validated.
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
        PROXIMITY_THRESHOLD_DEG = self.proximity_threshold_deg
        
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
            all_close = True
            dists_m = []
            for i in range(len(positions) - 1):
                lat1, lon1 = positions[i]
                lat2, lon2 = positions[i + 1]
                dist = math.sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2)
                dists_m.append(dist * 111320.0)
                if dist > PROXIMITY_THRESHOLD_DEG:
                    all_close = False
                    break

            cls_name = CLASS_NAMES[int(cls)] if 0 <= int(cls) < len(CLASS_NAMES) else "unknown"
            thresh_m = PROXIMITY_THRESHOLD_DEG * 111320.0
            if all_close:
                self.log.event('CONFIRMED',
                    f"class={cls_name}({cls}), "
                    f"dists=[{','.join(f'{d:.1f}m' for d in dists_m)}], "
                    f"thresh={thresh_m:.1f}m")
                return cls, positions
            else:
                self.log.event('REJECTED',
                    f"class={cls_name}({cls}), "
                    f"dists=[{','.join(f'{d:.1f}m' for d in dists_m)}], "
                    f"thresh={thresh_m:.1f}m, reason=proximity_fail")

        return None

    def deployed_location_callback(self, msg: ObjectLocations):
        # Mark target as deployed and store its final location
        cls = msg.class_id
        if cls in self.targets:
            self.targets[cls]["state"] = "deployed"
            self.targets[cls]["lat"] = msg.latitude
            self.targets[cls]["lon"] = msg.longitude

    def rejected_location_callback(self, msg: ObjectLocations):
        """Record a location the operator rejected for a specific class."""
        cls = int(msg.class_id)
        # Prefer the position filtering itself confirmed: future detections
        # arrive in that same frame, so the cross-vantage projection error
        # against mission_node's loiter-time estimate cancels. The message
        # coordinates are the fallback when this class was never confirmed
        # here (filtering restarted, or a rejection for an unseen class).
        target = self.targets.get(cls)
        if target is not None and target.get("confirmed_lat") is not None:
            lat, lon = target["confirmed_lat"], target["confirmed_lon"]
            source = 'confirmed'
        else:
            lat, lon = msg.latitude, msg.longitude
            source = 'message'

        entry = (lat, lon, cls)
        self.rejected_locations.append(entry)
        cls_name = (CLASS_NAMES[cls]
                    if 0 <= cls < len(CLASS_NAMES) else 'unknown')
        self.get_logger().info(
            f"Suppressing {cls_name} within "
            f"{self.rejected_ignore_radius_deg * 111320.0:.1f}m of "
            f"lat={lat:.6f}, lon={lon:.6f} (source={source})")
        self.log.event('REJECTED_LOCATION',
                       f"lat={lat:.6f}, lon={lon:.6f}, "
                       f"class={cls_name}({cls}), "
                       f"radius={self.rejected_ignore_radius_deg}deg, "
                       f"source={source}, "
                       f"msg_lat={msg.latitude:.6f}, "
                       f"msg_lon={msg.longitude:.6f}")

    def _is_rejected(self, lat, lon, class_id):
        """True when this class was rejected by the operator near this point."""
        return any(
            int(class_id) == rej_cls
            and is_within_radius(lat, lon, rej_lat, rej_lon,
                                 self.rejected_ignore_radius_deg)
            for rej_lat, rej_lon, rej_cls in self.rejected_locations
        )

    def mission_state_callback(self, msg: String):
        if msg.data != self.prev_state:
            self.state = msg.data
            # Clear frame history on any state transition to prevent
            # detections from deliver/deploy phases carrying into scan
            if msg.data in ('localize', 'deliver', 'deploy', 'scan'):
                self.frame_history.clear()
                self._last_frame_window_summary = None

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
                                class_name = (
                                    CLASS_NAMES[int(cls)]
                                    if 0 <= int(cls) < len(CLASS_NAMES)
                                    else "unknown"
                                )
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
