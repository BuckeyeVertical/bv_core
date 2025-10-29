
#!/usr/bin/env python3
"""
ROSbag Vision Processing Pipeline
Processes rosbag files with vision detection, filtering, and stitching.
Extracts images, runs object detection, geolocates detections, and creates stitched maps.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import yaml
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import List, Dict, Tuple, Optional
import argparse
from collections import deque
import threading
import queue


@dataclass
class Detection:
    """Single object detection"""
    class_id: int
    class_name: str
    confidence: float
    bbox: List[float]  # [x1, y1, x2, y2]
    timestamp: float
    frame_id: int


@dataclass
class GeolocatedDetection:
    """Detection with GPS coordinates"""
    detection: Detection
    latitude: float
    longitude: float
    altitude: float
    pose_timestamp: float


@dataclass
class ProcessingStats:
    """Statistics for the processing run"""
    total_frames: int = 0
    frames_processed: int = 0
    detections_found: int = 0
    geolocated_detections: int = 0
    stitched_images: int = 0
    start_time: float = 0
    end_time: float = 0


class BagVisionProcessor(Node):
    """Main processor node for rosbag vision pipeline"""
    
    def __init__(self, config_path: str, output_dir: str):
        super().__init__('bag_vision_processor')
        
        self.bridge = CvBridge()
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Setup directories
        self.frames_dir = self.output_dir / 'frames'
        self.annotated_dir = self.output_dir / 'annotated_frames'
        self.detections_dir = self.output_dir / 'detections'
        self.stitched_dir = self.output_dir / 'stitched'
        
        for d in [self.frames_dir, self.annotated_dir, self.detections_dir, self.stitched_dir]:
            d.mkdir(exist_ok=True)
        
        # Processing queues and buffers
        self.image_queue = deque(maxlen=1000)
        self.pose_buffer = deque(maxlen=100)
        self.gps_buffer = deque(maxlen=100)
        self.altitude_buffer = deque(maxlen=100)
        
        self.detections: List[Detection] = []
        self.geolocated_detections: List[GeolocatedDetection] = []
        self.images_for_stitching: List[np.ndarray] = []
        
        self.stats = ProcessingStats()
        self.frame_count = 0
        
        # Camera parameters
        self.camera_matrix = np.array(self.config['filtering']['c_matrix'])
        self.dist_coeffs = np.array(self.config['filtering']['dist_coefficients'])
        self.camera_orientation = np.array(self.config['filtering']['camera_orientation'])
        
        # Vision parameters
        self.detection_threshold = self.config['vision']['detection_threshold']
        self.batch_size = self.config['vision']['batch_size']
        
        # Setup subscribers
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.altitude_callback, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        
        self.get_logger().info(f'Bag Vision Processor initialized. Output: {self.output_dir}')
    
    def image_callback(self, msg: Image):
        """Process incoming images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            self.image_queue.append((cv_image, timestamp, self.frame_count))
            self.frame_count += 1
            self.stats.total_frames += 1
            
            # Save raw frame
            if self.config.get('save_raw_frames', False):
                frame_path = self.frames_dir / f'frame_{self.frame_count:06d}.jpg'
                cv2.imwrite(str(frame_path), cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')
    
    def gps_callback(self, msg: NavSatFix):
        """Buffer GPS data"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gps_buffer.append({
            'timestamp': timestamp,
            'lat': msg.latitude,
            'lon': msg.longitude,
            'status': msg.status.status
        })
    
    def altitude_callback(self, msg: Float64):
        """Buffer altitude data"""
        timestamp = self.get_clock().now().seconds_nanoseconds()
        timestamp = timestamp[0] + timestamp[1] * 1e-9
        self.altitude_buffer.append({
            'timestamp': timestamp,
            'altitude': msg.data
        })
    
    def pose_callback(self, msg: PoseStamped):
        """Buffer pose data"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pose_buffer.append({
            'timestamp': timestamp,
            'position': msg.pose.position,
            'orientation': msg.pose.orientation
        })
    
    def find_closest_data(self, timestamp: float, buffer: deque, max_dt: float = 0.5) -> Optional[Dict]:
        """Find closest data point in buffer by timestamp"""
        if not buffer:
            return None
        
        closest = min(buffer, key=lambda x: abs(x['timestamp'] - timestamp))
        if abs(closest['timestamp'] - timestamp) > max_dt:
            return None
        return closest
    
    def process_detections_batch(self):
        """Process a batch of images for detection"""
        if len(self.image_queue) < self.batch_size:
            return
        
        batch_images = []
        batch_metadata = []
        
        for _ in range(min(self.batch_size, len(self.image_queue))):
            img, ts, frame_id = self.image_queue.popleft()
            batch_images.append(img)
            batch_metadata.append((ts, frame_id))
        
        # Run detection (placeholder - integrate your detection model here)
        for img, (ts, frame_id) in zip(batch_images, batch_metadata):
            detections = self.run_detection(img, ts, frame_id)
            
            if detections:
                self.detections.extend(detections)
                self.stats.detections_found += len(detections)
                
                # Geolocate detections
                for det in detections:
                    geo_det = self.geolocate_detection(det)
                    if geo_det:
                        self.geolocated_detections.append(geo_det)
                        self.stats.geolocated_detections += 1
                
                # Save annotated frame
                annotated = self.draw_detections(img, detections)
                ann_path = self.annotated_dir / f'annotated_{frame_id:06d}.jpg'
                cv2.imwrite(str(ann_path), annotated)
            
            self.stats.frames_processed += 1
            
            # Add to stitching queue if configured
            if self.config.get('enable_stitching', True):
                if len(self.images_for_stitching) < self.config.get('max_stitch_images', 20):
                    self.images_for_stitching.append(img)
    
    def run_detection(self, image: np.ndarray, timestamp: float, frame_id: int) -> List[Detection]:
        """
        Run object detection on image
        PLACEHOLDER: Integrate your detection model (RF-DETR, YOLO, etc.)
        """
        detections = []
        
        # Example: Simple color-based detection (replace with actual model)
        # This is just a placeholder to demonstrate the structure
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Detect red objects (example)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > 100:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                confidence = min(area / 10000, 1.0)  # Fake confidence
                
                if confidence > self.detection_threshold:
                    det = Detection(
                        class_id=0,
                        class_name='object',
                        confidence=confidence,
                        bbox=[x, y, x+w, y+h],
                        timestamp=timestamp,
                        frame_id=frame_id
                    )
                    detections.append(det)
        
        return detections
    
    def geolocate_detection(self, detection: Detection) -> Optional[GeolocatedDetection]:
        """Convert detection to GPS coordinates"""
        # Find closest GPS and altitude data
        gps_data = self.find_closest_data(detection.timestamp, self.gps_buffer)
        alt_data = self.find_closest_data(detection.timestamp, self.altitude_buffer)
        pose_data = self.find_closest_data(detection.timestamp, self.pose_buffer)
        
        if not (gps_data and alt_data and pose_data):
            return None
        
        # Calculate pixel coordinates (center of bbox)
        x1, y1, x2, y2 = detection.bbox
        pixel_x = (x1 + x2) / 2
        pixel_y = (y1 + y2) / 2
        
        # Project to ground (simplified - you may need more sophisticated projection)
        # This uses camera intrinsics and altitude for basic projection
        lat_offset, lon_offset = self.project_pixel_to_ground(
            pixel_x, pixel_y, 
            alt_data['altitude'],
            pose_data['orientation']
        )
        
        return GeolocatedDetection(
            detection=detection,
            latitude=gps_data['lat'] + lat_offset,
            longitude=gps_data['lon'] + lon_offset,
            altitude=alt_data['altitude'],
            pose_timestamp=pose_data['timestamp']
        )
    
    def project_pixel_to_ground(self, px: float, py: float, altitude: float, orientation) -> Tuple[float, float]:
        """
        Project pixel to ground coordinates
        Simplified version - implement full camera projection based on your setup
        """
        # Undistort pixel
        pixel = np.array([[[px, py]]], dtype=np.float32)
        undistorted = cv2.undistortPoints(pixel, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
        
        # Simple projection (you need to implement proper camera model)
        # This is a placeholder calculation
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Normalized coordinates
        x_norm = (undistorted[0][0][0] - cx) / fx
        y_norm = (undistorted[0][0][1] - cy) / fy
        
        # Project to ground (assumes nadir-looking camera)
        ground_x = x_norm * altitude
        ground_y = y_norm * altitude
        
        # Convert to lat/lon offsets (rough approximation)
        lat_offset = ground_y / 111320.0  # meters to degrees latitude
        lon_offset = ground_x / (111320.0 * np.cos(np.radians(0)))  # adjust for longitude
        
        return lat_offset, lon_offset
    
    def draw_detections(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """Draw bounding boxes and labels on image"""
        annotated = image.copy()
        
        for det in detections:
            x1, y1, x2, y2 = map(int, det.bbox)
            
            # Draw box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f'{det.class_name}: {det.confidence:.2f}'
            cv2.putText(annotated, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return annotated
    
    def stitch_images(self):
        """Create panorama from collected images"""
        if len(self.images_for_stitching) < 2:
            self.get_logger().warn('Not enough images for stitching')
            return
        
        self.get_logger().info(f'Stitching {len(self.images_for_stitching)} images...')
        
        try:
            stitcher = cv2.Stitcher.create(cv2.Stitcher_PANORAMA)
            status, panorama = stitcher.stitch(self.images_for_stitching)
            
            if status == cv2.Stitcher_OK:
                output_path = self.stitched_dir / f'panorama_{datetime.now().strftime("%Y%m%d_%H%M%S")}.jpg'
                cv2.imwrite(str(output_path), panorama)
                self.stats.stitched_images += 1
                self.get_logger().info(f'Panorama saved: {output_path}')
            else:
                self.get_logger().error(f'Stitching failed with status: {status}')
        
        except Exception as e:
            self.get_logger().error(f'Error during stitching: {e}')
    
    def save_results(self):
        """Save detection results and statistics"""
        # Save detections
        detections_file = self.detections_dir / 'detections.json'
        with open(detections_file, 'w') as f:
            json.dump([asdict(d) for d in self.detections], f, indent=2)
        
        # Save geolocated detections
        geo_file = self.detections_dir / 'geolocated_detections.json'
        geo_data = []
        for gd in self.geolocated_detections:
            geo_dict = {
                'detection': asdict(gd.detection),
                'latitude': gd.latitude,
                'longitude': gd.longitude,
                'altitude': gd.altitude,
                'pose_timestamp': gd.pose_timestamp
            }
            geo_data.append(geo_dict)
        
        with open(geo_file, 'w') as f:
            json.dump(geo_data, f, indent=2)
        
        # Save statistics
        self.stats.end_time = datetime.now().timestamp()
        stats_file = self.output_dir / 'processing_stats.json'
        with open(stats_file, 'w') as f:
            json.dump(asdict(self.stats), f, indent=2)
        
        # Create CSV for easy viewing
        csv_file = self.detections_dir / 'geolocated_detections.csv'
        with open(csv_file, 'w') as f:
            f.write('frame_id,timestamp,class_name,confidence,latitude,longitude,altitude\n')
            for gd in self.geolocated_detections:
                f.write(f'{gd.detection.frame_id},{gd.detection.timestamp},'
                       f'{gd.detection.class_name},{gd.detection.confidence},'
                       f'{gd.latitude},{gd.longitude},{gd.altitude}\n')
        
        self.get_logger().info(f'Results saved to {self.output_dir}')
        self.get_logger().info(f'Statistics: {asdict(self.stats)}')
    
    def finalize(self):
        """Finalize processing"""
        self.get_logger().info('Finalizing processing...')
        
        # Process remaining images
        while self.image_queue:
            self.process_detections_batch()
        
        # Create stitched panorama
        if self.config.get('enable_stitching', True):
            self.stitch_images()
        
        # Save all results
        self.save_results()


def main():
    parser = argparse.ArgumentParser(description='Process ROSbag with vision pipeline')
    parser.add_argument('--config', type=str, required=True, help='Path to config YAML')
    parser.add_argument('--output', type=str, required=True, help='Output directory')
    parser.add_argument('--bag', type=str, required=True, help='Path to rosbag file')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    processor = Node('bag_processor_launcher')
    processor.get_logger().info(f'Starting bag playback: {args.bag}')
    
    # Create vision processor
    vision_processor = BagVisionProcessor(args.config, args.output)
    vision_processor.stats.start_time = datetime.now().timestamp()
    
    try:
        # Spin the processor while bag plays
        # Note: You need to run ros2 bag play separately or integrate it here
        processor.get_logger().info('Processing bag... Press Ctrl+C to stop')
        rclpy.spin(vision_processor)
    
    except KeyboardInterrupt:
        processor.get_logger().info('Processing interrupted')
    
    finally:
        vision_processor.finalize()
        vision_processor.destroy_node()
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
