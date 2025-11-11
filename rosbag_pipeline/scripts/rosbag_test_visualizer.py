#!/usr/bin/env python3
"""
ROSbag Vision Pipeline Test with Live Visualization
Processes rosbag and shows detections in cv2 window in real-time
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import argparse
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Dict
import yaml
from pathlib import Path


@dataclass
class Detection:
    """Single object detection"""
    class_id: int
    class_name: str
    confidence: float
    bbox: List[float]  # [x1, y1, x2, y2]
    timestamp: float
    frame_id: int


class BagVisualizerNode(Node):
    """Test node with live visualization"""
    
    def __init__(self, config_path: str = None):
        super().__init__('bag_visualizer')
        
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # Load config if provided
        if config_path and Path(config_path).exists():
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            self.detection_threshold = self.config['vision']['detection_threshold']
        else:
            self.get_logger().warn('No config provided, using defaults')
            self.detection_threshold = 0.3
        
        # Data buffers for geolocation
        self.pose_buffer = deque(maxlen=100)
        self.gps_buffer = deque(maxlen=100)
        self.altitude_buffer = deque(maxlen=100)
        
        # Statistics
        self.total_detections = 0
        self.total_frames = 0
        
        # Create window
        cv2.namedWindow('ROSbag Vision Pipeline', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('ROSbag Vision Pipeline', 1280, 720)
        
        # Subscribers
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.altitude_callback, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        
        self.get_logger().info('Bag Visualizer Node Started')
        self.get_logger().info('Waiting for images from /image_raw...')
        self.get_logger().info('Press Q in the window to quit')
    
    def image_callback(self, msg: Image):
        """Process and display images with detections"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            self.total_frames += 1
            
            # Run detection
            detections = self.run_detection(cv_image, timestamp, self.frame_count)
            self.total_detections += len(detections)
            
            # Draw detections
            annotated = self.draw_detections(cv_image, detections)
            
            # Add GPS info if available
            annotated = self.add_telemetry_overlay(annotated, timestamp)
            
            # Show the frame
            cv2.imshow('ROSbag Vision Pipeline', annotated)
            
            # Check for quit (1ms wait for key press)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                self.get_logger().info('Quit requested')
                rclpy.shutdown()
            
            self.frame_count += 1
            
            # Log progress every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Processed {self.frame_count} frames, '
                    f'Found {self.total_detections} detections'
                )
        
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
    
    def run_detection(self, image: np.ndarray, timestamp: float, frame_id: int) -> List[Detection]:
        """
        Run object detection on image
        This is a DEMO detection using color detection
        Replace with your actual model (RF-DETR, YOLO, etc.)
        """
        detections = []
        
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for different objects (example)
        color_ranges = [
            # Red objects
            {
                'name': 'red_object',
                'lower': np.array([0, 100, 100]),
                'upper': np.array([10, 255, 255]),
                'color': (0, 0, 255)
            },
            # Blue objects
            {
                'name': 'blue_object',
                'lower': np.array([100, 100, 100]),
                'upper': np.array([130, 255, 255]),
                'color': (255, 0, 0)
            },
            # Green objects
            {
                'name': 'green_object',
                'lower': np.array([40, 100, 100]),
                'upper': np.array([80, 255, 255]),
                'color': (0, 255, 0)
            },
        ]
        
        for class_id, color_range in enumerate(color_ranges):
            # Create mask for this color
            mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
            
            # Apply morphological operations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate confidence based on area and shape
                    confidence = min(area / 5000, 1.0)
                    aspect_ratio = w / h if h > 0 else 0
                    
                    # Filter by aspect ratio (roughly square objects)
                    if 0.5 < aspect_ratio < 2.0 and confidence > self.detection_threshold:
                        det = Detection(
                            class_id=class_id,
                            class_name=color_range['name'],
                            confidence=confidence,
                            bbox=[x, y, x+w, y+h],
                            timestamp=timestamp,
                            frame_id=frame_id
                        )
                        detections.append(det)
        
        return detections
    
    def draw_detections(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """Draw bounding boxes and labels on image"""
        annotated = image.copy()
        
        # Color map for different classes
        colors = [
            (0, 0, 255),    # Red
            (255, 0, 0),    # Blue
            (0, 255, 0),    # Green
            (0, 255, 255),  # Yellow
            (255, 0, 255),  # Magenta
        ]
        
        for det in detections:
            x1, y1, x2, y2 = map(int, det.bbox)
            color = colors[det.class_id % len(colors)]
            
            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            
            # Draw filled label background
            label = f'{det.class_name}: {det.confidence:.2f}'
            (label_w, label_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            cv2.rectangle(
                annotated, 
                (x1, y1 - label_h - baseline - 5), 
                (x1 + label_w, y1), 
                color, 
                -1
            )
            
            # Draw label text
            cv2.putText(
                annotated, label, (x1, y1 - baseline - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
            )
            
            # Draw center point
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            cv2.circle(annotated, (center_x, center_y), 3, color, -1)
        
        return annotated
    
    def add_telemetry_overlay(self, image: np.ndarray, timestamp: float) -> np.ndarray:
        """Add GPS and altitude overlay to image"""
        h, w = image.shape[:2]
        
        # Semi-transparent overlay panel
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (400, 150), (0, 0, 0), -1)
        image = cv2.addWeighted(overlay, 0.6, image, 0.4, 0)
        
        # Get latest GPS data
        gps_text = "GPS: No data"
        alt_text = "Alt: No data"
        
        if self.gps_buffer:
            latest_gps = self.gps_buffer[-1]
            gps_text = f"GPS: {latest_gps['lat']:.6f}, {latest_gps['lon']:.6f}"
        
        if self.altitude_buffer:
            latest_alt = self.altitude_buffer[-1]
            alt_text = f"Alt: {latest_alt['altitude']:.2f}m"
        
        # Draw text
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image, f"Frame: {self.frame_count}", (20, 35), font, 0.6, (0, 255, 0), 2)
        cv2.putText(image, f"Detections: {self.total_detections}", (20, 60), font, 0.6, (0, 255, 0), 2)
        cv2.putText(image, gps_text, (20, 85), font, 0.5, (255, 255, 255), 1)
        cv2.putText(image, alt_text, (20, 110), font, 0.5, (255, 255, 255), 1)
        cv2.putText(image, "Press 'Q' to quit", (20, 135), font, 0.5, (255, 255, 0), 1)
        
        return image
    
    def cleanup(self):
        """Clean up resources"""
        cv2.destroyAllWindows()
        self.get_logger().info(f'Total frames processed: {self.total_frames}')
        self.get_logger().info(f'Total detections found: {self.total_detections}')


def main():
    parser = argparse.ArgumentParser(description='Test ROSbag vision pipeline with visualization')
    parser.add_argument('--config', type=str, help='Path to config YAML (optional)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    visualizer = BagVisualizerNode(args.config)
    
    try:
        print("\n" + "="*60)
        print("ROSbag Vision Pipeline Test")
        print("="*60)
        print("\nInstructions:")
        print("1. In another terminal, run: ros2 bag play <your_bag.db3>")
        print("2. Watch detections appear in the cv2 window")
        print("3. Press 'Q' in the window to quit")
        print("\nWaiting for images...")
        print("="*60 + "\n")
        
        rclpy.spin(visualizer)
    
    except KeyboardInterrupt:
        print('\nShutdown requested')
    
    finally:
        visualizer.cleanup()
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
