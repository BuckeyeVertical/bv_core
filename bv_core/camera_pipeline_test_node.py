#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from bv_core.pipelines.camera_pipeline import CameraPipeline  # adjust import if needed
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraPipelineTestNode(Node):
    def __init__(self):
        super().__init__('camera_pipeline_test_node')

        # Example GStreamer pipeline: modify to match your setup
        self.gst = (
            "v4l2src device=/dev/video0 io-mode=2 do-timestamp=true ! "
            "image/jpeg,width=3840,height=2160,framerate=24/1 ! jpegdec"
            "videoconvert"
        )

        # Create CameraPipeline (record to /image_compressed)
        self.camera = CameraPipeline(
            gst_pipeline=self.gst,
            record=True,
            ros_context=self,
            fps=30.0,
            max_queue_size=2
        )

        # Optional publisher to visualize raw frames (uncompressed)
        self.raw_pub = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()

        self.camera.start()
        self.get_logger().info("Camera pipeline started")

        # Timer loop to fetch frames
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.frames_received = 0
        self.start_time = time.time()

    def timer_cb(self):
        frame = self.camera.get_frame(timeout=0.05)
        if frame is None:
            return

        self.frames_received += 1
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.raw_pub.publish(msg)

        if self.frames_received % 30 == 0:
            elapsed = time.time() - self.start_time
            fps = self.frames_received / elapsed
            self.get_logger().info(f"Streaming OK — {self.frames_received} frames @ {fps:.1f} FPS")

    def destroy_node(self):
        self.get_logger().info("Stopping camera pipeline...")
        self.camera.stop()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPipelineTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
