#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import cv2

from bv_core.pipelines.camera_pipeline import CameraPipeline  # adjust import if needed
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraPipelineTestNode(Node):
    def __init__(self):
        super().__init__('camera_pipeline_test_node')

        # Example GStreamer pipeline: modify to match your setup
        self.gst = "v4l2src device=/dev/video0 io-mode=2 do-timestamp=true ! video/x-raw,format=UYVY,width=1920,height=1080,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! queue max-size-buffers=1 leaky=downstream ! appsink drop=true sync=false"


        # Create CameraPipeline (record to /image_compressed)
        self.camera = CameraPipeline(
            gst_pipeline=self.gst,
            record=True,
            ros_context=self,
            fps=30.0,
            max_queue_size=2
        )

        self.window_name = "CameraPipelineTestNode"
        self.display_height = 1080
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

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

        if frame.shape[0] > self.display_height:
            scale = self.display_height / float(frame.shape[0])
            new_width = max(1, int(frame.shape[1] * scale))
            display_frame = cv2.resize(frame, (new_width, self.display_height))
        else:
            display_frame = frame

        cv2.resizeWindow(self.window_name, display_frame.shape[1], display_frame.shape[0])
        cv2.imshow(self.window_name, display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Received 'q' — shutting down display")
            rclpy.shutdown()

        if self.frames_received % 30 == 0:
            elapsed = time.time() - self.start_time
            fps = self.frames_received / elapsed
            self.get_logger().info(f"Streaming OK — {self.frames_received} frames @ {fps:.1f} FPS")

    def destroy_node(self):
        self.get_logger().info("Stopping camera pipeline...")
        self.camera.stop()
        cv2.destroyAllWindows()
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
