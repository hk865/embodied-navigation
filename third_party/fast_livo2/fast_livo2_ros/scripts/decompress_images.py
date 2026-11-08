#!/usr/bin/env python3
"""Decompress /camera/image_color/compressed → /camera/image_color for bag playback."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2

class DecompressNode(Node):
    def __init__(self):
        super().__init__('image_decompress')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            CompressedImage, '/camera/image_color/compressed', self.callback, 10)
        self.pub = self.create_publisher(Image, '/camera/image_color', 10)
        self.get_logger().info('Decompress node ready: /camera/image_color/compressed → /camera/image_color')

    def callback(self, msg: CompressedImage):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            img_msg.header = msg.header  # preserve original timestamp
            self.pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Decompress failed: {e}')

def main():
    rclpy.init()
    node = DecompressNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
