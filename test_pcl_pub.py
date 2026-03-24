#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import cv2

class LogoPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('logo_pcl_pub')

        self.publisher_ = self.create_publisher(PointCloud2, '/points', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        import os
        img_path = os.path.join(os.path.dirname(__file__), 'osrf.png')
        self.img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if self.img is None:
            raise FileNotFoundError(f"Image file 'osrf.png' not found at {img_path}.")

        self.img = cv2.resize(self.img, (200, 200))

        _, self.binary = cv2.threshold(self.img, 200, 255, cv2.THRESH_BINARY_INV)

        self.points = self.image_to_points(self.binary)

    def image_to_points(self, binary_img):
        h, w = binary_img.shape
        pts = []

        scale = 0.01 

        for v in range(h):
            for u in range(w):
                if binary_img[v, u] > 0:
                    x = 0.0
                    y = (u - w / 2) * scale
                    z = (v - h / 2) * scale

                    pts.append([x, y, -z]) 

        return pts

    def timer_callback(self):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.height = 1
        msg.width = len(self.points)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True

        buffer = [struct.pack('fff', *p) for p in self.points]
        msg.data = b''.join(buffer)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LogoPointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()