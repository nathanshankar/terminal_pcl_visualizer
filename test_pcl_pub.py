#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import time

class PCLPublisher(Node):
    def __init__(self):
        super().__init__('pcl_publisher')
        self.publisher_ = self.publisher_ = self.create_publisher(PointCloud2, '/points', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0

    def timer_callback(self):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Create a cube in front of the camera (X > 0)
        points = []
        for x in np.linspace(1.0, 3.0, 5):
            for y in np.linspace(-1.0, 1.0, 5):
                for z in np.linspace(-1.0, 1.0, 5):
                    # Add some rotation
                    rx = x
                    ry = y * np.cos(self.angle) - z * np.sin(self.angle)
                    rz = y * np.sin(self.angle) + z * np.cos(self.angle)
                    points.append([rx, ry, rz])
        
        self.angle += 0.1
        
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        
        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', *p))
        msg.data = b''.join(buffer)
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = PCLPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
