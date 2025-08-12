#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np


class SLAMNode(Node):
    """
    SLAM Node for micromouse robot
    Implements simultaneous localization and mapping using sensor data
    """
    
    def __init__(self):
        super().__init__('slam_node')
        
        # Publishers
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'pose', 10)
        
        # Subscribers
        self.laser_subscriber = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # SLAM parameters
        self.map_resolution = 0.05  # meters per pixel
        self.map_width = 400
        self.map_height = 400
        
        # Initialize map
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        self.get_logger().info('SLAM Node initialized')
    
    def laser_callback(self, msg):
        """Process laser scan data for mapping"""
        self.get_logger().debug(f'Received laser scan with {len(msg.ranges)} points')
        # TODO: Implement scan matching and map update
        
    def odom_callback(self, msg):
        """Process odometry data for localization"""
        self.get_logger().debug('Received odometry data')
        # TODO: Implement pose estimation and correction
        
    def publish_map(self):
        """Publish current map as OccupancyGrid"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = -self.map_width * self.map_resolution / 2.0
        map_msg.info.origin.position.y = -self.map_height * self.map_resolution / 2.0
        
        map_msg.data = self.occupancy_grid.flatten().tolist()
        
        self.map_publisher.publish(map_msg)


def main(args=None):
    rclpy.init(args=args)
    
    slam_node = SLAMNode()
    
    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()