#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf_transformations import euler_from_quaternion
import tf2_ros

import numpy as np
import math
import scipy.ndimage

def distance_transform(binary_occupancy_matrix):
    """
    Calculate distances from free cells to the nearest obstacle.
    Obstacles are True; inverted to 0 for the distance transform.
    """
    distance_matrix = scipy.ndimage.distance_transform_edt(1 - binary_occupancy_matrix)
    return distance_matrix

class Costmap(Node):
    
    def __init__(self):
        super().__init__('costmap', 
                         allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # Load parameters
        self.rate = self.get_parameter('rate').value
        self.min_cost = self.get_parameter('min_cost').value
        self.max_cost = self.get_parameter('max_cost').value
        self.decay_rate = self.get_parameter('decay_rate').value
        self.clearance = self.get_parameter('clearance').value

        # Robot and goal information
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_a = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Subscribers
        self.pose_subscriber = self.create_subscription(
            PoseStamped, 'odom_pose', self.pose_callback, 1
        )
        self.goal_subscriber = self.create_subscription(
            PoseStamped, 'goal', self.goal_callback, 1
        )
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 1
        )
        map_qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, qos_profile=map_qos_profile
        )

        # Publisher
        costmap_qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid, 'costmap', qos_profile=costmap_qos_profile
        )

        # Internal variables
        self.map_msg = None
        self.scan_msg = None

        # Transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer for periodic updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

    def pose_callback(self, msg):
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])[2]

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
    
    def scan_callback(self, msg):
        self.scan_msg = msg

    def map_callback(self, msg):
        self.map_msg = msg

    def timer_callback(self):
        if self.map_msg is None:
            return

        # Convert the occupancy grid to a binary matrix
        occgrid_msg = self.map_msg
        occupancy_matrix = np.array(
            occgrid_msg.data, dtype=np.int8
        ).reshape(
            occgrid_msg.info.height, occgrid_msg.info.width
        )
        binary_occupancy_matrix = occupancy_matrix >= 100

        # Compute distance transform and adjust for clearance
        distance_matrix = distance_transform(binary_occupancy_matrix)
        metric_distance_matrix = distance_matrix * occgrid_msg.info.resolution
        adjusted_dist = np.clip(metric_distance_matrix - self.clearance, 0, None)

        # Generate the costmap using exponential decay
        cost_matrix = self.max_cost * np.exp(-self.decay_rate * adjusted_dist)
        cost_matrix = np.maximum(cost_matrix, self.min_cost)
        cost_matrix = np.clip(cost_matrix, -127, 127).astype(np.int8)

        # Create and publish the costmap
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = occgrid_msg.header.frame_id
        costmap_msg.info = occgrid_msg.info
        costmap_msg.data = cost_matrix.flatten().tolist()
        self.costmap_publisher.publish(costmap_msg)

def main(args=None):
    rclpy.init(args=args)
    costmap_node = Costmap()
    try:
        rclpy.spin(costmap_node)
    except KeyboardInterrupt:
        pass
    finally:
        costmap_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
