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
    Compute the distance from each cell in the occupancy grid to the nearest obstacle
    """
    # binary_occupancy_matrix: True = obstacle, False = unoccupied
    # distance_transform_edt computes distance to the nearest zero value.
    # We invert so that obstacles become 0 and unoccupied cells become 1.
    distance_matrix = scipy.ndimage.distance_transform_edt(1 - binary_occupancy_matrix)
    return distance_matrix

class NavigationCostmap(Node):
    
    def __init__(self):
        super().__init__('navigation_costmap', 
                         allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # Retrieve parameters (provided via safe_navigation_costmap.yaml config file)
        # Parameters for the repulsive costmap
        self.rate = self.get_parameter('rate').value
        self.min_cost = self.get_parameter('min_cost').value if self.has_parameter('min_cost') else 1.0
        self.max_cost = self.get_parameter('max_cost').value if self.has_parameter('max_cost') else 100.0
        self.decay_rate = self.get_parameter('decay_rate').value if self.has_parameter('decay_rate') else 1.0
        self.safety_margin = self.get_parameter('safety_margin').value if self.has_parameter('safety_margin') else 0.0
        self.occupancy_threshold = self.get_parameter('occupancy_threshold').value if self.has_parameter('occupancy_threshold') else 0.0

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_a = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Subscribers
        self.pose_subscriber = self.create_subscription(PoseStamped, 'pose', self.pose_callback, 1)
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 1)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        
        map_qos_profile = QoSProfile(depth=1)
        map_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, qos_profile=map_qos_profile
        )

        # Publisher
        costmap_qos_profile = QoSProfile(depth=1)
        costmap_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_publisher = self.create_publisher(OccupancyGrid, 'costmap', qos_profile=costmap_qos_profile)

        self.map_msg = None
        self.scan_msg = None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer
        self.create_timer(1.0 / self.rate, self.timer_callback)

    def pose_callback(self, msg):
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y,
                                             msg.pose.orientation.z, msg.pose.orientation.w])[2]

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

        # convert ROS Occupancy Grid to NumPy array, reszhape it into matrix of maps dimensions
        occgrid_msg = self.map_msg
        occupancy_matrix = np.array(occgrid_msg.data, dtype=np.int8).reshape(
            occgrid_msg.info.height, occgrid_msg.info.width)

        # Convert to binary occupancy: True = obstacle, False = free
        binary_occupancy_matrix = occupancy_matrix >= int(100 * self.occupancy_threshold)

        # Compute distance transform in cells
        distance_matrix = distance_transform(binary_occupancy_matrix)

        # Convert to metric distance
        metric_distance_matrix = distance_matrix * occgrid_msg.info.resolution
        
        # Apply exponential decay to get cost
        # ensure that cost does not go below min_cost
        cost_matrix = self.max_cost * np.exp(-self.decay_rate * metric_distance_matrix)
        cost_matrix = np.maximum(cost_matrix, self.min_cost)

        # Clip to int8 range (-127,127)
        # costs are in [min_cost, max_cost] = [1,100]
        cost_matrix = np.clip(cost_matrix, -127, 127)
        cost_matrix = np.int8(cost_matrix)

        # Create costmap message
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = occgrid_msg.header.frame_id
        costmap_msg.info = occgrid_msg.info
        costmap_msg.data = cost_matrix.flatten().tolist()

        # Publish the costmap
        self.costmap_publisher.publish(costmap_msg)


def main(args=None):
    rclpy.init(args=args)
    navigation_costmap_node = NavigationCostmap()
    try: 
        rclpy.spin(navigation_costmap_node)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_costmap_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
