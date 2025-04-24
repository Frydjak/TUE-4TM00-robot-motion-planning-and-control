#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import TransformStamped

import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros

import numpy as np
import time 

from group11_tue4tm00_assignment2 import group11_path_planner

class CombinedSearchBasedPathPlanner(Node):
    
    def __init__(self):
        super().__init__('path_planner', 
                         allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # Node Parameters
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_a = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Maximum cost threshold for traversable cells
        self.max_cost = self.get_parameter('max_cost').value  # e.g., 100

        # Movement threshold before re-planning (meters)
        self.min_move_threshold = self.get_parameter('min_move_threshold').value

        # Keep track of the last pose used for path planning
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_initialized = False  # Add this line to define the attribute

        # Subscriptions
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            'pose',
            self.pose_callback,
            qos_profile=1
        )
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            'goal',
            self.goal_callback,
            qos_profile=1
        )
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=1
        )

        # Map subscriber
        map_qos_profile = QoSProfile(depth=1)
        map_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            qos_profile=map_qos_profile
        )

        # Costmap subscriber
        costmap_qos_profile = QoSProfile(depth=1)
        costmap_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            'costmap',
            self.costmap_callback,
            qos_profile=costmap_qos_profile
        )

        # Publisher for nav_msgs/Path
        path_qos_profile = QoSProfile(depth=1)
        path_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(Path, 'path', qos_profile=path_qos_profile)

        # Message placeholders
        self.pose_msg = None
        self.goal_msg = None
        self.scan_msg = None
        self.map_msg = None
        self.costmap_msg = None
        self.costmap_initialized = False  # Flag to track first costmap availability
        self.goal_initialized = False

        # TF buffer/listener (if needed for TF transforms)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Get rate
        self.rate = self.get_parameter('rate').value

        # Timer
        self.create_timer(1.0 / self.rate, self.timer_callback)

    def pose_callback(self, msg):
        """Callback function for the 'pose' topic."""
        self.pose_msg = msg
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([
            msg.pose.orientation.x, 
            msg.pose.orientation.y, 
            msg.pose.orientation.z, 
            msg.pose.orientation.w
        ])[2]

        # Recompute the path whenever the pose updates
        # If this is the first pose callback, always plan once.
        if not self.last_initialized and self.costmap_initialized and self.goal_initialized:
            self.last_x = self.pose_x
            self.last_y = self.pose_y
            self.last_initialized = True
            self.compute_and_publish_path()
            return

        # Calculate how far the robot has moved since last plan
        dx = abs(self.pose_x - self.last_x)
        dy = abs(self.pose_y - self.last_y)
        distance_moved = np.hypot(dx, dy)

        # If movement exceeds the threshold, re-plan
        if distance_moved >= self.min_move_threshold:
            self.compute_and_publish_path()
            self.last_x = self.pose_x
            self.last_y = self.pose_y

    def goal_callback(self, msg):
        """Callback function for the 'goal' topic."""
        self.goal_msg = msg
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        if not self.goal_initialized:
            self.goal_initialized = True
            self.get_logger().info("Goal received.")
    
    def scan_callback(self, msg):
        """Callback for the 'scan' topic."""
        self.scan_msg = msg

    def map_callback(self, msg):
        """Callback for the 'map' topic."""
        self.map_msg = msg

    def costmap_callback(self, msg):
        """Callback for the 'costmap' topic."""
        self.costmap_msg = msg
        # Trigger initial path computation if costmap becomes available
        if not self.costmap_initialized:
            self.costmap_initialized = True
            self.get_logger().info("First costmap received.")

    def timer_callback(self):
        """
        Called at 'self.rate' Hz. 
        If additional checks are needed periodically, add them here.
        """
        # This callback is currently empty because path re-planning
        # is triggered on pose update or newly received costmap/goal.

    def compute_and_publish_path(self):
        """Computes and publishes the minimal-cost path using the costmap."""
        if not self._is_valid_data():
            return
        
        self.get_logger().info("Computing minimal-cost path...")

        # Convert robot (pose) and goal to NumPy arrays
        start_position = np.asarray([self.pose_x, self.pose_y])
        goal_position  = np.asarray([self.goal_x, self.goal_y])
        
        # Extract costmap info
        costmap_origin = np.asarray([
            self.costmap_msg.info.origin.position.x,
            self.costmap_msg.info.origin.position.y
        ])
        costmap_resolution = self.costmap_msg.info.resolution
        
        # Reshape costmap data into a 2D matrix
        costmap_matrix = np.array(self.costmap_msg.data).reshape(
            self.costmap_msg.info.height,
            self.costmap_msg.info.width
        )
        costmap_matrix = np.float64(costmap_matrix)
        
        # Any cell >= max_cost is considered obstacle
        costmap_matrix[costmap_matrix >= self.max_cost] = -1

        # Convert start & goal from world to grid indices
        start_cell = group11_path_planner.world_to_grid(
            start_position,
            origin=costmap_origin,
            resolution=costmap_resolution
        )[0]
        goal_cell = group11_path_planner.world_to_grid(
            goal_position,
            origin=costmap_origin,
            resolution=costmap_resolution
        )[0]

        # Compute the path in grid coordinates (e.g. A* or BFS)
        path_grid = group11_path_planner.shortest_path_networkx(
            costmap_matrix,
            start_cell,
            goal_cell,
            diagonal_connectivity=True
        )

        # Convert the grid path back to world coordinates
        path_world = group11_path_planner.grid_to_world(
            path_grid,
            costmap_origin,
            costmap_resolution
        )

        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        if path_world.size > 0:
            # Add the current robot pose as start
            start_pose_stamped = PoseStamped()
            start_pose_stamped.header = path_msg.header
            start_pose_stamped.pose = self.pose_msg.pose
            path_msg.poses.append(start_pose_stamped)

            # Intermediate waypoints
            for waypoint in path_world:
                waypoint_pose = PoseStamped()
                waypoint_pose.header = path_msg.header
                waypoint_pose.pose.position.x = waypoint[0]
                waypoint_pose.pose.position.y = waypoint[1]
                path_msg.poses.append(waypoint_pose)

            # Add the goal pose
            goal_pose_stamped = PoseStamped()
            goal_pose_stamped.header = path_msg.header
            goal_pose_stamped.pose = self.goal_msg.pose
            path_msg.poses.append(goal_pose_stamped)

            self.path_publisher.publish(path_msg)
            self.get_logger().info("Path published!")
        else:
            self.get_logger().warn("No valid path found.")

    def _is_valid_data(self):
        """
        Simple helper to check if pose, goal, and costmap are initialized.
        You can add more robust checks here if needed.
        """
        if self.costmap_msg is None:
            self.get_logger().warn("No costmap received yet.")
            return False
        if not self.costmap_msg.data:
            self.get_logger().warn("Costmap data is empty.")
            return False
        if self.pose_msg is None:
            self.get_logger().warn("Robot pose not received.")
            return False
        if self.goal_msg is None:
            self.get_logger().warn("Goal pose not received.")
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    path_planner_node = CombinedSearchBasedPathPlanner()
    try: 
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        path_planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
