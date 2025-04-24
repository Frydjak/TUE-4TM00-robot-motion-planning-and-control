#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import TransformStamped

import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs

import numpy as np
import time

from group11_tue4tm00_project import group11_path_planner

class PathPlanner(Node):
    def __init__(self):
        super().__init__(
            'path_planner',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # Initialize parameters and variables
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_a = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        self.max_cost = self.get_parameter('max_cost').value
        self.rate = self.get_parameter('rate').value

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_initialized = False

        # Subscriptions for various topics
        self.pose_subscriber = self.create_subscription(PoseStamped, 'odom_pose', self.pose_callback, qos_profile=1)

        goal_qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, qos_profile=goal_qos_profile)

        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=1)

        map_qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos_profile=map_qos_profile)

        costmap_qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_subscriber = self.create_subscription(OccupancyGrid, 'costmap', self.costmap_callback, qos_profile=costmap_qos_profile)

        # Path publisher
        path_qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_publisher = self.create_publisher(Path, 'path', qos_profile=path_qos_profile)

        # Variables for storing received messages
        self.pose_msg = None
        self.goal_msg = None
        self.scan_msg = None
        self.map_msg = None
        self.costmap_msg = None
        self.costmap_initialized = False
        self.goal_initialized = False
        self.pose_initialized = False

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0 / self.rate, self.timer_callback)

    # Handle incoming robot pose
    def pose_callback(self, msg):
        self.pose_msg = msg
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])[2]

    # Handle goal updates
    def goal_callback(self, msg):
        self.goal_msg = msg
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        if not self.goal_initialized:
            self.goal_initialized = True
            self.get_logger().info(f"Goal initialized at ({self.goal_x}, {self.goal_y}).")
        else:
            self.get_logger().info(f"New goal received at ({self.goal_x}, {self.goal_y}).")

    # Placeholder for laser scan data
    def scan_callback(self, msg):
        self.scan_msg = msg

    # Placeholder for map updates
    def map_callback(self, msg):
        self.map_msg = msg

    # Handle costmap updates
    def costmap_callback(self, msg):
        self.costmap_msg = msg
        if not self.costmap_initialized:
            self.costmap_initialized = True
            self.get_logger().info(f"First costmap received in frame: {msg.header.frame_id}")

    # Timer loop for checking and planning
    def timer_callback(self):
        if not self._is_valid_data():
            return

        # Wait for the robot to initialize
        if not self.pose_initialized:
            if abs(self.pose_x) < 1e-6 and abs(self.pose_y) < 1e-6:
                self.get_logger().info("Waiting for the robot to initialize...")
                return
            else:
                self.pose_initialized = True
                self.get_logger().info(f"Robot initialized at ({self.pose_x}, {self.pose_y}).")

        # Wait for a goal to be set
        if not self.goal_initialized:
            self.get_logger().info("Waiting for the goal to be set...")
            return

        # Recalculate path if the goal has changed
        goal_changed = (self.goal_x != self.last_x or self.goal_y != self.last_y)
        if not self.last_initialized or goal_changed:
            self.last_x = self.goal_x
            self.last_y = self.goal_y
            self.last_initialized = True
            self.get_logger().info("Computing path for new goal or updated pose...")
            self.compute_and_publish_path()

    # Plan and publish the path
    def compute_and_publish_path(self):
        if not self._is_valid_data():
            return

        costmap_header = self.costmap_msg.header
        costmap_origin = np.asarray([
            self.costmap_msg.info.origin.position.x,
            self.costmap_msg.info.origin.position.y
        ])
        costmap_resolution = self.costmap_msg.info.resolution

        self.get_logger().info(
            f"Computing path from ({self.pose_x:.2f}, {self.pose_y:.2f}) "
            f"to ({self.goal_x:.2f}, {self.goal_y:.2f}) in frame '{costmap_header.frame_id}'..."
        )

        try:
            costmap_matrix = np.array(self.costmap_msg.data).reshape(
                self.costmap_msg.info.height,
                self.costmap_msg.info.width
            ).astype(np.float64)

            costmap_matrix[costmap_matrix >= self.max_cost] = -1

            # Convert pose and goal to grid coordinates
            start_cell = group11_path_planner.world_to_grid(
                [self.pose_x, self.pose_y],
                origin=costmap_origin,
                resolution=costmap_resolution
            )[0]
            goal_cell = group11_path_planner.world_to_grid(
                [self.goal_x, self.goal_y],
                origin=costmap_origin,
                resolution=costmap_resolution
            )[0]

            self.get_logger().info(f"Start cell: {start_cell[0]}, {start_cell[1]}, Goal cell: {goal_cell[0]}, {goal_cell[1]}")

            path_grid = group11_path_planner.shortest_path_networkx(
                costmap_matrix,
                start_cell,
                goal_cell,
                diagonal_connectivity=True
            )

            path_world = group11_path_planner.grid_to_world(
                path_grid,
                costmap_origin,
                costmap_resolution
            )

            if path_world.size > 0:
                self._publish_path(path_world, costmap_header.frame_id)
            else:
                self.get_logger().warn("No valid path found.")
        except Exception as e:
            self.get_logger().error(f"Path computation failed: {e}")

    def _publish_path(self, path_world, frame_id):
        """Publish the path as a `nav_msgs/Path` message."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id

        for waypoint in path_world:
            pose_st = PoseStamped()
            pose_st.header.frame_id = frame_id
            pose_st.pose.position.x = waypoint[0]
            pose_st.pose.position.y = waypoint[1]
            path_msg.poses.append(pose_st)

        self.path_publisher.publish(path_msg)
        self.get_logger().info(f"Path published in '{frame_id}' frame!")

    def _is_valid_data(self):
        """Make sure all required data (costmap, pose, goal) is available."""
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
    planner_node = PathPlanner()
    try:
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
