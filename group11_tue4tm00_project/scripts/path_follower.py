#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from tf_transformations import euler_from_quaternion
import tf2_ros

class SafePathFollower(Node):
    
    def __init__(self):
        super().__init__('safe_path_follower',
                         allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # Retrieve parameters from configuration or defaults
        self.rate = self.get_parameter('rate').value
        self.look_ahead_distance = self.get_parameter('look_ahead_distance').value
        self.final_goal_tolerance = self.get_parameter('final_goal_tolerance').value
        self.corridor_radius = self.get_parameter('corridor_radius').value
        self.block_cost_threshold = self.get_parameter('block_cost_threshold').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed_gain = self.get_parameter('angular_speed_gain').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        # Robot state initialization
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_a = 0.0

        # Path and goal tracking
        self.current_path = []
        self.goal_reached = False

        # Costmap properties
        self.costmap_matrix = None
        self.costmap_origin = (0.0, 0.0)
        self.costmap_resolution = 0.05
        self.costmap_width = 0
        self.costmap_height = 0

        # Set up subscribers
        self.pose_subscriber = self.create_subscription(
            PoseStamped, 'odom_pose', self.pose_callback, 1
        )
        costmap_qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid, 'costmap', self.costmap_callback, qos_profile=costmap_qos_profile
        )
        path_qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_subscriber = self.create_subscription(
            Path, 'path', self.path_callback, qos_profile=path_qos_profile
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 1
        )

        # Set up publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        # Set up TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer for periodic control updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

    # -----------------------------
    # Callbacks for incoming messages
    # -----------------------------
    def pose_callback(self, msg):
        # Update robot's position and orientation from pose message
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        quat = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.pose_a = yaw

    def costmap_callback(self, msg):
        # Update costmap information from occupancy grid
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        raw_data = np.array(msg.data, dtype=np.float64)
        self.costmap_matrix = raw_data.reshape(msg.info.height, msg.info.width)

    def path_callback(self, msg):
        # Convert received path to list of waypoints
        if not msg.poses:
            self.get_logger().warn("Empty path received. Stopping robot.")
            self.current_path = []
            self.goal_reached = False
            return
        self.current_path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.goal_reached = False
        self.get_logger().info(f"Got path with {len(self.current_path)} points.")

    def scan_callback(self, msg):
        # Handle laser scan data if needed (currently unused)
        pass

    # -----------------------------
    # Timer-based control loop
    # -----------------------------
    def timer_callback(self):
        # Stop robot if no path or goal already reached
        if not self.current_path or self.goal_reached:
            self.stop_robot()
            return

        # Stop robot if costmap is unavailable
        if self.costmap_matrix is None:
            self.get_logger().warn("No costmap yet, stopping.")
            self.stop_robot()
            return

        # Check distance to goal
        gx, gy = self.current_path[-1]
        dist_to_goal = math.hypot(gx - self.pose_x, gy - self.pose_y)
        if dist_to_goal < self.final_goal_tolerance:
            self.stop_robot()
            self.goal_reached = True
            self.get_logger().info("Current goal reached!")
            return

        # Determine look-ahead point
        look_x, look_y = self.find_lookahead_point()

        # Check if path is blocked in costmap
        if self.is_blocked_in_costmap(look_x, look_y):
            self.get_logger().warn("Look-ahead corridor blocked. Stopping.")
            self.stop_robot()
            return

        # Publish velocity command
        cmd_vel = self.compute_cmd_vel(look_x, look_y)
        self.cmd_vel_pub.publish(cmd_vel)

    # -----------------------------
    # Helper methods
    # -----------------------------
    def find_lookahead_point(self):
        # Locate the next waypoint on the path to follow
        gx, gy = self.current_path[-1]
        dist_to_goal = math.hypot(gx - self.pose_x, gy - self.pose_y)
        if dist_to_goal < self.look_ahead_distance:
            return gx, gy

        closest_idx = 0
        min_dist_sq = float('inf')
        for i, (px, py) in enumerate(self.current_path):
            d2 = (px - self.pose_x)**2 + (py - self.pose_y)**2
            if d2 < min_dist_sq:
                min_dist_sq = d2
                closest_idx = i

        for j in range(closest_idx, len(self.current_path)):
            px, py = self.current_path[j]
            d = math.hypot(px - self.pose_x, py - self.pose_y)
            if d >= self.look_ahead_distance:
                return px, py

        return self.current_path[-1]

    def is_blocked_in_costmap(self, wx, wy):
        # Check if cost in the area around a point exceeds the threshold
        row_col = self.world_to_grid(wx, wy)
        if row_col is None:
            return True
        row, col = row_col
        r_radius = int(self.corridor_radius / self.costmap_resolution)

        row_min = max(0, row - r_radius)
        row_max = min(self.costmap_height - 1, row + r_radius)
        col_min = max(0, col - r_radius)
        col_max = min(self.costmap_width - 1, col + r_radius)

        for rr in range(row_min, row_max + 1):
            for cc in range(col_min, col_max + 1):
                if self.costmap_matrix[rr, cc] >= self.block_cost_threshold:
                    return True
        return False

    def world_to_grid(self, x, y):
        # Convert world coordinates to grid indices
        col = int((x - self.costmap_origin[0]) / self.costmap_resolution)
        row = int((y - self.costmap_origin[1]) / self.costmap_resolution)
        if row < 0 or row >= self.costmap_height or col < 0 or col >= self.costmap_width:
            return None
        return (row, col)

    def compute_cmd_vel(self, tx, ty):
        # Compute linear and angular velocities for navigation
        heading = math.atan2(ty - self.pose_y, tx - self.pose_x)
        yaw_error = self.normalize_angle(heading - self.pose_a)

        angular_z = self.angular_speed_gain * yaw_error
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)

        speed_factor = max(0.0, 1.0 - abs(yaw_error))
        linear_x = self.linear_speed * speed_factor

        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        return cmd_vel

    def stop_robot(self):
        # Publish a zero velocity command to stop the robot
        self.cmd_vel_pub.publish(Twist())

    def normalize_angle(self, angle):
        # Normalize angle to the range [-pi, pi]
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = SafePathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
