#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

import numpy as np
from tf_transformations import euler_from_quaternion

class SafeReactiveNavigation(Node):

    def __init__(self):
        super().__init__('safe_reactive_navigation', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.pose_x = 0.0  # Robot x-position
        self.pose_y = 0.0  # Robot y-position
        self.pose_a = 0.0  # Robot yaw angle
        self.goal_x = 0.0  # Goal x-position
        self.goal_y = 0.0  # Goal y-position

        # Parameters
        self.kappa = 1.0              # Attractive force gain
        self.eta = 0.25               # Repulsive force scaling factor
        self.d0 = 2.0                 # Obstacle influence distance (meters)
        self.max_linear_speed = 0.5   # Maximum linear speed
        self.max_angular_speed = 1.0  # Maximum angular speed

        # Initialize closest obstacle data
        self.closest_obstacle = None
        self.min_distance = float('inf')

        # Get a node parameter for update rate
        default_rate = Parameter('rate', Parameter.Type.DOUBLE, 10.0)
        self.rate = self.get_parameter_or('rate', default_rate).value

        # Subscribers
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 1)
        self.create_subscription(PoseStamped, 'goal', self.goal_callback, 1)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.cmd_vel = Twist()

        # Timer for periodic updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

    def pose_callback(self, msg):
        """
        Callback function for the pose topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, self.pose_a = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

    def goal_callback(self, msg):
        """
        Callback function for the goal topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    def scan_callback(self, msg):
        """
        Callback function for the scan topic, handling messages of type sensor_msgs.msg.LaserScan
        """
        x_r = self.pose_x
        y_r = self.pose_y
        theta_r = self.pose_a

        min_distance = float('inf')
        closest_obstacle = None

        for i, r in enumerate(msg.ranges):
            if r >= msg.range_min and r <= msg.range_max and np.isfinite(r):
                if r < min_distance:
                    min_distance = r
                    angle = msg.angle_min + i * msg.angle_increment
                    # Obstacle position in world frame
                    x_obs_world = x_r + r * np.cos(theta_r + angle)
                    y_obs_world = y_r + r * np.sin(theta_r + angle)
                    closest_obstacle = (x_obs_world, y_obs_world)

        # Store the closest obstacle position and distance
        self.closest_obstacle = closest_obstacle
        self.min_distance = min_distance

    def timer_callback(self):
        """
        Callback function for periodic timer updates
        """
        x_r = self.pose_x
        y_r = self.pose_y
        theta_r = self.pose_a
        x_g = self.goal_x
        y_g = self.goal_y

        # Compute distance to goal
        dist_to_goal = np.hypot(x_g - x_r, y_g - y_r)

        # If close to the goal, stop
        if dist_to_goal < 0.4:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel)
            return

        # Attractive Force
        force_attr_x = -self.kappa * (x_r - x_g)
        force_attr_y = -self.kappa * (y_r - y_g)

        # Repulsive Force
        force_rep_x = 0.0
        force_rep_y = 0.0

        eta = self.eta
        d0 = self.d0

        if self.closest_obstacle is not None and self.min_distance <= d0 and self.min_distance > 0.001:
            x_obs = self.closest_obstacle[0]
            y_obs = self.closest_obstacle[1]
            dx = x_r - x_obs
            dy = y_r - y_obs
            dist = self.min_distance
            # Compute repulsive force
            scalar = eta * (1.0 / dist - 1.0 / d0) / (dist ** 2)
            force_rep_x = scalar * (dx / dist)
            force_rep_y = scalar * (dy / dist)

        # Total Force
        force_total_x = force_attr_x + force_rep_x
        force_total_y = force_attr_y + force_rep_y

        # Compute desired heading
        theta_desired = np.arctan2(force_total_y, force_total_x)

        # Compute angle difference
        delta_theta = theta_desired - theta_r
        # Normalize delta_theta to [-pi, pi]
        delta_theta = np.arctan2(np.sin(delta_theta), np.cos(delta_theta))

        # Compute linear and angular velocities
        v = np.hypot(force_total_x, force_total_y)
        max_v = self.max_linear_speed
        max_w = self.max_angular_speed

        # Scale linear velocity based on the angle difference
        cmd_v = v * np.cos(delta_theta)
        # Limit linear velocity
        cmd_v = np.clip(cmd_v, 0.0, max_v)  # Prevent backward motion

        # Angular velocity proportional to angle difference
        angular_gain = 1.5  # Gain for angular speed
        cmd_w = angular_gain * delta_theta
        # Limit angular velocity
        cmd_w = np.clip(cmd_w, -max_w, max_w)

        # Publish cmd_vel
        self.cmd_vel.linear.x = cmd_v
        self.cmd_vel.angular.z = cmd_w
        self.cmd_vel_pub.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    safe_reactive_navigation_node = SafeReactiveNavigation()
    rclpy.spin(safe_reactive_navigation_node)
    safe_reactive_navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
