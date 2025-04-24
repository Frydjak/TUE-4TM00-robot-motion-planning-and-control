#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.qos
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import math
import time

class Odometry(Node):
    
    def __init__(self):
        super().__init__('odometry',
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True)
        
        # Initialize parameters
        self.rate = self.get_parameter('rate').value

        # Robot pose (x, y, yaw in radians)
        self.pose_x = 0.0  
        self.pose_y = 0.0
        self.pose_a = 0.0

        # Time tracking for integration
        self.last_integration_time = self.get_clock().now()

        # Store the latest cmd_vel message
        self.cmd_vel_msg = Twist()

        # Set up subscriptions
        pose_in_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pose_in_subscriber = self.create_subscription(
            PoseStamped,
            'pose_in',
            self.pose_in_callback,
            qos_profile=pose_in_qos_profile
        )

        cmd_vel_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile=cmd_vel_qos_profile
        )

        scan_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=scan_qos_profile
        )

        map_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            qos_profile=map_qos_profile
        )

        # Publisher for integrated pose
        pose_out_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pose_out_publisher = self.create_publisher(
            PoseStamped,
            'pose_out',
            qos_profile=pose_out_qos_profile
        )

        # Timer to update pose at the given rate
        self.timer = self.create_timer(1.0 / self.rate, self.timer_update)

    def pose_in_callback(self, msg):
        """
        Updates the robot's pose when a new ground-truth pose is received.
        """
        x = msg.pose.position.x
        y = msg.pose.position.y

        quat = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quat)

        self.pose_x = x
        self.pose_y = y
        self.pose_a = yaw

    def cmd_vel_callback(self, msg):
        """
        Stores the latest velocity command for integration.
        """
        self.cmd_vel_msg = msg

    def scan_callback(self, msg):
        """
        Handles laser scan data if needed (not used in this implementation).
        """
        self.scan_msg = msg

    def map_callback(self, msg):
        """
        Handles map updates if needed (not used in this implementation).
        """
        self.map_msg = msg

    def timer_update(self):
        """
        Periodically integrates velocity to update the robot's estimated pose.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_integration_time).nanoseconds * 1e-9
        self.last_integration_time = current_time

        if dt <= 0.0 or dt > 1.0:
            return

        vx = self.cmd_vel_msg.linear.x
        vy = self.cmd_vel_msg.linear.y
        wz = self.cmd_vel_msg.angular.z

        self.pose_x += vx * dt * math.cos(self.pose_a) - vy * dt * math.sin(self.pose_a)
        self.pose_y += vx * dt * math.sin(self.pose_a) + vy * dt * math.cos(self.pose_a)
        self.pose_a += wz * dt

        self.pose_a = (self.pose_a + math.pi) % (2 * math.pi) - math.pi

        pose_out_msg = PoseStamped()
        pose_out_msg.header.stamp = current_time.to_msg()
        pose_out_msg.header.frame_id = 'odom'

        pose_out_msg.pose.position.x = self.pose_x
        pose_out_msg.pose.position.y = self.pose_y
        pose_out_msg.pose.position.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, self.pose_a)
        pose_out_msg.pose.orientation.x = quat[0]
        pose_out_msg.pose.orientation.y = quat[1]
        pose_out_msg.pose.orientation.z = quat[2]
        pose_out_msg.pose.orientation.w = quat[3]

        self.pose_out_publisher.publish(pose_out_msg)

def main(args=None):
    rclpy.init(args=args)
    odometry_node = Odometry()
    try:
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
