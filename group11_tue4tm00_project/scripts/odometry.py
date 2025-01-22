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
        
        #
        # Declare/obtain parameters
        #
        self.rate = self.get_parameter('rate').value

        # These store our best estimate of the robot pose (x,y,theta).
        # They are updated at 10 Hz by integrating cmd_vel, 
        # and reset whenever a new ground-truth pose_in arrives.
        self.pose_x = 0.0  
        self.pose_y = 0.0
        self.pose_a = 0.0  # yaw in radians

        # Keep the last time we integrated so we can compute dt
        # Start it at the current node time.
        self.last_integration_time = self.get_clock().now()

        # Keep track of the latest Twist message
        self.cmd_vel_msg = Twist()  # default to zero velocity

        #
        # Subscribers
        #
        # 1) Subscribe to the low-rate (0.3 Hz) pose_in
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

        # 2) Subscribe to cmd_vel (used for dead-reckoning)
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

        # 3) Subscribe to the scan topic (if needed for any advanced logic)
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
        self.scan_msg = LaserScan()

        # 4) Subscribe to the map topic (if you need map info)
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
        self.map_msg = OccupancyGrid()

        #
        # Publishers
        #
        # Publish the integrated pose at 10 Hz to 'pose_out'
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

        #
        # Timer - run at self.rate (e.g., 10 Hz)
        #
        self.timer = self.create_timer(1.0 / self.rate, self.timer_update)

    def pose_in_callback(self, msg):
        """
        Callback for the low-rate ground-truth (or corrected) pose, e.g. at 0.3 Hz.
        When a new message arrives, reset the integrated pose to these values.
        """
        # Extract x, y, yaw from the incoming PoseStamped
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Convert quaternion to euler angles
        quat = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        roll, pitch, yaw = euler_from_quaternion(quat)

        # Override our integrated state with the "ground truth"
        self.pose_x = x
        self.pose_y = y
        self.pose_a = yaw

    def cmd_vel_callback(self, msg):
        """
        Callback for cmd_vel. We store the latest Twist so we can integrate it at 10 Hz.
        """
        self.cmd_vel_msg = msg

    def scan_callback(self, msg):
        """
        Callback for the scan topic, if needed in your design.
        """
        self.scan_msg = msg

    def map_callback(self, msg):
        """
        Callback for the map topic, if needed in your design.
        """
        self.map_msg = msg

    def timer_update(self):
        """
        Called at self.rate (e.g. 10 Hz). We integrate the latest velocity commands
        forward in time and publish an updated pose_out.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_integration_time).nanoseconds * 1e-9
        self.last_integration_time = current_time

        # If dt is extremely large or zero (e.g., on startup), skip integration
        if dt <= 0.0 or dt > 1.0:
            return

        # Grab linear and angular velocities from the last cmd_vel
        vx = self.cmd_vel_msg.linear.x      # forward velocity (m/s)
        vy = self.cmd_vel_msg.linear.y      # (usually 0 for differential drive)
        wz = self.cmd_vel_msg.angular.z     # yaw rate (rad/s)

        #
        # Simple differential-drive or unicycle integration
        #
        # If you assume cmd_vel is in the robot's local frame, 
        # the forward velocity is along the robot's heading, 
        # so you'd do:
        #
        #   self.pose_x += vx*dt*cos(self.pose_a) - vy*dt*sin(self.pose_a)
        #   self.pose_y += vx*dt*sin(self.pose_a) + vy*dt*cos(self.pose_a)
        #   self.pose_a += wz*dt
        #
        # If your robot or simulator uses a different convention, adapt accordingly.

        self.pose_x += vx * dt * math.cos(self.pose_a) - vy * dt * math.sin(self.pose_a)
        self.pose_y += vx * dt * math.sin(self.pose_a) + vy * dt * math.cos(self.pose_a)
        self.pose_a += wz * dt

        # Normalize yaw to [-pi, pi] or [0, 2*pi] if desired
        # for cleanliness in logs or for continuity
        self.pose_a = (self.pose_a + math.pi) % (2 * math.pi) - math.pi

        # Now prepare the PoseStamped to publish
        pose_out_msg = PoseStamped()
        pose_out_msg.header.stamp = current_time.to_msg()
        # Use a relevant frame, e.g. 'odom' or 'map'. Here we just say 'odom'
        pose_out_msg.header.frame_id = 'odom'

        # Fill in the position
        pose_out_msg.pose.position.x = self.pose_x
        pose_out_msg.pose.position.y = self.pose_y
        pose_out_msg.pose.position.z = 0.0

        # Convert yaw back to quaternion
        quat = quaternion_from_euler(0.0, 0.0, self.pose_a)
        pose_out_msg.pose.orientation.x = quat[0]
        pose_out_msg.pose.orientation.y = quat[1]
        pose_out_msg.pose.orientation.z = quat[2]
        pose_out_msg.pose.orientation.w = quat[3]

        # Publish
        self.pose_out_publisher.publish(pose_out_msg)

    #
    # main()
    #
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
