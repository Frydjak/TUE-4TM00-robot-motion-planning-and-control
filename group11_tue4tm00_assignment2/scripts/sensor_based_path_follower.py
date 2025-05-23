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

class SafePathFollower(Node):
    
    def __init__(self):
        super().__init__('safe_path_follower', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # If needed in your design, define your node parameters
        self.pose_x = 0.0 # robot x-position
        self.pose_y = 0.0 # robot y-position
        self.pose_a = 0.0 # robot yaw angle
        self.goal_x = 0.0 # goal x-position
        self.goal_y = 0.0 # goal y-position
        
        # If needed in your design, get a node parameter for update rate
        self.rate = 10.0 
        rate = self.get_parameter('rate').value
        self.rate = rate if rate is not None else self.rate 
        
        # If needed in your design, create a subscriber to the pose topic
        self.pose_subscriber = self.create_subscription(PoseStamped, 'pose', self.pose_callback, 1)
        self.pose_msg = PoseStamped()

        # If needed in your design, create a subscriber to the goal topic
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 1)
        self.goal_msg = PoseStamped()

        # If needed in your design, create a subscriber to the scan topic
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.scan_msg = LaserScan()

        # If needed in your design, create a subscriber to the map topic with the QoS profile of transient_local durability
        map_qos_profile = QoSProfile(depth=1)
        map_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos_profile=map_qos_profile)
        self.map_msg = OccupancyGrid()

        # If needed in your design, create a subscriber to the costmap topic of message type nav_msgs.msg.OccupancyGrid
        costmap_qos_profile = QoSProfile(depth=1)
        costmap_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_subscriber = self.create_subscription(OccupancyGrid, 'costmap', self.costmap_callback, qos_profile=costmap_qos_profile)
        self.costmap_msg = OccupancyGrid()

        # If needed in your design, create a subscriber to the path topic of message type nav_msgs.msg.Path
        path_qos_profile = QoSProfile(depth=1)
        path_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_subscriber = self.create_subscription(Path, 'path', self.path_callback, qos_profile=path_qos_profile)
        self.path_msg = Path()

        # Create a publisher for the cmd_vel topic of message type geometry_msgs.msg.Twist
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.cmd_vel_msg = Twist()

        # If needed in your design, create a buffer and listener to the /tf topic 
        # to get transformations via self.tf_buffer.lookup_transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # If need in your design, crease a timer for periodic updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

        self.velocity_Kp = self.get_parameter('velocity_Kp').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value

    def pose_callback(self, msg):
        """
        Callback function for the pose topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        #TODO: If needed, use the pose topic messages in your design
        self.pose_msg = msg
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]

    def goal_callback(self, msg):
        """
        Callback function for the goal topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        #TODO: If needed, use the pose topic messages in your design
        self.goal_msg = msg
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
    
    def scan_callback(self, msg):
        """
        Callback function for the scan topic, handling messages of type sensor_msgs.msg.LaserScan
        """
        #TODO: If needed, use the scan topic messages in your design 
        self.scan_msg = msg

    def map_callback(self, msg):
        """
        Callback function for the map topic, handling messages of type nav_msgs.msg.OccupancyGrid
        """
        #TODO: If needed, use the map topic messages in your design
        self.map_msg = msg

    def costmap_callback(self, msg):
        """
        Callback function for the costmap topic, handling messages of type nav_msgs.msg.OccupancyGrid
        """
        #TODO: If needed, use the costmap topic messages in your design
        self.costmap_msg = msg  

    def path_callback(self, msg):
        """
        Callback function for the path topic, handling messages of type nav_msgs.msg.Path
        """
        #TODO: If needed, use the path topic messages in your design
        self.path_msg = msg            

    def timer_callback(self):
        """
        Callback function for peridic timer updates
        """
        #TODO: If needed, use the timer callbacks in your design

        if self.path_msg.poses:
            # Get the current position of the robot
            current_position = np.array([self.pose_x, self.pose_y])

            # Get the first waypoint (if any)
            current_waypoint = np.array([self.path_msg.poses[0].pose.position.x, 
                                        self.path_msg.poses[0].pose.position.y])

            # Calculate the distance to the current waypoint
            distance_to_waypoint = np.linalg.norm(current_position - current_waypoint)

            # Set a threshold for reaching the waypoint (e.g., 0.1 meters)
            waypoint_threshold = self.waypoint_threshold
            velocity_Kp = self.velocity_Kp

            if distance_to_waypoint < waypoint_threshold:
                # If we reached the current waypoint, remove it from the path
                self.path_msg.poses.pop(0)  # Remove the first waypoint
                
                # If no more waypoints are left, stop the robot (reach goal)
                if len(self.path_msg.poses) == 0:
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_msg.linear.y = 0.0
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    self.get_logger().info("Goal reached, stopping robot.")
                    return
            
                # Compute the direction to the next waypoint in the global frame
            direction_global = current_waypoint - current_position

            # Transform the direction to the robot's local frame
            angle_robot = self.pose_a  # Robot's orientation (yaw) in radians
            rotation_matrix = np.array([
                [np.cos(angle_robot), np.sin(angle_robot)],
                [-np.sin(angle_robot), np.cos(angle_robot)]
            ])
            direction_local = np.dot(rotation_matrix, direction_global)

            # Log the distance in the global frame
            self.get_logger().info(f"Distance to waypoint: dx={direction_global[0]}, dy={direction_global[1]}")

            # Create Twist message to command robot movement
            self.cmd_vel_msg.linear.x = direction_local[0] * velocity_Kp  # Forward speed (scaled)
            self.cmd_vel_msg.linear.y = direction_local[1] * velocity_Kp  # Sideways speed (scaled)

            # Send the command to move the robot
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

            self.get_logger().info(f"Moving towards waypoint {current_waypoint}")
        else:
            self.get_logger().info("No path received yet.")

def main(args=None):
    rclpy.init(args=args)
    safe_path_follower_node = SafePathFollower()
    try: 
        rclpy.spin(safe_path_follower_node)
    except KeyboardInterrupt:
        pass 
    # finally:   
    #     navigation_costmap_node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
