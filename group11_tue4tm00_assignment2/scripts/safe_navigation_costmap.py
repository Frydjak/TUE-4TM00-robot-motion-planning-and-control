#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros

import numpy as np
import random
import time 

class NavigationCostmap(Node):
    
    def __init__(self):
        super().__init__('navigation_costmap', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # If needed in your design, define your node parameters
        self.pose_x = 0.0 # robot x-position
        self.pose_y = 0.0 # robot y-position
        self.pose_a = 0.0 # robot yaw angle
        self.goal_x = 0.0 # goal x-position
        self.goal_y = 0.0 # goal y-position
        
        # If needed in your design, get a node parameter for update rate
        self.rate = 1.0 
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

        # Create a publisher for the costmap topic of type OccupancyGrid
        costmap_qos_profile = QoSProfile(depth=1)
        costmap_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_publisher = self.create_publisher(OccupancyGrid, 'costmap', qos_profile=costmap_qos_profile)
        self.costmap_msg = OccupancyGrid()

        # If needed in your design, create a buffer and listener to the /tf topic 
        # to get transformations via self.tf_buffer.lookup_transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # If need in your design, crease a timer for periodic updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

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
        #TODO: If needed, use the map topic messages in your
        self.map_msg = msg

    def timer_callback(self):
        """
        Callback function for peridic timer updates
        """
        #TODO: If needed, use the timer callbacks in your design 
        
        # For example, publish random costmap with the same settings as the map message 
        self.costmap_msg = self.map_msg
        self.costmap_msg.data = [random.randint(0, 100) for _ in range(len(self.costmap_msg.data))]
        self.costmap_publisher.publish(self.costmap_msg)


def main(args=None):
    rclpy.init(args=args)
    navigation_costmap_node = NavigationCostmap()
    try: 
        rclpy.spin(navigation_costmap_node)
    except KeyboardInterrupt:
        pass 
    # finally:   
    #     navigation_costmap_node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
