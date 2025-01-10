#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
import rclpy.qos
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path

import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros

class PathPlanner(Node):
    
    def __init__(self):
        super().__init__('path_planner', allow_undeclared_parameters=True, 
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
        pose_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, # BEST_EFFORT | RELIABLE
            durability=rclpy.qos.DurabilityPolicy.VOLATILE, # TRANSIENT_LOCAL | VOLATILE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # KEEP_LAST | KEEP ALL
            depth=1, # Integer queue size
        )
        self.pose_subscriber = self.create_subscription(PoseStamped, 'pose', self.pose_callback, qos_profile=pose_qos_profile)
        self.pose_subscriber  # prevent unused variable warning
        self.pose_msg = PoseStamped()

        # If needed in your design, create a subscriber to the goal topic
        goal_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, # BEST_EFFORT | RELIABLE
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, # TRANSIENT_LOCAL | VOLATILE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # KEEP_LAST | KEEP ALL
            depth=1, # Integer queue size
        )
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, qos_profile=goal_qos_profile)
        self.goal_subscriber  # prevent unused variable warning
        self.goal_msg = PoseStamped()

        # If needed in your design, create a subscriber to the scan topic
        scan_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, # BEST_EFFORT | RELIABLE
            durability=rclpy.qos.DurabilityPolicy.VOLATILE, # TRANSIENT_LOCAL | VOLATILE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # KEEP_LAST | KEEP ALL
            depth=1, # Integer queue size
        )
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=scan_qos_profile)
        self.scan_subscriber  # prevent unused variable warning
        self.scan_msg = LaserScan()

        # If needed in your design, create a subscriber to the map topic with the QoS profile of transient_local durability
        map_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, # BEST_EFFORT | RELIABLE
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, # TRANSIENT_LOCAL | VOLATILE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # KEEP_LAST | KEEP ALL
            depth=1, # Integer queue size
        )
        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos_profile=map_qos_profile)
        self.map_msg = OccupancyGrid()

        # Create a subscriber to the costmap topic of message type nav_msgs.msg.OccupancyGrid
        costmap_qos_profile = QoSProfile(depth=1)
        costmap_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_subscriber = self.create_subscription(OccupancyGrid, 'costmap', self.costmap_callback, qos_profile=costmap_qos_profile)
        self.costmap_msg = OccupancyGrid()

        # Create a publisher for the path topic of message type nav_msgs.msg.Path
        path_qos_profile = QoSProfile(depth=1)
        path_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(Path, 'path', qos_profile=path_qos_profile)
        self.path_msg = Path()

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

    def goal_callback(self, msg):
        """
        Callback function for the goal topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        #TODO: If needed, use the pose topic messages in your design
        self.goal_msg = msg
    
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

    def timer_callback(self):
        """
        Callback function for peridic timer updates
        """
        #TODO: If needed, use the timer callbacks in your design 
        
        # For example, publish the straight path between the pose and the goal messages 
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(self.pose_msg)
        self.path_msg.poses.append(self.goal_msg)
        self.path_publisher.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlanner()
    try: 
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass 


if __name__ == '__main__':
    main()
