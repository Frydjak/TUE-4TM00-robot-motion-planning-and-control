#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros

import numpy as np
import time 

class SafeReactiveNavigation(Node):
    
    def __init__(self):
        super().__init__('safe_reactive_navigation', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # time.sleep(5.0) # Some Delays
        self.pose_x = 0.0 # robot x-position
        self.pose_y = 0.0 # robot y-position
        self.pose_a = 0.0 # robot yaw angle
        self.goal_x = 0.0 # goal x-position
        self.goal_y = 0.0 # goal y-position
        
        # If needed in your design, get a node parameter for update rate
        default_rate = Parameter('rate', Parameter.Type.DOUBLE, 10.0) 
        self.rate = self.get_parameter_or('rate', default_rate).value

        # If needed in your design, create a subcriber to the pose topic
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 1)

        # If needed in your design, create a subcriber to the goal topic
        self.create_subscription(PoseStamped, 'goal', self.goal_callback, 1)

        # If needed in your design, create a subcriber to the scan topic
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)

        # If needed in your design, create a publisher for the cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.cmd_vel = Twist()

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
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]


    def goal_callback(self, msg):
        """
        Callback function for the goal topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        #TODO: If needed, use the pose topic messages in your design
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
    
    def scan_callback(self, msg):
        """
        Callback function for the scan topic, handling messages of type sensor_msgs.msg.LaserScan
        """
        #TODO: If needed, use the scan topic messages in your design
        pass          

    def timer_callback(self):
        """
        Callback function for peridic timer updates
        """
        #TODO: If needed, use the timer callbacks in your design 
        
        # For example, publish the cmd_vel message to directly go to the goal while rotating around itself
        ctrl_gain = 0.1
        const_ang_vel = 1.0
        self.cmd_vel.linear.x = np.dot([np.cos(self.pose_a), np.sin(self.pose_a)], [-ctrl_gain*(self.pose_x-self.goal_x), -ctrl_gain*(self.pose_y-self.goal_y)])
        self.cmd_vel.linear.y = np.dot([-np.sin(self.pose_a), np.cos(self.pose_a)], [-ctrl_gain*(self.pose_x-self.goal_x), -ctrl_gain*(self.pose_y-self.goal_y)])
        self.cmd_vel.angular.z = const_ang_vel
        self.cmd_vel_pub.publish(self.cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    safe_reactive_navigation_node = SafeReactiveNavigation()
    rclpy.spin(safe_reactive_navigation_node)
    safe_reactive_navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
