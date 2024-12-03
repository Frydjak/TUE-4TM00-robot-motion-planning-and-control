#!/usr/bin/env python3

import math
import numpy as np
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


class SafeTwistTeleop2D(Node):
    
    def __init__(self):
        super().__init__('safe_twist_teleop_2D', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        
        # If needed in your design, get a node parameter for update rate
        default_rate = Parameter('rate', Parameter.Type.DOUBLE, 10.0) 
        self.rate = self.get_parameter_or('rate', default_rate).value
                
        # If needed in your design, create a subscriber to the input cmd_vel topic
        self.create_subscription(Twist, 'cmd_vel_in', self.cmd_vel_in_callback, 1)

        # If needed in your design, create a subcriber to the scan topic
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)

        # If needed in your design, create a subcriber to the pose topic
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 1)

        # If needed in your design, create a publisher for the output cmd_vel topic
        self.cmd_vel_out_pub = self.create_publisher(Twist, 'cmd_vel_out', 1)
        self.cmd_vel_out = Twist()

        # If needed in your design, create a buffer and listener to the /tf topic 
        # to get transformations via self.tf_buffer.lookup_transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # If need in your design, crease a timer for periodic updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

        
        # Existing parameter declarations
        self.declare_parameter('d0', 5.0)  # Influence distance (meters)
        self.declare_parameter('eta', 0.25)  # Scaling factor for repulsive force
        self.declare_parameter('max_linear_velocity', 0.5)  # Max linear speed
        self.declare_parameter('max_angular_velocity', 1.5)  # Max angular speed
        
        # New parameters for scaling adjustments
        self.declare_parameter('k_v', 0.5)
        self.declare_parameter('k_omega', 0.9)
        
        # Retrieve parameters
        self.d0 = self.get_parameter('d0').value
        self.eta = self.get_parameter('eta').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.k_v = self.get_parameter('k_v').value
        self.k_omega = self.get_parameter('k_omega').value
        
        # Initialize repulsive force components
        self.F_rep_x = 0.0
        self.F_rep_y = 0.0

        # Initialize variables to store the latest user command
        self.v_cmd = 0.0
        self.omega_cmd = 0.0

    def scan_callback(self, msg):
        """
        Callback function for the scan topic, handling messages of type sensor_msgs.msg.LaserScan
        """
        # Parameters
        d0 = self.d0  # Influence distance
        eta = self.eta  # Scaling factor for repulsive force
        
        # Convert ranges to a NumPy array
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Filter out invalid ranges
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        valid_ranges = ranges[valid]
        valid_angles = angles[valid]
        
        # Find local minima in the valid ranges
        diff_ranges = np.diff(valid_ranges)
        sign_changes = np.diff(np.sign(diff_ranges))
        local_minima_indices = np.where(sign_changes > 0)[0] + 1  # +1 due to diff

        # Initialize repulsive force components
        F_rep_x = 0.0
        F_rep_y = 0.0

        for idx in local_minima_indices:
            r = valid_ranges[idx]
            if r < d0:
                # Compute the distance to the obstacle
                d = r
                # Compute the strength of the repulsive force
                force = eta * (1.0 / d - 1.0 / d0) / (d ** 2)
                # Compute the angle of the obstacle relative to the robot
                obs_angle = valid_angles[idx]
                # Compute the components of the repulsive force
                F_rep_x += force * math.cos(obs_angle)
                F_rep_y += force * math.sin(obs_angle)

        # Store the total repulsive force
        self.F_rep_x = F_rep_x
        self.F_rep_y = F_rep_y

    def pose_callback(self, msg):
        """
        Callback function for the pose topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        #TODO: If needed, use the pose topic messages in your design
        pass

    def cmd_vel_in_callback(self, msg):
        """
        Callback function for the input cmd_vel topic, handling messages of type geometry_msgs.msg.LaserScan
        """
        # Store the latest user command velocities
        self.v_cmd = msg.linear.x
        self.omega_cmd = msg.angular.z              
        

    def timer_callback(self):
        """
        Callback function for periodic timer updates
        """
        # Parameters for scaling adjustments
        k_v = self.k_v
        k_omega = self.k_omega

        # Compute adjustments based on the latest sensor data
        v_adj = -k_v * self.F_rep_x
        omega_adj = -k_omega * self.F_rep_y

        # Combine the user's command with the repulsive adjustments
        v_safe = self.v_cmd + v_adj
        omega_safe = self.omega_cmd + omega_adj

        # Limit the velocities to acceptable ranges
        v_safe = max(min(v_safe, self.max_linear_velocity), -self.max_linear_velocity)
        omega_safe = max(min(omega_safe, self.max_angular_velocity), -self.max_angular_velocity)

        # Create and publish the safe command
        self.cmd_vel_out.linear.x = v_safe
        self.cmd_vel_out.angular.z = omega_safe
        self.cmd_vel_out_pub.publish(self.cmd_vel_out)


def main(args=None):
    rclpy.init(args=args)
    safe_twist_teleop_2d_node = SafeTwistTeleop2D()
    rclpy.spin(safe_twist_teleop_2d_node)
    safe_twist_teleop_2d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()