#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscriber_lidar = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.subscriber_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        
        self.THRESHOLD = 0.4
        self.ranges = []
        self.angles = []
        self.angles_cos = []
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.speed = 0.

        self.initialized = False
        
        self.get_logger().info('Safety Node initialized')
 
    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ranges = np.array(scan_msg.ranges)
        # TODO: publish command to brake
        # Initialization: read off values from the sensor
        if self.initialized == False:
            self.angle_min = scan_msg.angle_min
            self.angle_max = scan_msg.angle_max
            self.angle_increment = scan_msg.angle_increment
            self.angles = np.linspace(self.angle_min, self.angle_max, len(self.ranges))
            self.angles_cos = np.cos(self.angles)
            self.initialized = True
        self.calculate_TTC()
        # TTC Calculation
    def calculate_TTC(self):
        if self.speed < 0.1 and self.speed > -0.1:
            return
        V_x = self.angles_cos * self.speed
        TTCs = np.divide(self.ranges, V_x)
        lower_than_threshold = False
        for TTC in TTCs:
            if TTC > 0 and TTC < self.THRESHOLD:
                lower_than_threshold = True
                break
        if lower_than_threshold:

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
