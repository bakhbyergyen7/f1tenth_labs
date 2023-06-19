#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
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

        self.speed = 0.0
        self.TTC_start = 0.2 # this is the threshold in (s)

        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 1)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 1)

        self.ack_msg = AckermannDriveStamped()
        self.ack_msg.drive = AckermannDrive()
        self.rate = self.create_rate(500)

        self.get_logger().info('Safety Node initialized')
 
    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Initializing and populating ranges & making sure if they are NaN or inf
        ranges = np.array([], dtype = np.float32)
        ranges = np.array(scan_msg.ranges)
        rangeNaN = np.isnan(ranges)
        rangeinf = np.isinf(ranges)

        TTC = np.array([], dtype=np.float32)
        
        for i in range(len(ranges)):
            if rangeNaN[i] == True:
                ranges[i] = 0
                TTC = np.append(TTC, [101])
            elif rangeinf[i] == True:
                ranges[i] = 1
                TTC = np.append(TTC, [102])
            else:
                #TODO: calculate TTC
                theta = scan_msg.angle_min + i * scan_msg.angle_increment
                rel_vel = self.speed * np.cos(theta)

                if max([rel_vel, 0]) != 0:
                    ## car is running close to obstacle
                    TTC = np.append(TTC, [ranges[i] / max([rel_vel, 0])])

                else:
                    ## car is running away from obstacle
                    TTC = np.append(TTC, [100])

        self.TTC_decision(TTC)

    def TTC_decision(self, TTC):
        # TODO: Publish brake message
        if np.min(TTC) < self.TTC_start:
            while rclpy.ok():
                self.ack_msg.drive.speed = 0.0
                self.pub_drive.publish(self.ack_msg)
        else:
            self.ack_msg.drive.speed = 4.0
            self.pub_drive.publish(self.ack_msg)
            self.rate.sleep()
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
