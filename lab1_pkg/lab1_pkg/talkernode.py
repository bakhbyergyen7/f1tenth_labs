#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class TalkerNode(Node):

    def __init__(self):
        super().__init__("talker")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.declare_parameter('v', 0.0)
        self.declare_parameter('d', 0.0)

    def publish_message(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.get_parameter('v').value
        msg.drive.steering_angle = self.get_parameter('d').value
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "v = %.2f, d=%.2f"' % (msg.drive.speed, msg.drive.steering_angle))
