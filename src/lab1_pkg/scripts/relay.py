#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

from lab1_pkg.relaynode import RelayNode

def main (args=None):
    rclpy.init(args=args)
    node = RelayNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
