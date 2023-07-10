#!/usr/bin/env python3

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from lab1_pkg.talkernode import TalkerNode

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()

    # Retrieve the values of parameters v and d

    v = node.get_parameter('v').value
    d = node.get_parameter('d').value
    node.get_logger().info('Parameters -v: %.2f, d: %.2f' % (v, d))

    while rclpy.ok():
        node.publish_message()
        rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()

if __name__=="__main__":
    main()
