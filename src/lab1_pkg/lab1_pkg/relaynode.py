#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class RelayNode(Node):
    def __init__(self):
        super().__init__('relay')
        self.subscription_ = self.create_subscription(
                AckermannDriveStamped,
                'drive',
                self.message_callback,
                10
                
        )
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def message_callback(self, msg):
        modified_msg = AckermannDriveStamped()
        modified_msg.drive.speed = msg.drive.speed * 3
        modified_msg.drive.steering_angle = msg.drive.steering_angle * 3
        self.publisher_.publish(modified_msg)
        self.get_logger().info('Modified and published: "speed=%.2f, steering_angle=%.2f"' % (modified_msg.drive.speed, modified_msg.drive.steering_angle))
