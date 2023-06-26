#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.sub_scan = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 1000)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1000)
        # TODO: set PID gains
        self.kp = float(input("Please enter a value for Kp : "))
        self.kd = float(input("Please enter a value for Kd : "))
        self.ki = float(input("Please enter a value for Ki : "))

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.desired_distance = 1.2
        self.L = 1 # lookahead distance

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        #TODO: implement
        
        i = int((angle - range_data.angle_min) / range_data.angle_increment)
        if np.isnan(range_data.ranges[i]) or np.isinf(range_data.ranges[i]):
            return 0.0
        return range_data.ranges[i]

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        # 1. Obtain two laser scans (distances) a and b

        a = self.get_range(range_data, np.radians(50.0))
        b = self.get_range(range_data, np.radians(90.0))
        
        # 2. Use the distances a and b to calculate the angle alpha between the car's x-axis and the right wall

        theta = np.radians(40.0)
        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))

        # 3. Use alpha to find the current distance D_t to the car, and then alpha and D_t to find the estimated future distance D_t_1 to the wall

        D_t = b * np.cos(alpha)
        D_t_1 = D_t + self.L * np.sin(alpha)

        self.prev_error = self.error
        error = dist - D_t_1


        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        del_time = 0.1
        self.integral += self.prev_error * del_time
        derivative = (error - self.prev_error) / del_time
        angle = self.kp * error + self.ki * self.integral + self.kd * derivative

        # TODO: Use kp, ki & kd to implement a PID controller
        angle = -(self.kp * error + self.ki * self.integral + self.kd * derivative)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle

        if 0 < np.degrees(abs(angle)) < 10:
            drive_msg.drive.speed = 1.5
        elif 10 < np.degrees(abs(angle)) < 20:
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5
        
        # TODO: fill in drive message and publish
        self.pub_drive.publish(drive_msg)
        self.prev_error = error
    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg, self.desired_distance) # TODO: replace with error calculated by get_error()
        velocity = 1.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    lab3_wall_follow = WallFollow()
    rclpy.spin(lab3_wall_follow)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lab3_wall_follow.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
