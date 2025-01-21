#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class RectangularPathDriver:
    def __init__(self):
        rospy.init_node('path_tracer_node')

        self.cmd_vel_pub = rospy.Publisher('/sim_p3at/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/sim_p3at/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)

        self.points = [
            (-5.5, -5.35),
            (7, -5.35),
            (7, 5.35),
            (-5.5, 5.35)
        ]

        self.current_index = 1
        self.tolerance = 0.05
        self.linear_speed = 0.8
        self.angular_speed = 0.2

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = euler_from_quaternion(orientation_list)

    def move_straight(self, target_x, target_y):
        while not rospy.is_shutdown():
            x_distance = math.sqrt(abs(target_x - self.current_x)**2)
            y_distance = math.sqrt(abs(target_y - self.current_y)**2)

            if x_distance < self.tolerance or y_distance < self.tolerance :
                break

            rospy.loginfo(f"{self.current_x} {self.current_y}")
            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        self.stop_robot()

    def rotate_90_degrees(self):
    # Calculate the target yaw (normalized to [0, 2π])
        target_yaw = (self.current_yaw + math.pi / 2) % (2 * math.pi)

        # Proportional gain for angular velocity control
        kp = 1.0  # Tune this value for responsiveness vs. overshoot

        while not rospy.is_shutdown():
            # Compute the yaw difference and normalize to [-π, π]
            yaw_diff = target_yaw - self.current_yaw
            yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi

            if abs(yaw_diff) < 0.001:  # Stop when within 0.01 radians (~0.57 degrees)
                break

            # Proportional control for smoother and more precise rotations
            twist = Twist()
            twist.angular.z = max(0.05, min(kp * abs(yaw_diff), self.angular_speed)) * (1 if yaw_diff > 0 else -1)
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        self.stop_robot()



    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def drive_path(self):
        while not rospy.is_shutdown():
            next_index = (self.current_index + 1) % len(self.points)
            target_x, target_y = self.points[next_index]

            self.move_straight(target_x, target_y)
            self.rotate_90_degrees()

            self.current_index = next_index

if __name__ == '__main__':
    try:
        driver = RectangularPathDriver()
        driver.drive_path()
    except rospy.ROSInterruptException:
        pass

