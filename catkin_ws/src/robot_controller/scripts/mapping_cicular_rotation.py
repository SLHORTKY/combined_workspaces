#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from math import pi

class CircularMotion:
    def __init__(self):
        rospy.init_node('circular_motion_node', anonymous=True)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/sim_p3at/cmd_vel', Twist, queue_size=10)
        
        # Subscriber for odometry to check orientation
        self.odom_sub = rospy.Subscriber('/sim_p3at/odom', Odometry, self.odom_callback)
        
        # Initial orientation flag
        self.orientation_set = False
        
        # Desired orientation (90 degrees rotated clockwise in radians)
        self.target_yaw = -pi / 2
        
        # Motion parameters
        self.linear_velocity = 0.5  # meters per second
        self.radius = 1.5           # meters (adjust for a wider or tighter circle)
        self.angular_velocity = self.linear_velocity / self.radius
        
        # Velocity command
        self.twist = Twist()
        
        rospy.loginfo("Circular Motion Node Started")
    
    def odom_callback(self, msg):
        if not self.orientation_set:
            # Get current yaw from odometry
            orientation_q = msg.pose.pose.orientation
            _, _, yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
            
            # Calculate the error in yaw
            yaw_error = self.target_yaw - yaw
            
            # Correct orientation if needed
            if abs(yaw_error) > 0.01:  # Small threshold for precision
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.5 * yaw_error
                self.cmd_vel_pub.publish(self.twist)
            else:
                self.orientation_set = True
                rospy.loginfo("Orientation aligned. Starting circular motion.")
        else:
            # Once orientation is set, move in a circle
            self.twist.linear.x = self.linear_velocity
            self.twist.angular.z = self.angular_velocity
            self.cmd_vel_pub.publish(self.twist)
    
    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Convert quaternion to Euler angles."""
        from math import atan2, asin
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)
        
        return roll, pitch, yaw

    def stop_robot(self):
        """Stop the robot when shutting down."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Robot stopped.")

if __name__ == "__main__":
    try:
        node = CircularMotion()
        rospy.on_shutdown(node.stop_robot)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

