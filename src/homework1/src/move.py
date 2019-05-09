#!/usr/bin/env python


# Every python controller needs these lines
import rospy

# The velocity command message
from geometry_msgs.msg import Twist

# The laser scan message
from sensor_msgs.msg import LaserScan


# This is called every time we get a LaserScan message from ROS.
def laser_callback(msg):
	print msg.ranges[0]

	# Drive forward at a given speed.  The robot points up the x-axis.
	command = Twist()
	command.linear.x = 0.1
	command.linear.y = 0.0
	command.linear.z = 0.0
	command.angular.x = 0.0
	command.angular.y = 0.0
	command.angular.z = 0.0

	# Publish the command using the global publisher
	pub.publish(command)



if __name__ == '__main__':
    rospy.init_node('move2')

    # A subscriber for the laser scan data
    sub = rospy.Subscriber('scan', LaserScan, laser_callback)

    # A publisher for the move data
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    rospy.spin()
