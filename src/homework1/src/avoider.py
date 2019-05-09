#!/usr/bin/env python


# Every python controller needs these lines
import rospy

# The velocity command message
from geometry_msgs.msg import Twist

# The laser scan message
from sensor_msgs.msg import LaserScan
from math import sin, cos, atan2


# This is called every time we get a LaserScan message from ROS.
def laser_callback(msg):
	# Set obstacle gain
	gain = -0.0005
	# Get the angle data from the sensor
	delta_theta = msg.angle_increment
	angle_min = msg.angle_min
	ranges = msg.ranges

	# Initialize vectors
	goal_vec = [1,0]
	obst_vec = [0,0]
	robot_vec = [0,0]


	# Run through the sensor measurements for summation
	for i in range(0,len(ranges)):
		# Only look at points with high intensity
		if msg.intensities[i] > 0.25:
			theta = i*delta_theta + angle_min
			obst_vec[0] += cos(theta)/(ranges[i]**2)
			obst_vec[1] += sin(theta)/(ranges[i]**2)

	# Calculate desired robot velocity vector
	robot_vec[0] = goal_vec[0] + gain*obst_vec[0]
	robot_vec[1] = goal_vec[1] + gain*obst_vec[1]

	# Drive forward at speed x_robot and rotate proportional to the angle of robot_vec. The robot points up the x-axis.
	command = Twist()
	command.linear.x = 0.1*robot_vec[0]
	command.linear.y = 0.0
	command.linear.z = 0.0
	command.angular.x = 0.0
	command.angular.y = 0.0
	command.angular.z = 2*atan2(robot_vec[1],robot_vec[0])

	# Publish the command using the global publisher
	pub.publish(command)



if __name__ == '__main__':
    rospy.init_node('avoider')

    # A subscriber for the laser scan data
    sub = rospy.Subscriber('scan', LaserScan, laser_callback)

    # A publisher for the move data
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    rospy.spin()
