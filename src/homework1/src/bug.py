#!/usr/bin/env python


# Every python controller needs these lines
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import message_filters

# The velocity command message
from geometry_msgs.msg import Twist

# The laser scan message
from sensor_msgs.msg import LaserScan
from math import sin, cos, atan, pi, tanh

# State variable and numeric assignments for relevant states
go_to_goal = 0
wall_follow = 1
rotate = 2
finished = 3
robot_state = go_to_goal

# Control gain for robot rotation in go-to-goal state and wall separation during
# wall following
K_heading = 1
K_follow = 1

# Setting goal location based in robot coordinate system; currently
# set to opposite corner from robot
x_goal = 7.0
y_goal = 7.0
# Set desired wall distance; this dictates when wall following begins and
# how far away the robot tries to stay from a wall
desired_distance = 0.5

# Global variables used to help with state functions
angle_rotate_start = 0.0
init = True
num_cycles = 0
slope = 0.0

# This callback synchronizes odometry and lidar messages.  It also calculates the heading of the robot as an Euler angle, so 
# that it's easier to use with trigonometric functions.
def callback(odom_msg, lidar_msg):
	(_, _, heading) = euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])

	# Extract current x and y pose for brevity
	x_cur = odom_msg.pose.pose.position.x
	y_cur = odom_msg.pose.pose.position.y

	# Bring in the global variables
	global robot_state
	global angle_rotate_start
	global init
	global num_cycles
	global slope
	
	# On first iteration, find the slope of the line we're trying to follow
	if init:
		slope = (y_goal - y_cur)/(x_goal - x_cur)
		init = False
		
	# Go-to-goal (line-following) behavior
	if robot_state == go_to_goal:
		# If goal is reached, stop moving and enter Finished state
		if abs(y_goal-y_cur) < 0.05 and abs(x_goal-x_cur) < 0.05:
			robot_state = finished
			print("Finished")
			vel_des = 0.0
			omega_des = 0.0
		else:
			# Go through lidar readings in front of robot
			for i in range(len(lidar_msg.ranges)/3,2*len(lidar_msg.ranges)/3):
				# If any reading in front of the robot is within
				# desired_distance, rotate 90 degrees to start wall
				# wall following
				if lidar_msg.ranges[i] < desired_distance:
					vel_des = 0.0
					angle_rotate_start = heading
					omega_des = 0.1
					robot_state = rotate
					print("Rotate")
					break
			# If no obstacle too close, check heading and turn toward
			# goal if necessary; else, drive in a straight line toward goal
			angle_diff = atan((y_goal-y_cur)/(x_goal-x_cur)) - heading
			if abs(angle_diff) > 0.1:
				omega_des = K_heading * angle_diff
				vel_des = 0.0
		
			else:
				omega_des = 0.0
				vel_des = 0.1*tanh(3*lidar_msg.ranges[len(lidar_msg.ranges)/2])*tanh(20*((y_goal-y_cur)**2+(x_goal-x_cur)**2))

	# Rotate behavior to transition from line-following to wall-following
	elif robot_state == rotate:
		vel_des = 0.0
		# Determine where we want to turn to
		des_heading = angle_rotate_start + pi/2
		angle_diff = des_heading - heading
		# If still not turned 90 degrees total, keep turning
		if angle_diff > 0.05:
			omega_des = K_heading*tanh(3*angle_diff)
		# Otherwise, start wall following
		else:
			omega_des = 0.0
			robot_state = wall_follow
			num_cycles = 0
			print("Wall follow")
			
	# Wall-following behavior
	elif robot_state == wall_follow:
		# Check if the robot is close to the desired line; make sure at least
		# a few cycles have passed so that the robot has moved from its initial 
		# position
		if abs(y_goal-y_cur - slope*(x_goal-x_cur)) < 0.01 and num_cycles > 20:
			num_cycles = 0
			robot_state = go_to_goal
			vel_des = 0.0
			omega_des = 0.0
			print("Line follow")
		else:
			# Default to driving forward and check lidar readings on the 				# right side to determine how much the robot should rotate
			vel_des = 0.1
			min_dist = []
			for i in range(0,len(lidar_msg.ranges)/3):
				if not min_dist or min_dist < lidar_msg.ranges[i]:
					min_dist = lidar_msg.ranges[i]
			# Scale desired distance since the laser scanner is centered
			# on the robot and we need to look at the side
			distance_diff = (1+K_follow)*desired_distance - min_dist
			omega_des = 0.1*distance_diff

			# Check ahead of the robot for obstacles; if any points are 
			# too close, stop moving forward and rotate away from the
			# obstacle
			for i in range(len(lidar_msg.ranges)/3,2*len(lidar_msg.ranges)/3):
				if lidar_msg.ranges[i] < desired_distance:
					omega_des = 0.1
					vel_des = 0.0
					break
		
		num_cycles += 1

	# Finished state - don't move, we're at the goal
	else:
		vel_des = 0.0
		omega_des = 0.0
			

	# Apply the control calculated from above.  The robot points up the x-axis.
	command = Twist()
	command.linear.x = vel_des
	command.linear.y = 0.0
	command.linear.z = 0.0
	command.angular.x = 0.0
	command.angular.y = 0.0
	command.angular.z = omega_des

	# Publish the command using the global publisher
	pub.publish(command)



if __name__ == '__main__':
	rospy.init_node('bug')

	odom_sub = message_filters.Subscriber('/odom', Odometry)
	lidar_sub = message_filters.Subscriber('/scan', LaserScan)
	sub = message_filters.TimeSynchronizer([odom_sub, lidar_sub], 10)
	sub.registerCallback(callback)

	# A publisher for the move data
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

	rospy.spin()

