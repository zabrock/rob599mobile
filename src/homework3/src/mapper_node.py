#!/usr/bin/env python


# Every python controller needs these lines
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import message_filters

# Packages for probability array and video output
import numpy as np
from PIL import Image

# The velocity command message
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid

# The laser scan message
from sensor_msgs.msg import LaserScan

# Needed math functions
from math import sin, cos, atan, pi, floor, exp

# Assume known start location, map size
x_start = 2.0
y_start = 2.0
resolution = 0.05
map_width = 10.0
map_height = 10.0
# Assume sensor standard deviation
sense_sd = 0.1

# Set up the occupancy grid data structure
rows = int(map_width/resolution)
cols = int(map_height/resolution)
data = np.array([-0.01]*rows*cols)

# This callback synchronizes odometry and lidar messages.  It also calculates the heading of the robot as an Euler angle, so 
# that it's easier to use with trigonometric functions.
def callback(odom_msg, lidar_msg):
	(_, _, heading) = euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])

	# Extract current x and y pose for brevity
	x_odom = odom_msg.pose.pose.position.x
	y_odom = odom_msg.pose.pose.position.y

	# Go through the ranges in LaserScan and determine the x,y coordinates
	# from the camera frame
	min_lidar_angle = lidar_msg.angle_min
	lidar_angle_delta = lidar_msg.angle_increment
	for j in range(0,len(lidar_msg.ranges)/2):
		i = 2*j
		
		dist = lidar_msg.ranges[i]
		# Start at the cell with the robot
		x_cell = x_odom
		y_cell = y_odom
		# Find the location of the laser endpoint
		lidar_angle = min_lidar_angle + i*lidar_angle_delta
		x_point = x_odom + lidar_msg.ranges[i]*cos(lidar_angle+heading)
		y_point = y_odom + lidar_msg.ranges[i]*sin(lidar_angle+heading)
		x_diff = x_point - x_cell
		y_diff = y_point - y_cell

		# As long as the vector from the cell to the endpoint is the same
		# direction as the vector from the robot to the endpoint...
		while (x_point-x_cell)*x_diff > 0 and (y_point-y_cell)*y_diff > 0:
			# Correlate cell location to grid indices
			cell_idx = index_from_state(x_cell,y_cell)
			# Only update cells up to three times the maximum range of the
			# laser scanner for readings that don't hit an obstacle
			if lidar_msg.intensities[i] > 0.25 or dist > 3*sense_sd:
				# Calculate P(occ|s)
				P_occ = data[cell_idx]
				if P_occ < 0:
					P_occ = 0.5
				Ps_occ = normpdf(dist,0,sense_sd)
				Pocc_s_num = P_occ*Ps_occ
				Pocc_s_den = Pocc_s_num + (1-P_occ)*(1-Ps_occ)
				data[cell_idx] = Pocc_s_num/Pocc_s_den
			else:
				break
			
			# Move to the next cell at step resolution. This is a less
			# accurate method than Bresenham's line algorithm or similar 
			# methods, but as a hacked solution, it seems to work okay.
			dist -= resolution
			x_cell += resolution * cos(lidar_angle+heading)
			y_cell += resolution * sin(lidar_angle+heading)
			

def index_from_state(x,y):
	x_global = x
	y_global = y
	row_idx = int(floor(y_global/resolution))
	col_idx = int(floor(x_global/resolution))
	# Lock row and column indices to be within bounds of image
	if row_idx >= rows:
		row_idx = rows-1
	elif row_idx < 0:
		row_idx = 0
	if col_idx >= cols:
		col_idx = cols-1
	elif col_idx < 0:
		col_idx = 0
	return row_idx*cols+col_idx

def normpdf(x, mean, sd):
	# Calculate PDF given input x, mean, and standard deviation; used as
	# sensor model
	var = float(sd)**2
	denom = (2*pi*var)**.5
	num = exp(-(float(x)-float(mean))**2/(2*var))
	return num/denom

if __name__ == '__main__':
	rospy.init_node('mapper_node')

	odom_sub = message_filters.Subscriber('/base_pose_ground_truth', Odometry)
	lidar_sub = message_filters.Subscriber('/scan', LaserScan)
	sub = message_filters.TimeSynchronizer([odom_sub, lidar_sub], 10)
	sub.registerCallback(callback)

	rospy.spin()

	# After program is terminated, close the video and write out final map
	img_data = np.zeros((rows,cols,3),dtype=np.uint8)
	
	temp = np.transpose(np.reshape(data,(rows,cols)))
	mask = np.where(temp < 0.0)
	temp[mask] = 0.5
	masked_data = 255*(1-temp)
	masked_data[mask] = 0

	img_data[:,:,2] = 255*(1-temp)
	img_data[:,:,0] = masked_data
	img_data[:,:,1] = masked_data
	
	img = Image.fromarray(img_data,mode='RGB')
	img.save('final_map_custom2.png')
	img.show()
