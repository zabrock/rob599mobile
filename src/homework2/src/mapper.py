#!/usr/bin/env python


# Every python controller needs these lines
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import message_filters

# Packages for probability array and video output
import numpy as np
import skvideo.io
from PIL import Image

# The velocity command message
from geometry_msgs.msg import Twist

# The laser scan message
from sensor_msgs.msg import LaserScan

# Needed math functions
from math import sin, cos, atan, pi, floor, exp

# Assume known start location, map size
x_start = 2.0
y_start = 2.0
resolution = 0.05
map_width = 10.0
map_length = 10.0
sense_sd = 0.1

rows = int(map_width/resolution)
cols = int(map_length/resolution)
num_cells = rows * cols
metric_map = { 'width': map_width,
	'length': map_length,
	'resolution': resolution,
	'values': 0.5*np.ones((rows,cols),dtype=np.float32),
	'rows':rows,
	'cols':cols
}
data = np.zeros((metric_map['rows'],metric_map['cols'],3),dtype=np.uint8)

init = True

writer = skvideo.io.FFmpegWriter("outputvideo.mp4")

# This callback synchronizes odometry and lidar messages.  It also calculates the heading of the robot as an Euler angle, so 
# that it's easier to use with trigonometric functions.
def callback(odom_msg, lidar_msg):
	global data
	(_, _, heading) = euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])

	# Extract current x and y pose for brevity
	x_odom = odom_msg.pose.pose.position.x
	y_odom = odom_msg.pose.pose.position.y

	# Go through the ranges in LaserScan and determine the x,y coordinates
	# from the camera frame
	min_lidar_angle = lidar_msg.angle_min
	lidar_angle_delta = lidar_msg.angle_increment
	tol = 0.01
	x = []
	y = []
	for i in range(0,len(lidar_msg.ranges)):
		
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
		point_idx = index_from_state(x_point,y_point,metric_map)
		while (x_point-x_cell)*x_diff > 0 and (y_point-y_cell)*y_diff > 0:
			cell_idx = index_from_state(x_cell,y_cell,metric_map)
			# Calculate P(s|occ)
			P_occ = metric_map['values'][cell_idx[0],cell_idx[1]]
			if dist < 3*sense_sd and lidar_msg.intensities[i] < 0.25:
				#Ps_occ = normpdf(10*resolution,0,sense_sd)
				Ps_occ = 0.5
			else:
				Ps_occ = normpdf(dist,0,sense_sd)
				Pocc_s_num = P_occ*Ps_occ
				Pocc_s_den = Pocc_s_num + (1-P_occ)*(1-Ps_occ)
				metric_map['values'][cell_idx[0],cell_idx[1]] = Pocc_s_num/Pocc_s_den
			
			# Move to the next point at step resolution
			dist -= resolution
			x_cell += resolution * cos(lidar_angle+heading)
			y_cell += resolution * sin(lidar_angle+heading)

	
	data[:,:,2] = np.transpose(255*metric_map['values'])
	masked_data = np.transpose(255*metric_map['values'])
	mask = np.where(np.logical_and(np.transpose(metric_map['values']) < 0.6,np.transpose(metric_map['values']) > 0.4))
	masked_data[mask] = 0
	data[:,:,0] = masked_data
	data[:,:,1] = masked_data
	data = add_robot_to_img(x_odom,y_odom,data,metric_map)
	writer.writeFrame(data)
			

def index_from_state(x,y,metric_map):
	x_global = x_start + x
	y_global = y_start + y
	res = metric_map['resolution']
	row_idx = int(floor(y_global/res))
	col_idx = int(floor(x_global/res))
	if row_idx >= metric_map['rows']:
		row_idx = metric_map['rows']-1
	elif row_idx < 0:
		row_idx = 0
	if col_idx >= metric_map['cols']:
		col_idx = metric_map['cols']-1
	elif col_idx < 0:
		col_idx = 0
	return (row_idx,col_idx)

def normpdf(x, mean, sd):
	var = float(sd)**2
	denom = (2*pi*var)**.5
	num = exp(-(float(x)-float(mean))**2/(2*var))
	return num/denom

def distance(x1,y1,x2,y2):
	return math.sqrt((x2-x1)**2-(y2-y1)**2)

def add_robot_to_img(x_rob,y_rob,in_data,metric_map):
	theta = np.linspace(0,2*pi,num=60,endpoint=False)
	rad = np.linspace(0,5*resolution,num = 6)
	y_circ = np.matmul(np.transpose(np.asmatrix(np.sin(theta))),np.asmatrix(rad)) + y_rob
	x_circ = np.matmul(np.transpose(np.asmatrix(np.cos(theta))),np.asmatrix(rad)) + x_rob
	y_circ = y_circ.flatten().tolist()[0]
	x_circ = x_circ.flatten().tolist()[0]
	for x,y in zip(x_circ,y_circ):
		robot_idx = index_from_state(x,y,metric_map)
		in_data[robot_idx[1],robot_idx[0]] = [255,0,0]
	
	return in_data

if __name__ == '__main__':
	rospy.init_node('mapper')

	odom_sub = message_filters.Subscriber('/odom', Odometry)
	lidar_sub = message_filters.Subscriber('/scan', LaserScan)
	sub = message_filters.TimeSynchronizer([odom_sub, lidar_sub], 10)
	sub.registerCallback(callback)

	rospy.spin()
	writer.close()

	img_data = np.zeros((metric_map['rows'],metric_map['cols'],3),dtype=np.uint8)
	img_data[:,:,2] = np.transpose(255*metric_map['values'])
	masked_data = np.transpose(255*metric_map['values'])
	mask = np.where(np.logical_and(np.transpose(metric_map['values']) < 0.6,np.transpose(metric_map['values']) > 0.4))
	masked_data[mask] = 0
	img_data[:,:,0] = masked_data
	img_data[:,:,1] = masked_data
	
	img = Image.fromarray(img_data,mode='RGB')
	img.save('output_map.png')
	img.show()
