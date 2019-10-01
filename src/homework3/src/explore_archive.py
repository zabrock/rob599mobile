#!/usr/bin/env python


# Every python controller needs these lines
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import message_filters
import smach

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler

# The velocity command message
from geometry_msgs.msg import Twist

# Packages for probability array and video output
import numpy as np
import skvideo.io
from PIL import Image
from scipy import ndimage

# The laser scan message
from sensor_msgs.msg import LaserScan

# Needed math functions
from math import sin, cos, atan, pi, floor, exp, sqrt

# Assume known start location, map size
x_start = 2.0
y_start = 2.0
resolution = 0.10
map_width = 10.0
map_length = 10.0
# Assume sensor standard deviation
sense_sd = 0.1

# Set up the occupancy grid data structure
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
# Set up the RGB visualization data structure
data = np.zeros((metric_map['rows'],metric_map['cols'],3),dtype=np.uint8)
# Initialize video writer
#writer = skvideo.io.FFmpegWriter("outputvideo.mp4")
#init = True

# This callback synchronizes odometry and lidar messages.  It also calculates the heading of the robot as an Euler angle, so 
# that it's easier to use with trigonometric functions.
def callback(odom_msg, lidar_msg):
#	global data
#	global init
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
			cell_idx = index_from_state(x_cell,y_cell,metric_map)
			# Only update cells up to three times the maximum range of the
			# laser scanner for readings that don't hit an obstacle
			if lidar_msg.intensities[i] > 0.25 or dist > 3*sense_sd:
				# Calculate P(occ|s)
				P_occ = metric_map['values'][cell_idx[0],cell_idx[1]]
				Ps_occ = normpdf(dist,0,sense_sd)
				Pocc_s_num = P_occ*Ps_occ
				Pocc_s_den = Pocc_s_num + (1-P_occ)*(1-Ps_occ)
				metric_map['values'][cell_idx[0],cell_idx[1]] = Pocc_s_num/Pocc_s_den
			else:
				break
			
			# Move to the next cell at step resolution. This is a less
			# accurate method than Bresenham's line algorithm or similar 
			# methods, but as a hacked solution, it seems to work okay.
			dist -= resolution
			x_cell += resolution * cos(lidar_angle+heading)
			y_cell += resolution * sin(lidar_angle+heading)

	# Scale occupancy grid to RGB values and mask unknown areas with blue
#	data[:,:,2] = 255-np.transpose(255*metric_map['values'])
#	masked_data = 255-np.transpose(255*metric_map['values'])
#	mask = np.where(np.logical_and(np.transpose(metric_map['values']) < 0.6,np.transpose(metric_map['values']) > 0.4))
#	masked_data[mask] = 0
#	data[:,:,0] = masked_data
#	data[:,:,1] = masked_data
#	# Add robot to image as a red circle
#	data = add_robot_to_img(x_odom,y_odom,data,metric_map)
#	# Save image as video frame
#	writer.writeFrame(data)
#	# Show update after first sensor measurement
#	if init:
#		img = Image.fromarray(data,mode='RGB')
#		img.save('first_measurement.png')
#		img.show()
#		init = False
			

def index_from_state(x,y,metric_map):
#	x_global = x_start + x
#	y_global = y_start + y
	x_global = x
	y_global = y
	res = metric_map['resolution']
	row_idx = int(floor(y_global/res))
	col_idx = int(floor(x_global/res))
	# Lock row and column indices to be within bounds of image
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
	# Calculate PDF given input x, mean, and standard deviation; used as
	# sensor model
	var = float(sd)**2
	denom = (2*pi*var)**.5
	num = exp(-(float(x)-float(mean))**2/(2*var))
	return num/denom

def add_robot_to_img(x_rob,y_rob,in_data,metric_map):
	# Get x and y coordinates for a filled circle
	theta = np.linspace(0,2*pi,num=60,endpoint=False)
	rad = np.linspace(0,0.18,num = 6) # Turtlebot2 diameter is 354mm
	y_circ = np.matmul(np.transpose(np.asmatrix(np.sin(theta))),np.asmatrix(rad)) + y_rob
	x_circ = np.matmul(np.transpose(np.asmatrix(np.cos(theta))),np.asmatrix(rad)) + x_rob
	y_circ = y_circ.flatten().tolist()[0]
	x_circ = x_circ.flatten().tolist()[0]
	# Assign all pixels in x,y to be red for output
	for x,y in zip(x_circ,y_circ):
		robot_idx = index_from_state(x,y,metric_map)
		metric_map['values'][robot_idx[0],robot_idx[1]] = 0.01
		in_data[robot_idx[1],robot_idx[0]] = [255,0,0]
	
	return in_data

class Explorer(smach.State):
	def __init__(self,move_base):
		smach.State.__init__(self,outcomes=['continue','finish','timeout'])
		# Positive is counter-clockwise, negative is clockwise
		self.move_base = move_base
		self.sub = rospy.Subscriber('/base_pose_ground_truth',Odometry,self.odom_callback)
		self.x_rob = 0.0
		self.y_rob = 0.0
	def execute(self,userdata):
		# State execution
		current_map = metric_map['values']
		current_map[current_map > 0.6] = 0.0
		current_map[current_map < 0.4] = 0.0
		labels,nlabels = ndimage.label(current_map)
		cy, cx = np.vstack(ndimage.center_of_mass(current_map, labels, np.arange(nlabels)+1)).T
		sizes = ndimage.sum(current_map, labels, np.arange(nlabels)+1)
		
		array = (current_map*256).astype(np.uint8)
		img = Image.fromarray(array)
	#	img.save('final_map.png')
		img.show()

		reward = 0.0
		for i in range(0,nlabels):
			if sizes[i] < 50:
				continue
			cost = self.distance(cx[i],cy[i])
			if reward < sizes[i]/cost:
				x_goal = metric_map['resolution']*cx[i]
				y_goal = metric_map['resolution']*cy[i]
				print('new potential cell: ',cx[i],cy[i])
		goalCoords = [x_goal,y_goal,0]
		print('new goal: ',goalCoords)
		# Make a goal.  This has a coordinate frame, a time stamp, and a target pose.  The Target pose is a
		# Point (where the z component should be zero) and a Quaternion (for the orientation).
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(*goalCoords), heading(0))
		# Send the goal ot move base, and wait for a result.  We can put a timeout on the wait, so that we don't
		# get stuck here for unreachable goals.  Once we're done, get the state from move base.
		self.move_base.send_goal(goal)
		success = self.move_base.wait_for_result(rospy.Duration(120))
		state = self.move_base.get_state()

		# Did everything work as expects?
		if success and state == GoalStatus.SUCCEEDED:
			print('Made it!')
			return 'continue'
		else:
			self.move_base.cancel_goal()
			print(state)
			return 'finish'
	def odom_callback(self,msg):
		# Extract current x and y pose
		self.x_rob = msg.pose.pose.position.x
		self.y_rob = msg.pose.pose.position.y

	def distance(self,x,y):
		dist = sqrt((x-self.x_rob)**2 + (y-self.y_rob)**2)
		return dist
		
class Spinner(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['finished','continue'])
		self.sub = rospy.Subscriber('/base_pose_ground_truth',Odometry,self.odom_callback)
		self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.theta_d1 = 0.0
		self.del_theta = 0.0
		self.update = False

	def execute(self,userdata):
		self.update = True
		if self.del_theta < 3.14:
			cmd = Twist()
			cmd.angular.z = 0.5
			self.pub.publish(cmd)
			return 'continue'
		else:
			self.update = False
			self.del_theta = 0.0
			return 'finished'

	def odom_callback(self,odom_msg):
		(_, _, heading) = euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])
		if self.update:
			del_theta = heading - self.theta_d1
			if del_theta > 3.14:
				del_theta -= 3.14
			if del_theta < -3.14:
				del_theta += 3.14
			self.del_theta += del_theta

		self.theta_d1 = heading
		

def heading(yaw):
	"""A helper function to getnerate quaternions from yaws."""
	q = quaternion_from_euler(0, 0, yaw)
	return Quaternion(*q)

def main():
	rospy.init_node('explorer')

	# An action client for move base
	move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	move_base.wait_for_server()

	odom_sub = message_filters.Subscriber('/base_pose_ground_truth', Odometry)
	lidar_sub = message_filters.Subscriber('/scan', LaserScan)
	sub = message_filters.TimeSynchronizer([odom_sub, lidar_sub], 10)
	sub.registerCallback(callback)

	sm = smach.StateMachine(outcomes=['sm_timeout','sm_explored'])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add('SPIN',Spinner(),
					transitions={'continue':'SPIN',
						'finished':'EXPLORE'})
		smach.StateMachine.add('EXPLORE',Explorer(move_base),
					transitions={'continue':'SPIN',
						'finish':'sm_explored',
						'timeout':'sm_timeout'})

	# Execute SMACH plan
	outcome = sm.execute()

	# After program is terminated, close the video and write out final map
#	writer.close()
#	img_data = np.zeros((metric_map['rows'],metric_map['cols'],3),dtype=np.uint8)
#	img_data[:,:,2] = 255-np.transpose(255*metric_map['values'])
#	masked_data = 255-np.transpose(255*metric_map['values'])
#	mask = np.where(np.logical_and(np.transpose(metric_map['values']) < 0.6,np.transpose(metric_map['values']) > 0.4))
#	masked_data[mask] = 0
#	img_data[:,:,0] = masked_data
#	img_data[:,:,1] = masked_data
	
#	img = Image.fromarray(img_data,mode='RGB')
#	img.save('final_map.png')
#	img.show()

if __name__ == '__main__':
	main()
