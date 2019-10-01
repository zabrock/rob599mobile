#!/usr/bin/env python


# Every python controller needs these lines
import rospy
# Message synchronization
import message_filters
# Heading conversion
from tf.transformations import euler_from_quaternion

# Packages for array operations and image processing functions
import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares, minimize, leastsq

# Message types we need
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Needed math functions
from math import sin, cos, atan, pi, floor, exp, sqrt

# Point visualization
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class CylinderCounter:
	def __init__(self,rad):
		self.pub = rospy.Publisher('/visualization_marker',Marker,queue_size=10)
		self.odom_sub = message_filters.Subscriber('/odom', Odometry)
		self.lidar_sub = message_filters.Subscriber('/scan', LaserScan)
		self.sub = message_filters.TimeSynchronizer([self.odom_sub, self.lidar_sub], 10)
		self.sub.registerCallback(self.callback)
		self.init_points()
		self.x_rob = 0.0
		self.y_rob = 0.0
		self.th_rob = 0.0
		self.rad = rad
		self.found_points = []
		self.num_obs = []

	def init_points(self):
		self.points = Marker()
		self.points.header.frame_id="odom"
		self.points.header.stamp = rospy.Time.now()
		self.points.ns = 'cylinders'
		self.points.action = Marker.ADD
		self.points.id = 0
		self.points.type = Marker.POINTS
		self.points.scale.x = 0.2
		self.points.scale.y = 0.2
		self.points.color.r = 1.0
		self.points.color.a = 1.0
	
	def callback(self,odom_msg, lidar_msg):
		(_, _, self.th_rob) = euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])

		min_lidar_angle = lidar_msg.angle_min
		lidar_angle_delta = lidar_msg.angle_increment
		ranges = np.array(lidar_msg.ranges)

		# Extract current x and y pose for brevity
		self.x_rob = odom_msg.pose.pose.position.x
		self.y_rob = odom_msg.pose.pose.position.y

		# Take the difference between consecutive distance measurements
		diff = np.ediff1d(lidar_msg.ranges,to_end=0.0)
		# Find wherever the distance between consecutive measurements is within
		# a set tolerance of the expected object radius
		obj = np.abs(diff) < 1.5*self.rad
		# Discard wherever the distance is the maximum range of the scanner
		not_max_dist = np.array(lidar_msg.intensities) > 0.5
		obj = np.logical_and(obj,not_max_dist)

		# Identify groups within boolean array and find their centroids with respect to the robot
		labels,nlabels = ndimage.label(obj)
		for i in range(1,nlabels+1): # 1, nlabels+1
			# Test to identify individual features
			'''
			dist = np.mean(np.array(lidar_msg.ranges)[labels == i])
			
			angles = np.where(labels == i)
			avg_angle = min_lidar_angle + lidar_angle_delta*np.mean([angles[0],angles[-1]])

			x_obj = self.x_rob + dist*cos(avg_angle+self.th_rob)
			y_obj = self.y_rob + dist*sin(avg_angle+self.th_rob)
			'''
			# Estimate circle center by minimum distance plus expected radius
			feature_ranges = ranges[labels == i]
			if len(feature_ranges) < 30:
				continue
			min_range_ix = np.argmin(feature_ranges)
			angle_ixs = np.where(labels == i)[0]
			angle_idx = angle_ixs[min_range_ix]
			angle_est = min_lidar_angle + lidar_angle_delta*angle_idx
			center_est = polar_to_xy(np.array(np.min(feature_ranges)+self.rad),np.array(angle_est))
			
			# Convert all lidar points to x,y
			feature_angles = lidar_angle_delta*angle_ixs.astype(float) + min_lidar_angle
			[x,y] = polar_to_xy(np.array(feature_ranges),feature_angles)
			# Find best-fit circle through least-squares
			out, ier = leastsq(self.min_fun2,center_est,args=(x,y))
			xc = out[0]; yc = out[1]
			R_i = self.calc_R(x,y,xc,yc)
			R = R_i.mean()
			res = np.sum((R_i-R)**2)

			# Add found cylinders to list
			if res < 1.0 and abs(R-self.rad) < 0.3*self.rad:
				self.add_point_to_list(xc,yc)

		if len(self.points.points) > 0:
			self.pub.publish(self.points)

	def min_fun2(self,center,x,y):
		Ri = self.calc_R(x,y,*center)
		return Ri - Ri.mean()

	def calc_R(self,x,y,xc,yc):
		return np.sqrt((x-xc)**2 + (y-yc)**2)
	
	def add_point_to_list(self,xc,yc):
		update = False
		# Convert to odom frame
		x = xc*cos(self.th_rob)-yc*sin(self.th_rob) + self.x_rob
		y = xc*sin(self.th_rob)+yc*cos(self.th_rob) + self.y_rob
		for i in range(0,len(self.found_points)):
			point = self.found_points[i]
			# If the point to be added is close enough to a previously-found point (within
			# 1 diameter), update that point's location with the newly-determined location
			if abs(x-point[0]) < 2*self.rad and abs(y-point[1]) < 2*self.rad:
				self.found_points[i] = ((self.num_obs[i]*point[0]+x)/(self.num_obs[i]+1),(self.num_obs[i]*point[1]+y)/(self.num_obs[i]+1))
				self.num_obs[i] = self.num_obs[i] + 1
				update = True
				if self.num_obs[i] == 10:
					# After seeing the same feature ten times, assume it's a cylinder
					# This threshold is used to prevent seeing corners as circles
					print 'Found cylinder at: ', x, ' ', y
					pt = Point()
					pt.x = self.found_points[i][0]
					pt.y = self.found_points[i][1]
					self.points.points.append(pt)
				break
		if not update:
			# Add new point to potential cylinder location list
			self.found_points.append((x,y))
			self.num_obs.append(1)
			

		
	
def polar_to_xy(rad,angle):
	# Convert a point or list of points in polar coordinates to x,y coordinates
	if np.size(rad) > 1:
		x = np.zeros(np.size(rad))
		y = np.zeros(np.size(rad))
		for i in range(0,np.size(rad)):
			if np.size(angle) == np.size(rad):
				x[i] = rad[i]*cos(angle[i])
				y[i] = rad[i]*sin(angle[i])
			elif np.size(angle) == 1:
				x[i] = rad[i]*cos(angle)
				y[i] = rad[i]*sin(angle)
	else:
		x = rad*cos(angle)
		y = rad*sin(angle)
	return [x,y]

def main():
	rospy.init_node('counter')
	counter = CylinderCounter(0.25) # Cylinder diameter is 50 cm

	rospy.spin()

if __name__ == '__main__':
	main()
