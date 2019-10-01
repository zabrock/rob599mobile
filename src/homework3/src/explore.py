#!/usr/bin/env python


# Every python controller needs these lines
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import message_filters
import smach
import time

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler

from nav_msgs.msg import OccupancyGrid

# The velocity command message
from geometry_msgs.msg import Twist

# Packages for probability array and video output
import numpy as np
from scipy import ndimage

# The laser scan message
from sensor_msgs.msg import LaserScan

# Needed math functions
from math import sin, cos, atan, pi, floor, exp, sqrt

class Explorer(smach.State):
	def __init__(self,move_base,start_time):
		smach.State.__init__(self,outcomes=['continue','goal_not_reached','finish','timeout'])
		self.move_base = move_base
		self.sub = rospy.Subscriber('/base_pose_ground_truth',Odometry,self.odom_callback)
		self.gmap = rospy.Subscriber('/map',OccupancyGrid,self.map_callback)
		self.pub = rospy.Publisher('/visualization_marker',Marker,queue_size=10)
		self.x_rob = 0.0
		self.y_rob = 0.0
		self.start_time = start_time
	def execute(self,userdata):
		# State execution
		grid = np.array(self.gmap.data)
		grid = np.resize(grid,(self.gmap.info.height,self.gmap.info.width))
		edges = np.logical_and(np.abs(ndimage.laplace(grid))>0,np.abs(ndimage.laplace(grid))<10)
		labels,nlabels = ndimage.label(edges)
		cy, cx = np.vstack(ndimage.center_of_mass(edges, labels, np.arange(nlabels)+1)).T
		sizes = ndimage.sum(grid, labels, np.arange(nlabels)+1)

		points = Marker()
		points.header.frame_id="map"
		points.header.stamp = rospy.Time.now()
		points.ns = 'frontier_centroids'
		points.action = Marker.ADD
		points.id = 0
		points.type = Marker.POINTS
		points.scale.x = 0.2
		points.scale.y = 0.2
		points.color.g = 1.0
		points.color.a = 1.0

		# greedily pick closest frontier
		reward = 0.0
		goalCoords = []
		for i in range(0,nlabels):
			pt = Point()
			pt_coord = self.state_from_index(cx[i],cy[i])
			pt.x = pt_coord[0]
			pt.y = pt_coord[1]
			pt.z = 0.0
			points.points.append(pt)
			this_reward = -sizes[i]/self.distance(pt_coord)
			if -sizes[i] > 5 and this_reward > reward:
				print(this_reward,pt_coord)
				reward = this_reward
				goalCoords = pt_coord
				goalHeading = np.arctan2(goalCoords[1]-self.y_rob,goalCoords[0]-self.x_rob)
		# Make a goal.  This has a coordinate frame, a time stamp, and a target pose.  The Target pose is a
		# Point (where the z component should be zero) and a Quaternion (for the orientation).
		if not goalCoords:
			print('No more goals of relevant size (>5) identified!')
			return 'finish'
		
		self.pub.publish(points)
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(*goalCoords), heading(goalHeading))
		# Send the goal ot move base, and wait for a result.  We can put a timeout on the wait, so that we don't
		# get stuck here for unreachable goals.  Once we're done, get the state from move base.
		self.move_base.send_goal(goal)
		success = self.move_base.wait_for_result(rospy.Duration(120))
		state = self.move_base.get_state()

		# Did everything work as expects?
		if rospy.is_shutdown() or time.time() - self.start_time > 300:
			print('Timeout or ROS shutdown - finishing program')
			return 'timeout'
		if success and state == GoalStatus.SUCCEEDED:
			print('Made it!')
			return 'continue'
		else:
			print('Goal not reachable')
			self.move_base.cancel_goal()
			return 'goal_not_reached'
	def odom_callback(self,msg):
		# Extract current x and y pose
		self.x_rob = msg.pose.pose.position.x - 2.0
		self.y_rob = msg.pose.pose.position.y - 2.0

	def map_callback(self,msg):
		self.gmap = msg

	def distance(self,pt_coords):
		dist = sqrt((pt_coords[0]-self.x_rob)**2 + (pt_coords[1]-self.y_rob)**2)
		return dist

	def state_from_index(self,xcell,ycell):
		xo = self.gmap.info.origin.position.x
		yo = self.gmap.info.origin.position.y
		resolution = self.gmap.info.resolution
		x = xo + resolution*xcell
		y = yo + resolution*ycell
		return (x,y,0)

	def index_from_state(self,x,y):
		row_idx = int(floor(y/resolution))
		col_idx = int(floor(x/resolution))
		# Lock row and column indices to be within bounds of image
		if row_idx >= rows:
			row_idx = rows-1
		elif row_idx < 0:
			row_idx = 0
		if col_idx >= cols:
			col_idx = cols-1
		elif col_idx < 0:
			col_idx = 0
		return (row_idx,col_idx)
		
class Spinner(smach.State):
	def __init__(self,start_time):
		smach.State.__init__(self,outcomes=['finished','continue','timeout'])
		self.sub = rospy.Subscriber('/base_pose_ground_truth',Odometry,self.odom_callback)
		self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.theta_d1 = 0.0
		self.del_theta = 0.0
		self.update = False
		self.start_time = start_time

	def execute(self,userdata):
		if rospy.is_shutdown() or time.time() - self.start_time > 300:
			print('ROS shutdown or timeout!')
			return 'timeout'
		self.update = True
		cmd = Twist()
		if self.del_theta < 5:
			cmd.angular.z = 1.0
			self.pub.publish(cmd)
			return 'continue'
		elif self.del_theta < 6.28:
			cmd.angular.z = 0.5
			self.pub.publish(cmd)
			return 'continue'
		else:
			cmd.angular.z = 0.0
			self.pub.publish(cmd)
			self.update = False
			self.del_theta = 0.0
			return 'finished'

	def odom_callback(self,odom_msg):
		(_, _, heading) = euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])
		if self.update:
			delta_theta = heading - self.theta_d1
			if delta_theta > 6:
				delta_theta -= 6.28
			if delta_theta < -6:
				delta_theta += 6.28
			self.del_theta += delta_theta

		self.theta_d1 = heading

class Backup(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['finished'])
		self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

	def execute(self,userdata):
		cmd = Twist()
		cmd.linear.x = -0.1
		time.sleep(1)
		return 'finished'
		

def heading(yaw):
	"""A helper function to getnerate quaternions from yaws."""
	q = quaternion_from_euler(0, 0, yaw)
	return Quaternion(*q)

def main():
	rospy.init_node('explorer')

	# An action client for move base
	move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	move_base.wait_for_server()

	sm = smach.StateMachine(outcomes=['sm_timeout','sm_explored'])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add('SPIN',Spinner(time.time()),
					transitions={'continue':'SPIN',
						'finished':'EXPLORE',
						'timeout':'sm_timeout'})
		smach.StateMachine.add('EXPLORE',Explorer(move_base,time.time()),
					transitions={'continue':'SPIN',
						'goal_not_reached':'BACKUP',
						'finish':'sm_explored',
						'timeout':'sm_timeout'})
		smach.StateMachine.add('BACKUP',Backup(),
					transitions={'finished':'EXPLORE'})

	# Execute SMACH plan
	outcome = sm.execute()

if __name__ == '__main__':
	main()
