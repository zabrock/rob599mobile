#!/usr/bin/env python

import rospy
import actionlib
import smach
import time
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler

class Patroller(smach.State):
	def __init__(self,move_base,patrol_route):
		smach.State.__init__(self,outcomes=['success','rm_waypoint','fail'])
		# Iterator over the points in patrol_route
		self.waypoint = 0
		# Positive is counter-clockwise, negative is clockwise
		self.move_base = move_base
		self.patrol_route = patrol_route
	def execute(self,userdata):
		# State execution
		goalCoords = self.patrol_route[self.waypoint][0:2]
		goalCoords = [goalCoords[0],goalCoords[1],0]
		goalHeading = self.patrol_route[self.waypoint][2]
		# Make a goal.  This has a coordinate frame, a time stamp, and a target pose.  The Target pose is a
		# Point (where the z component should be zero) and a Quaternion (for the orientation).
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
		if success and state == GoalStatus.SUCCEEDED:
			print('Made it!')
			self.waypoint += 1
			if self.waypoint >= len(self.patrol_route):
				self.waypoint = 0
				print('Patrol route finished, returning to first waypoint')
			return 'success'
		elif len(self.patrol_route) > 0:
			self.move_base.cancel_goal()
			print('Waypoint not reachable, deleting from patrol route')
			self.patrol_route = np.delete(self.patrol_route,self.waypoint,0)
			if len(self.patrol_route) == 0:
				print('Patrol route has no points!')
				return 'fail'
			if self.waypoint >= len(self.patrol_route):
				self.waypoint = 0
			return 'rm_waypoint'
		else:
			print('Did not reach patrol point and no remaining waypoints!')
			return 'fail'

class waitAtCorner(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['success','fail'])
		# Nothing to initialize
	def execute(self,userdata):
		# Wait at the corner for two seconds
		time.sleep(2)
		return 'success'
		

def heading(yaw):
	"""A helper function to getnerate quaternions from yaws."""
	q = quaternion_from_euler(0, 0, yaw)
	return Quaternion(*q)

def main():
	rospy.init_node('patroller')

	# An action client for move base
	move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	move_base.wait_for_server()

	route = np.loadtxt('patrol')

	sm = smach.StateMachine(outcomes=['sm_fail'])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add('PATROL',Patroller(move_base,route),
					transitions={'success':'PATROL',
						'rm_waypoint':'PATROL',
						'fail':'sm_fail'})

	# Execute SMACH plan
	outcome = sm.execute()

if __name__ == '__main__':
	main()

