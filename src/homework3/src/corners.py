#!/usr/bin/env python

import rospy
import actionlib
import smach
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler

class ToCorner(smach.State):
	def __init__(self,move_base):
		smach.State.__init__(self,outcomes=['success','fail'])
		# Corner identifier is 0=top left, 1=bottom left,
		# 2=bottom right, 3=top right
		self.corner = 0
		# Positive is counter-clockwise, negative is clockwise
		self.direction = 1
		self.cornerCoords = [(1.0,1.0,0),(9.0,1.0,0),(9.0,9.0,0),(1.0,9.0,0)]
		pi = 3.1415
		self.headings = [-3*pi/4,-pi/4,pi/4,3*pi/4]
		print(len(self.cornerCoords))
		self.move_base = move_base
	def execute(self,userdata):
		# State execution
		goalCorner = self.cornerCoords[self.corner]
		print(goalCorner)
		# Make a goal.  This has a coordinate frame, a time stamp, and a target pose.  The Target pose is a
		# Point (where the z component should be zero) and a Quaternion (for the orientation).
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(*goalCorner), heading(self.headings[self.corner]))
		# Send the goal to move base, and wait for a result.  We can put a timeout on the wait, so that we don't
		# get stuck here for unreachable goals.  Once we're done, get the state from move base.
		self.move_base.send_goal(goal)
		success = self.move_base.wait_for_result(rospy.Duration(120))
		state = self.move_base.get_state()

		# Did everything work as expects?
		if success and state == GoalStatus.SUCCEEDED:
			print('Made it!')
			self.corner += self.direction
			if self.corner == len(self.cornerCoords)-1:
				self.direction = -1
			if self.corner == 0:
				self.direction = 1
			return 'success'
		else:
			self.move_base.cancel_goal()
			print('Problem...')
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
	rospy.init_node('corners')

	# An action client for move base
	move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	move_base.wait_for_server()

	sm = smach.StateMachine(outcomes=['nav fail','wait fail'])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add('TO_CORNER',ToCorner(move_base),
					transitions={'success':'WAIT',
						'fail':'nav fail'})
		smach.StateMachine.add('WAIT',waitAtCorner(),
					transitions={'success':'TO_CORNER',
						'fail':'wait fail'})

	# Execute SMACH plan
	outcome = sm.execute()

if __name__ == '__main__':
	main()

