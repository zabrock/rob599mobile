#!/usr/bin/env python

# Simple use of the ROS Navigation Stack
# Bill Smart
# ROB 599: Spring 2019

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler



def heading(yaw):
	"""A helper function to getnerate quaternions from yaws."""
	q = quaternion_from_euler(0, 0, yaw)
	return Quaternion(*q)


if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('move')

	# Make a goal.  This has a coordinate frame, a time stamp, and a target pose.  The Target pose is a
	# Point (where the z component should be zero) and a Quaternion (for the orientation).
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose = Pose(Point(5, 5, 0), heading(3.14))

	# An action client for move base
	move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	move_base.wait_for_server()

	# Send the goal ot move base, and wait for a result.  We can put a timeout on the wait, so that we don't
	# get stuck here for unreachable goals.  Once we're done, get the state from move base.
	move_base.send_goal(goal)
	success = move_base.wait_for_result(rospy.Duration(120))
	state = move_base.get_state()

	# Did everything work as expects?
	if success and state == GoalStatus.SUCCEEDED:
		print('Made it!')
	else:
		move_base.cancel_goal()
		print('Problem...')


