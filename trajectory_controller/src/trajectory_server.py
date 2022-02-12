#!/usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import JointState
import time
import numpy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryGoal, FollowJointTrajectoryResult


class trajectoryserver(object):

	def __init__(self):
		self._as = actionlib.SimpleActionServer('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, self.execute_callback, False)
		self.pub_command = rospy.Publisher('/command_joints', JointState, queue_size = 100)
		self._result = FollowJointTrajectoryResult()
		self._as.start()
		self.ctrl_c = False

	def execute_callback(self, goal):
		for points in goal.trajectory.
