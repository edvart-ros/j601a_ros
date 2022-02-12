#!/usr/bin/env python

import rospy
import serial
import std_msgs.msg
from sensor_msgs.msg import JointState
import time
import numpy


class iterative_joint_state_publisher():

        def __init__(self):
                self.command_sub = rospy.Subscriber('joint_states_middleman', JointState, self.joint_command_callback, queue_size=100)
                self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

                self.current_joint_states = JointState()
                self.current_joint_states.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

		self.current_joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.current_joint_states.header.stamp.secs = 2222222222
                self.current_joint_states.header.stamp.nsecs = 111111111

		#self.joint_state_pub.publish(self.current_joint_states)

        def joint_command_callback(self, msg):
                print('joint command received, sending joint states')
		goal_joint_angles = numpy.array(msg.position)
		self.joint_state_pub.publish(self.current_joint_states)

		iterations = numpy.absolute((numpy.amax(goal_joint_angles) - numpy.amax(self.current_joint_states.position)))*500

		increments = (goal_joint_angles-self.current_joint_states.position)/iterations

		for i in range(int(iterations)):
			self.current_joint_states.position += increments
  	                self.joint_state_pub.publish(self.current_joint_states)
			time.sleep(0.008)
		self.current_joint_states.position = msg.position
		self.joint_state_pub.publish(self.current_joint_states)



if __name__ == '__main__':
        rospy.init_node('iterative_joint_state_publisher')
	obj = iterative_joint_state_publisher()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting down")
