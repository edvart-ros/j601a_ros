#!/usr/bin/env python

import rospy
import serial
import std_msgs.msg
from sensor_msgs.msg import JointState
import time
import numpy
from threading import Lock


class serial_subscriber():

	def __init__(self):

		self.mutex = Lock()
		self.joint_angle_indexes = [1, 3, 5, 7, 9, 11]

		self.pub_middleman = rospy.Publisher('joint_states_middleman', JointState, queue_size=1)
		self.ser = serial.Serial('/dev/ttyS0')
		self.ser.baudrate = 115200
		self.ser.timeout = 15
		print('serial port opened')

		self.joint_states_msg = JointState()
		self.joint_states_msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
		self.joint_states_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.joint_states_msg.header.stamp.secs = 2222222222
                self.joint_states_msg.header.stamp.nsecs = 111111111

		self.command_sub = rospy.Subscriber('command_joints', JointState, self.command_callback, queue_size=10)

		self.command_joints_serial_msg = ['G00 J1=','00',' J2=','00',' J3=','00',' J4=','00',' J5=','00',' J6=','00']

	def command_callback(self, msg):

		self.mutex.acquire()
		

		i = 0
		for index in self.joint_angle_indexes:
			self.command_joints_serial_msg[index] = str(msg.position[i]*180/numpy.pi)
			self.joint_states_msg.position[i] = msg.position[i]
			i += 1

		self.ser.write(''.join(self.command_joints_serial_msg) + '\r\n')
		self.pub_middleman.publish(msg)

		while True:
			if self.ser.read() == "%":
				break
		self.mutex.release()


if __name__ == '__main__':
	rospy.init_node('serial_receiver')
	obj = serial_subscriber()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting down")

