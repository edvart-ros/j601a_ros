#!/usr/bin/env python

import rospy
import serial
import std_msgs.msg
from sensor_msgs.msg import JointState

def subscriber_callback(msg):
	i = 0
	while i < 20:
		ser.write('G00 J1=' + str(i) + '\r\n')
		ser.read
		ser.read
		i += 1
		


rospy.init_node('joint_command_receiver')
subscriber = rospy.Subscriber('/joint_command_receiver', JointState, subscriber_callback)
position = JointState()
ser = serial.Serial('/dev/ttyS0')
ser.baudrate = 115200
ser.timeout = 15


while not rospy.is_shutdown():
	rospy.spin()
	
ser.close()
