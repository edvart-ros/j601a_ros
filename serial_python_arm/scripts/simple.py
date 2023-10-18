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
        self.command_joints_serial_string = ['G00 J1=','00',' J2=','00',' J3=','00',' J4=','00',' J5=','00',' J6=','00']
        self.string_angle_indices = [1, 3, 5, 7, 9, 11]

        self.ser = serial.Serial('/dev/ttyS0')
        self.ser.baudrate = 115200
        self.ser.timeout = 15
        print('serial port opened')

        self.command_sub = rospy.Subscriber('command_joints', JointState, self.command_callback, queue_size=10)


    def command_callback(self, msg):
        self.mutex.acquire()

        i = 0
        for index in self.string_angle_indices:
            self.command_joints_serial_string[index] = str(msg.position[i]*180/numpy.pi)
            self.joint_states_msg.position[i] = msg.position[i]
            i += 1
        self.ser.write(''.join(self.command_joints_serial_string) + '\r\n')

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
