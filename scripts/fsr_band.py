#!/usr/bin/env python
import rospy
import os
import time
import sys
import numpy as np
import ctypes
import serial
import ast
import math
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class FSR_Band:
	def __init__(self, port='/dev/ttyACM0', baud_rate=9600):

		# Initialize Node
		rospy.init_node('fsr_band', anonymous=True)

		# Variables
		self.num_sensors = 6
		self.values = None

		print("Opening Port")
		self.s = serial.Serial(port, baud_rate, timeout=1)
		time.sleep(2)

		# Initialize publisher
		self.pub = rospy.Publisher('fsr', Float32MultiArray, queue_size=1)

	def read_data(self):
		data = self.s.readline().rstrip()
		tmp = data.split(' ')
		# print(len(tmp))
		# print(tmp[2])
		values = []
		if len(tmp) == self.num_sensors + 2:
			for i in range(2, len(tmp)):
				values.append(float(tmp[i]))

		print(values)
		self.values = values

	def pub_data(self):
		if self.values is not None:
			msg = Float32MultiArray()
			msg.data = self.values

			msg.layout.dim = [MultiArrayDimension()]

			msg.layout.dim[0].label = 'fsr_band'
			msg.layout.dim[0].size = 6
			msg.layout.dim[0].stride = 1

			self.pub.publish(msg)
			self.values = None

	def run(self):
		while not rospy.is_shutdown():
			self.read_data()
			self.pub_data()

		self.s.close()


if __name__ == '__main__':
	F = FSR_Band()
	F.run()
