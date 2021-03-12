#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, Int32
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int32, String, Float32
import numpy as np
import pandas as pd
import serial
import time
from sklearn.ensemble import RandomForestClassifier
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerRequest, TriggerResponse
import datetime
import os
from math import pi
from ndx_emg_teleop.srv import write_data, plot_data, write_dataRequest, write_dataResponse
import pickle
import math
import matplotlib.pyplot as plt
# sys.path.extend(['../', '../../'])
from scipy import signal


class FSRClient:

	def __init__(self):
		# Initialize Node
		rospy.init_node('fsr_client', anonymous=True)
		home = os.path.expanduser("~")

		# Parameters
		self.data_write_path = os.path.join(home, rospy.get_param('~data_write_path', 'experiments/fsr_band'))
		self.filename = rospy.get_param('~filename_client', 'fsr_client.csv')
		self.append_timestamp = rospy.get_param('~append_timestamp', False)
		self.pred_window_val = rospy.get_param('~prediction_window', 5)
		learning_model_name = rospy.get_param('~learning_model', 'right_gesture_classifier.sav')
		self.learning_model_name = os.path.join(self.data_write_path, learning_model_name)

		try:
			self.learning_model = pickle.load(open(self.learning_model_name, 'rb'))
		except IOError:
			rospy.logerr("Learning Model %s not found!!", self.learning_model_name)

		# Check if Write Path Exist
		if not os.path.isdir(self.data_write_path):
			os.makedirs(self.data_write_path)

		# Variables
		self.data_raw = []
		self.data_features = []
		self.data_emg_features = []
		# self.data_load_cell = []
		self.load_cell_arr = []
		self.data = None
		self.label = 0
		self.new_packet = False
		self.data_packet = None
		self.prediction = False
		self.load_cell_reading = 0
		self.force_prediction_data = []
		self.latest_force_val = 0
		self.pause_reading_force = False
		self.pred_window = []
		self.pred_window_x = []
		self.pred_window_y = []
		self.pred_window_z = []
		self.write_data_proxy = None
		self.clear_data_proxy = None
		self.toggle_label_proxy = None
		self.stop_left_arm_proxy = None
		self.stop_right_arm_proxy = None

		# Data for plot
		self.plot_data = []
		self.update = False
		self.arr_proc = None
		plt.ion()
		self.fig = None
		self.if_plot = False
		self.ch = [2]

		# Initialize Subscribers
		rospy.Subscriber('/fsr', Float32MultiArray, self.extracted_features_callback, queue_size=1)
		rospy.Subscriber('/fsr', Float32MultiArray, self.emg_plot_raw, queue_size=1)

		# Advertise Services
		rospy.Service('write_data', write_data, self.write_data)
		rospy.Service('calibrate_gesture', write_data, self.gesture_calibration_callback)
		rospy.Service('clear_data_array', Trigger, self.clear_data_array)
		rospy.Service('toggle_label', Trigger, self.toggle_label)
		rospy.Service('toggle_prediction', Trigger, self.toggle_prediction)
		rospy.Service('toggle_plot', plot_data, self.toggle_plot)

		# Wait for Services
		self.wait_for_services()

		# System Ready Notice
		rospy.loginfo("System Ready")

	def wait_for_services(self):
		try:
			rospy.wait_for_service('write_data')
			self.write_data_proxy = rospy.ServiceProxy('write_data', write_data)
		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)

		try:
			rospy.wait_for_service('clear_data_array')
			self.clear_data_proxy = rospy.ServiceProxy('clear_data_array', Trigger)
		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)

		try:
			rospy.wait_for_service('toggle_label')
			self.toggle_label_proxy = rospy.ServiceProxy('toggle_label', Trigger)
		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)

	def gesture_calibration_callback(self, req):
		gestures = req.filename.split(',')
		print(gestures)
		self.gesture_calibration(gestures)
		msg = "Gestures Calibrated"
		return True, msg

	def gesture_calibration(self, gestures):
		num_gestures = len(gestures)
		gesture_counter = 0
		reps = 5
		timer = 3

		rospy.loginfo("Starting Data Collection Process")
		time.sleep(3)
		response = self.clear_data_proxy(TriggerRequest())
		for g in gestures:
			gesture_counter += 1
			if gesture_counter == num_gestures:
				rospy.loginfo("\nCOLLECTING DATA FOR THE FINAL GESTURE!!")

			rospy.loginfo("\nCollecting Data for {} gesture!".format(g))
			self.label = 0
			time.sleep(3)
			response = self.clear_data_proxy(TriggerRequest())
			for i in range(reps):
				if i == reps - 1:
					rospy.loginfo("\nFINAL ITERATION FOR THIS GESTURE!!")

				rospy.loginfo("\nKeep the hand in Rest State")
				time.sleep(12)
				for t in range(timer):
					rospy.loginfo("Get Ready to Execute {} gesture in {} seconds...".format(g, timer - t))
					time.sleep(1)

				# Toggle Label
				self.label = 1

				rospy.loginfo("\nKeep the hand in {} gesture State".format(g))
				time.sleep(12)
				for t in range(timer):
					rospy.loginfo("Get Ready to return to Rest State in {} seconds...".format(timer - t))
					time.sleep(1)

				# Toggle Label
				self.label = 0

			# Write Data
			time.sleep(1)
			req = write_dataRequest()
			req.filename = g
			response = self.write_data_proxy(req)
			response = self.clear_data_proxy(TriggerRequest())
			rospy.loginfo("\nData Written for {} gesture".format(g))
			if gesture_counter < num_gestures:
				rospy.loginfo("Take a 10 seconds break! :)" )
				time.sleep(10)
				for t in range(timer):
					rospy.loginfo("Starting Data Collection for Next Gesture in {} seconds...".format(timer - t))
					time.sleep(1)
			response = self.clear_data_proxy(TriggerRequest())

	def toggle_label(self, req):
		"""
		command to call: rosservice call /toggle_label
		"""
		if self.label == 1:
			self.label = 0
		else:
			self.label = 1

		msg = "Label Status Changed, Current Label Value: " + str(self.label)
		return TriggerResponse(success=True, message=msg)

	def toggle_prediction(self, req):
		"""
		command to call: rosservice call /toggle_prediction
		"""
		self.prediction = not self.prediction
		if len(self.force_prediction_data) > 0:
			df = pd.DataFrame(data=self.force_prediction_data)
			print(self.data_write_path)
			filename = 'Force_Prediction_' + datetime.datetime.now().strftime("%Y-%m-%d-%H%M%S") + '.csv'
			df.to_csv(os.path.join(self.data_write_path, filename), index=False)

			self.force_prediction_data = []

		msg = "Label Status Changed, Current Label Value: " + str(self.prediction)
		return TriggerResponse(success=True, message=msg)

	def toggle_plot(self, req):
		"""
		command to call: rosservice call /toggle_plot
		"""
		self.ch = req.ch
		if -1 in self.ch:
			self.if_plot = False
		else:
			self.if_plot = True

		print(list(self.ch))

		msg = "Plotting Status Changed: Current Status " + str(self.if_plot)
		return True, msg

	def emg_plot_raw(self, msg):
		if self.if_plot:
			# self.if_new_msg = True
			data = np.asarray(msg.data)
			data_packet = data.reshape(msg.layout.dim[0].size, 1)
			self.plot_data.append(data_packet)
			arr = np.asarray(self.plot_data)
			print('Arr Shape: {}'.format(arr.shape))
			arr_proc = arr.reshape(arr.shape[0], arr.shape[1])
			# print(arr_proc.shape)
			print('Plot Arr Len: {}'.format(len(self.plot_data)))
			print('Arr Proc Shape: {}'.format(arr_proc.shape))
			if arr_proc.shape[0] > 3*30:
				self.plot_data.pop(0)
			self.arr_proc = arr_proc
			self.update = True

	def make_raw_data_plot(self):
		if self.fig is None:
			# plt.ion()
			self.fig = plt.figure()
		if self.update:
			plt.clf()
			plt.cla()
			for i in range(len(self.ch)):
				try:
					ax = self.fig.add_subplot(((len(self.ch)) * 100) + ((1) * 10) + ((i + 1)))
					ax.plot(self.arr_proc[:, self.ch[i] - 1])
					ax.set_ylim([0, 1200])
				except IndexError:
					pass

			self.fig.canvas.draw()
			self.update = False
			print(self.arr_proc.shape)
			print("Plotting")

	def extracted_features_callback(self, msg):

		data = np.asarray(msg.data)
		data_packet = data.reshape(msg.layout.dim[0].size)
		self.data_packet = data_packet.copy()

		data_packet = np.append(data_packet, self.label)
		self.data_features.append(data_packet)

		self.new_packet = True

	def write_features_data(self, data, filename):
		print(len(data))
		if len(data):
			if self.append_timestamp:
				filename = filename.split('.csv')[0] + ' ' + datetime.datetime.now().strftime("%Y-%m-%d-%H%M%S") + '.csv'
			else:
				filename = filename.split('.csv')[0] + '.csv'

			if len(data):
				print('Writing Data')
				d = np.asarray(data)
				dd = np.reshape(d[0], [1, d[0].shape[0]])
				for i in range(d.shape[0] - 1):
					dd = np.concatenate((dd, np.reshape(d[i + 1], [1, d[i + 1].shape[0]])), 0)
				# concatenate the load cell information to the other features
				# dd = np.concatenate((dd, self.data_load_cell), 0)
				df = pd.DataFrame(data=dd)
				print(self.data_write_path)
				df.to_csv(os.path.join(self.data_write_path, filename), index=False)
				print('Filename: {}, Rows: {}, Cols: {}'.format(filename, dd.shape[0], dd.shape[1]))

	def write_data(self, req):
		"""
		command to call: rosservice call /write_data
		"""
		# Pause Force data appending
		self.pause_reading_force = True

		filename = req.filename
		if self.append_timestamp:
			filename_features = filename.split('.csv')[0] + ' ' + datetime.datetime.now().strftime(
				"%Y-%m-%d-%H%M%S") + '_features.csv'
		else:
			filename_features = filename.split('.csv')[0] + '_features.csv'

		print('\nFilename: {}'.format(filename))
		self.write_features_data(self.data_features, filename_features)
		self.data_features = []

		msg = "Data Written to File: "  + filename_features
		return write_dataResponse(success=True, message=msg)

	def clear_data_array(self, req):
		"""
		command to call: rosservice call /clear_data_array
		"""
		self.data_raw = []
		self.data_features = []
		self.load_cell_arr = []
		msg = "Data Array Cleared, Data_Raw Length: " + str(len(self.data_raw)) + \
			" Data_Features Length: " + str(len(self.data_features))
		return TriggerResponse(success=True, message=msg)

	def run_classifier(self):
		t = time.time()
		while not rospy.is_shutdown():
			# print(self.label)
			if self.if_plot:
				self.make_raw_data_plot()
			else:
				if self.fig is not None:
					self.fig = None
					plt.close('all')

			if self.new_packet:
				if self.prediction:
					p = self.learning_model.predict(np.reshape(self.data_packet, [1, self.data_packet.shape[0]]))
					print(p[0])
					self.pred_window.append(p[0])

					if len(self.pred_window) >= self.pred_window_val:
						vote = max(set(self.pred_window), key=self.pred_window.count)
						self.pred_window.pop(0)

						if vote == 0:
							print('Rest')
						elif vote == 1:
							print('Pinch')
						elif vote == 2:
							print('Tripod')
						elif vote == 3:
							print('Power')
						elif vote == 4:
							print('Extension')

					self.new_packet = False

		print('\nElapsed Time: ', time.time()-t)


if __name__ == '__main__':
	A = FSRClient()
	A.run_classifier()
