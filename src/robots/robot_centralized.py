# coding: utf-8
import numpy as np
import math
import os
import sys

class CentralizedRobotSystem():

	def __init__(self, name, dataset_labels, localization_algo):
		#label can choose which robot in the dataset (1-5), 0 is default
		#list index in the index of the robot in the robot_list
		self.name = name
		self.dataset_labels = dataset_labels #subject IDs
		self.num_robots = len(dataset_labels)
		self.state = np.matrix(np.zeros((2*self.num_robots,1)))
		self.orientaions = np.matrix(np.zeros((self.num_robots,1)))		
		self.localization_algo = localization_algo
		self.state_variance  = 0


	def set_starting_state(self, start_state_arr):
		for i, label in enumerate(self.dataset_labels):
			self.time = start_state_arr[label][0]
			self.state[2*i,0] = start_state_arr[label][1] #for x-axis 
			self.state[2*i+1,0] = start_state_arr[label][2] #for y-axis 
			self.orientaions[i, 0] = start_state_arr[label][3] #for orientation 
		self.state_variance  = self.localization_algo.state_variance_init(self.num_robots)
		


	def set_time(self, time):
		self.time = time

	def set_start_moving_times(self, start_moving_times): 
		self.prev_prop_times = start_moving_times

	def set_state(self, state):
		self.state = state

	def set_state_var(self, sigma_s):
		self.state_variance = sigma_s
		

	def set_orientations(self, orientaions):
		self.orientaions = orientaions
		

	def get_name(self):
		return self.name
	
	def get_robot_labels(self):
		return self.dataset_labels

	def get_status(self):
		return [self.state, self.orientaions, self.state_variance]

	def get_trace_state_variance(self, robot_index):
		robot_data = [self.state, self.orientaions, self.state_variance, robot_index]
		return self.localization_algo.calculate_trace_state_variance(robot_data)

	def load_map(self, landmark_map): 
		self.landmark_map = landmark_map

	def state_update(self, robot_index, update_type, sensor_data):
		robot_data = [self.state, self.orientaions, self.state_variance, robot_index]
		[self.state, self.orientaions, self.state_variance] = self.localization_algo.algo_update(robot_data, update_type, sensor_data)
		return [self.state, self.orientaions, self.state_variance, update_type]






