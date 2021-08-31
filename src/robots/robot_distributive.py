import numpy as np
import math
import os
import sys
from robots.robot_parameters import *

sys.path.insert(0, 'Environmen_for_Localization_Simulation/functions/algo_EKF')
class DistributiveRobot():

	def __init__(self, labels_set, robot_index, loc_algo):
		#label can choose which robot in the dataset (1-5)
		#robot index in the index of the robot in the robot_list
		self.label = labels_set[robot_index] #subject ID
		self.labels_set = labels_set
		self.robot_index = robot_index
		self.num_robots = len(labels_set)

		
		self.orientaions = np.matrix(np.zeros((self.num_robots,1)))
		
		self.localization_algo = loc_algo # create the localization algo for the robot
		
		#self.th_state_variance = self.bound_algo.initialization_for_state_variance(self.num_robots)

		self.state = np.matrix(np.zeros((2*self.num_robots,1)))
		self.state_variance  = self.localization_algo.state_variance_init(self.num_robots)

		self.freqs = [0, 0, 0] #[f_odometery, f_observartion, f_communication]


	def set_starting_state(self, start_state_arr, known_other_start_loc = True):
		self.time = start_state_arr[self.label][0]
		if known_other_start_loc:
			for i, label in enumerate(self.labels_set):
				self.state[2*i,0] = start_state_arr[label][1] #for x-axis 
				self.state[2*i+1,0] = start_state_arr[label][2] #for y-axis 
				self.orientaions[i, 0] = start_state_arr[label][3] #for orientation 
		else:
			i = self.robot_index
			self.state[2*i,0] = start_state_arr[self.label][1] #for x-axis 
			self.state[2*i+1,0] = start_state_arr[self.label][2] #for y-axis 
			self.orientaions[i, 0] = start_state_arr[self.label][3] #for orientation 

		#self.th_state_variance  = self.bound_algo.initialization_for_state_variance(self.num_robots)
		self.state_variance  = self.localization_algo.state_variance_init(self.num_robots)


	def set_state(self, state):
		self.state = state
		
	def set_state_var(self, sigma_s):
		self.state_variance = sigma_s

	def set_orientations(self, orientaions):
		self.orientaions = orientaions
		
	def set_start_time(self, time):
		self.time = time

	def set_freqs(self, freqs):
		self.freqs = freqs

	def get_freqs(self):
		return self.freqs

	def get_name(self):
		return self.name
		
	def get_algo_name(self):
		return self.localization_algo.get_name()

	def get_status(self):
		return [self.state, self.orientaions, self.state_variance]

	def get_pos(self):
		return self.state[2*self.robot_index:2*self.robot_index+2]

	def get_th_state_variance(self):
		return self.th_state_variance

	def get_own_orientation(self):
		return self.orientaions.item(self.robot_index)

	def get_labels_set(self):
		return self.labels_set

	def get_index(self):
		return self.robot_index

	def get_label(self):
		return self.labels_set[self.robot_index]

	def get_trace_state_variance(self):
		robot_data = [self.state, self.orientaions, self.state_variance, self.robot_index]
		return self.localization_algo.calculate_trace_state_variance(robot_data)

	def get_bound(self):
		return self.bound_algo.calculate_trace_state_variance(self.th_state_variance, self.robot_index)

	def get_th_state_variance(self):
		return self.th_state_variance

	def set_start_moving_times(self, start_moving_times): 
		self.prev_prop_time = start_moving_times[self.robot_index]

	def load_map(self, landmark_map): 
		self.landmark_map = landmark_map

	def state_update(self, update_type, sensor_data):
		robot_data = [self.state, self.orientaions, self.state_variance, self.robot_index]
		[self.state, self.orientaions, self.state_variance] = self.localization_algo.algo_update(robot_data, update_type, sensor_data)
		return [self.state, self.orientaions, self.state_variance, update_type]

	'''
	def bound_update(self, update_type, measurement_data):
		if update_type == 'propagation update' : #propagation update 
			v = measurement_data[1]
			sigma_odo = np.matrix([[0.05, 0], [0, 0.01]]) #with respect to velocity and orientationb
			sensor_covariance = [sigma_odo]
		elif update_type == 'communication':
			sensor_covariance = 0
		else:
			#sigma_obser = np.matrix([[0.0215, 0], [0,0.01]]) #with respect to range and bearing
			sigma_obser = np.matrix([[0.2, 0], [0,0.2]]) 
			#sensor_covariance = sigma_obser
			sensor_covariance = [var_dis, var_angle]
			
		robot_data = [self.state, self.freqs, self.th_state_variance, self.robot_index]
		sensor_data = [measurement_data, sensor_covariance] 
		self.th_state_variance = self.bound_algo.estimation_update(robot_data, update_type, sensor_data)
		return self.th_state_variance
	'''
