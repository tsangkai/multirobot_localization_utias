
import numpy as np
import numpy.matlib
import math
import os
import sys
#from numpy import linalg as LA
from random import randint

sys.path.insert(0, 'Environmen_for_Localization_Simulation/functions/algo_EKF')

class global_time():
	"""docstring for """
	def __init__(self, start_time, robot_list):
		self.all_possible_freqs = np.array([[round(f, 3) for f in robot.get_freqs()] for robot in robot_list])
		self.all_possible_next_time = np.array([[round(start_time+1/f, 3) for f in robot.get_freqs()] for robot in robot_list])
	def get_time(self):
		self.least_index = np.unravel_index(np.argmin(self.all_possible_next_time), self.all_possible_next_time.shape)
		return [self.least_index, self.all_possible_next_time[self.least_index]] #[index, time]
	def get_all_freqs(self):
		return self.all_possible_freqs
	def time_update(self):
		self.all_possible_next_time[self.least_index] = self.all_possible_next_time[self.least_index] + 1/self.all_possible_freqs[self.least_index]


def find_nearest_time_idx_on_data_array(data_array, start_idx, time_target):
	"""only for increasing order array"""
	i = start_idx
	if data_array[i].get('time') < time_target:
		while data_array[i].get('time') < time_target:
			i = i+1
		if data_array[i].get('time')-time_target < time_target-data_array[i-1].get('time'):
			return i
		else:
			return i-1
	else: 
		return i 

def get_robot_actual_loc(groundtruth_data, groundtruth_indices, robot_index, current_time):
	"""find the actual location at a given time from groudtruth data"""
	gt_index = find_nearest_time_idx_on_data_array(groundtruth_data[robot_index], groundtruth_indices[robot_index], current_time) #find the index on odometry data with closest time
	robot_pos = [groundtruth_data[robot_index][gt_index].get('x_pos'), groundtruth_data[robot_index][gt_index].get('y_pos'), groundtruth_data[robot_index][gt_index].get('orientation')]
	groundtruth_indices[robot_index] = gt_index
	return [robot_pos, groundtruth_indices]

def generate_observation_data(observer_pos, observee_pos, measurement_deviation):
	[x_1, y_1, thera_1] = observer_pos
	[x_2, y_2, thera_2] = observee_pos
	distance = math.sqrt((x_1-x_2)*(x_1-x_2)+(y_1-y_2)*(y_1-y_2)) + np.random.normal(0, measurement_deviation)
	bearing = math.atan2((y_2-y_1),(x_2-x_1)) - thera_1 + np.random.normal(0, measurement_deviation)
	return[distance, bearing]


def estimation_process_cont_multirobot(landmark_map, start_time, start_state_arr, measurement_data, odometry_data, groundtruth_data, robot_list, duration, robots_valid_lm_obser, comm_fail_rate):
	print landmark_map
	algo_name = robot_list[0].get_algo_name()
	dataset_labels = robot_list[0].get_labels_set()
	general_estimation_file_name = 'est' +  algo_name


	print("Estimation process Start: "+algo_name)
	num_comm = 0

	robots_invalid_lm_obser = list(set(dataset_labels) - set(robots_valid_lm_obser))

	num_robot = len(robot_list)
	odometry_indices = np.zeros(num_robot, dtype = np.int)  # bookkeeping a pointer for each robot for odometry data for easier searching
	groundtruth_indices = np.zeros(num_robot, dtype = np.int)
	
	est_result_array = [[] for i in range(num_robot)]


	for i, robot in enumerate(robot_list): 
		robot_freq = [20,5,1]
		robot.set_freqs(robot_freq)
		print robot_freq
		
		#sigma_s_trace = robot.get_trace_state_variance()
		#est_result = {'time':start_time, 'x_pos':start_state_arr[i][0], 'y_pos':start_state_arr[i][1], 'sigma_s_trace':sigma_s_trace}
		#est_result_array[i].append(est_result)
	#start_time = 0
	envir_time = global_time(start_time, robot_list)
	all_possible_freqs = envir_time.get_all_freqs()
	[current_index, current_time] = envir_time.get_time() 

	print 'current_index'
	print current_index
	
	measurment_range_arr = []
	measurment_range = 0
	measurement_deviation = 0.2
	while current_time < start_time + duration:
		robot_index = current_index[0] # determind which robot to perform opertations
		if current_index[1] == 0: #odometry update
			pass
			
			od_index = find_nearest_time_idx_on_data_array(odometry_data[robot_index], odometry_indices[robot_index], current_time) #find the index on odometry data with closest time
			odometry_indices[robot_index] = od_index
			delta_t = 1/all_possible_freqs[robot_index][0] # time interval = 1/freq_odometry
			velocity = odometry_data[robot_index][od_index].get('velocity')
			orientation = odometry_data[robot_index][od_index].get('orientation')

			propagation_data = [delta_t, velocity, orientation]
			[s, orinetations, sigma_s] = robot_list[robot_index].state_update('propagation update', propagation_data)
			th_sigma_s = robot_list[robot_index].bound_update('propagation update', propagation_data)	
			

			#record the data
			robot_pos = robot_list[robot_index].get_pos()
			sigma_s_trace = robot_list[robot_index].get_trace_state_variance()
			bound = robot_list[robot_index].get_bound()
			est_result = {'time':current_time, 'x_pos':robot_pos.item(0,0), 'y_pos':robot_pos.item(1,0), 'sigma_s_trace':sigma_s_trace, 'bound': bound}
			#print est_result
			est_result_array[robot_index].append(est_result)
			
			'''
			if robot_index == 0:
				print(str(robot_index) + ' prop')
				print est_result
			'''
		
		elif current_index[1] == 1: #observation update
			[self_loc, groundtruth_indices] = get_robot_actual_loc(groundtruth_data, groundtruth_indices, robot_index, current_time)
			
			#landmark observation
			subject_ID = '15'
			if robot_index == 0:
				landmark_loc = landmark_map[subject_ID]
				landmark_loc = [landmark_loc[0], landmark_loc[1], 0] #landmark doesn't have orinataion here
				[measurment_range, bearing] = generate_observation_data(self_loc, landmark_loc, measurement_deviation)
				landmark_measeurement_data = [landmark_loc, measurment_range, bearing]
				[s, orinetations, sigma_s] = robot_list[robot_index].state_update('landmark observation update', landmark_measeurement_data)
				th_sigma_s = robot_list[robot_index].bound_update('landmark observation update', landmark_measeurement_data)
				'''
				#relative observation
				target_robot_index = 1
				[target_loc, groundtruth_indices] = get_robot_actual_loc(groundtruth_data, groundtruth_indices, target_robot_index, current_time)
				[measurment_range, bearing] = generate_observation_data(self_loc, target_loc, measurement_deviation)
				relative_measeurement_data = [target_robot_index, measurment_range, bearing]
				[s, orinetations, sigma_s] = robot_list[robot_index].state_update('relative observation update', relative_measeurement_data)
				th_sigma_s = robot_list[robot_index].bound_update('relative observation update', relative_measeurement_data)
				'''
				#record the data
				robot_pos = robot_list[robot_index].get_pos()
				sigma_s_trace = robot_list[robot_index].get_trace_state_variance()
				bound = robot_list[robot_index].get_bound()
				est_result = {'time':current_time, 'x_pos':robot_pos.item(0,0), 'y_pos':robot_pos.item(1,0), 'sigma_s_trace':sigma_s_trace, 'bound': bound}
				est_result_array[robot_index].append(est_result)
				
				#print(str(robot_index) + ' obser')
				#print est_result
		'''		
		else: #communication update
			pass
			
			if robot_index == 1: # 2nd robot
				target_robot_index = 2
				[target_loc, groundtruth_indices] = get_robot_actual_loc(groundtruth_data, groundtruth_indices, target_robot_index, current_time)
				[measurment_range, bearing] = generate_observation_data(self_loc, target_loc, measurement_deviation)
				relative_measeurement_data = [target_robot_index, measurment_range, bearing]
				[s, orinetations, sigma_s] = robot_list[robot_index].state_update('relative observation update', relative_measeurement_data)
				th_sigma_s = robot_list[robot_index].bound_update('relative observation update', relative_measeurement_data)
			
				#record the data
				robot_pos = robot_list[robot_index].get_pos()
				sigma_s_trace = robot_list[robot_index].get_trace_state_variance()
				bound = robot_list[robot_index].get_bound()
				est_result = {'time':current_time, 'x_pos':robot_pos.item(0,0), 'y_pos':robot_pos.item(1,0), 'sigma_s_trace':sigma_s_trace, 'bound': bound}
				est_result_array[robot_index].append(est_result)
				

				sender_robot_index = robot_index
				receiver_index = 0
				[sender_s, sender_orientaions, sender_state_variance] = robot_list[sender_robot_index].get_status()
				comm_data = [sender_s, sender_state_variance]
				[s, orinetations, sigma_s] = robot_list[receiver_index].state_update('communication', comm_data)
				comm_data = robot_list[sender_robot_index].get_th_state_variance()
				th_sigma_s = robot_list[receiver_index].bound_update('communication', comm_data)
				
				#record the data
				robot_pos = robot_list[receiver_index].get_pos()
				sigma_s_trace = robot_list[receiver_index].get_trace_state_variance()
				bound = robot_list[receiver_index].get_bound()
				est_result = {'time':current_time, 'x_pos':robot_pos.item(0,0), 'y_pos':robot_pos.item(1,0), 'sigma_s_trace':sigma_s_trace, 'bound': bound}
				est_result_array[receiver_index].append(est_result)
				
				#print(str(robot_index) + ' comm')
				#print est_result
		'''
		measurment_range_arr.append(measurment_range)
		prev_time = current_time
		envir_time.time_update()
		[current_index, current_time] = envir_time.get_time() 
		if prev_time > current_time:
			print('prev: ' + str(prev_time) + '; current' + str(current_time))
			sys.exit("incorrect time handling!")

	#print ("Estimation Process Finished 	# of comm: " + str(num_comm))
	print 'measurment_range_arr'
	print max(measurment_range_arr)
	return est_result_array