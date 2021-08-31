import numpy as np
import numpy.matlib
import math
import os
import sys
import random
from numpy import linalg as LA

sys.path.insert(0, 'Environmen_for_Localization_Simulation/functions/algo_EKF')

import class_centralized_system

def is_pos_def(x):
	return np.all(np.linalg.eigvals(x) > 0)

def find_min_match_dataline(loc_dic, datas):

	min_index = min(loc_dic, key=lambda k: loc_dic[k][1])
	min_loc = loc_dic[min_index][0]
	return [min_index, min_loc]

def update_loc_dic(robot_index, loc_dic, datas):
	next_loc = loc_dic[robot_index][0] + 1 #next location if possible in the data file
	try:
		loc_dic[robot_index] = [next_loc, datas[robot_index][next_loc]['time']]
	except IndexError:
		#print 'delect the robot from request list'
		del loc_dic[robot_index]

	#return next_loc
	return 1

def estimation_process_centralized(landmark_map, start_time, start_state_arr, measurement_data, odometry_data, centralized_system, duration, robots_valid_lm_obser, comm_fail_rate):
	
	algo_name = centralized_system.get_algo_name()
	dataset_labels = centralized_system.get_robot_labels()
	general_estimation_file_name = 'est' +  algo_name
	
	print("Estimation process Start: "+algo_name)

	num_rela_obser = 0

	robots_invalid_lm_obser = list(set(dataset_labels) - set(robots_valid_lm_obser))

	est_result_file_arr = []
	for i, robot_label in enumerate(dataset_labels): #text files to keep track of the robots' paths
	 	estimation_file_name = general_estimation_file_name+'%d' %robot_label
		file="/Results_files/estimation_results/"+estimation_file_name+".txt"
		path=os.getcwd()+file
		est_result_file = open(path, "w")
		est_result_file_arr.append(est_result_file)

	num_robots = len(dataset_labels)
	est_result_array = [[] for i in range(num_robots)]

	# use loc dicrionary to keep track of the location(which line)in the data files
	odo_req_loc_dic = {} 
	meas_req_loc_dic = {}

	#feeding initialization info into each robot and request array
	#req_loc_dic= {robot_index: [loc, time]}

	for i in range(num_robots): 
		start_loc = 1
		odo_req_loc_dic[i] = [start_loc, odometry_data[i][start_loc]['time']]
		start_loc = 0
		meas_req_loc_dic[i] = [start_loc, measurement_data[i][start_loc]['time']]

	
	# recording starting state 
	#[s, orinetations, sigma_s] = centralized_system.get_status()
	for i in range(num_robots): 
		sigma_s_trace = centralized_system.get_trace_state_variance(i)
		est_result_file_arr[i].write(str(start_time)+ '\t'+ str(start_state_arr[i][0]) + '\t' + str(start_state_arr[i][1]) + '\t' +str(sigma_s_trace) + '\n')
		est_result = {'time':start_time, 'x_pos':start_state_arr[i][0], 'y_pos':start_state_arr[i][1], 'sigma_s_trace':sigma_s_trace}
		est_result_array[i].append(est_result)

	t = start_time
	while t < start_time + duration and len(odo_req_loc_dic) != 0 and len(meas_req_loc_dic) != 0: 
		[odo_index, odo_loc] = find_min_match_dataline(odo_req_loc_dic, odometry_data)
		odo_time = odometry_data[odo_index][odo_loc]['time']
		[meas_index, meas_loc] = find_min_match_dataline(meas_req_loc_dic, measurement_data)
		meas_time = measurement_data[meas_index][meas_loc]['time']

		if odo_time <= meas_time:
			#propagtion update
			t = odo_time
			delta_t = odometry_data[odo_index][odo_loc]['time'] - odometry_data[odo_index][odo_loc-1]['time'] #look the the previous like to find time interval
			velocity = odometry_data[odo_index][odo_loc]['velocity']
			orientation = odometry_data[odo_index][odo_loc]['orientation']
			propagation_data = [delta_t, velocity, orientation]
			[s, orinetations, sigma_s] = centralized_system.state_update('propagation update', propagation_data, odo_index)


			#recording datas
			j = odo_index * 2
			robot_pos = s[j:j+2,0]
			sigma_s_trace = centralized_system.get_trace_state_variance(odo_index)

			est_result_file_arr[odo_index].write(str(t)+ '\t'+ str(robot_pos.item(0,0)) + '\t' + str(robot_pos.item(1,0)) + '\t' +str(sigma_s_trace) + '\n')
			est_result = {'time':t, 'x_pos':robot_pos.item(0,0), 'y_pos':robot_pos.item(1,0), 'sigma_s_trace':sigma_s_trace}
			est_result_array[odo_index].append(est_result)

			update_loc_dic(odo_index, odo_req_loc_dic, odometry_data)

		else:
			pass
			#measurement update
			t = meas_time
			subject_ID = measurement_data[meas_index][meas_loc]['subject_ID']
			measurment_range = measurement_data[meas_index][meas_loc]['measurment_range']
			bearing = measurement_data[meas_index][meas_loc]['bearing']
			
			if int(subject_ID) > 5:
				pass
				#landmark observation
				if meas_index+1 in robots_valid_lm_obser: 
					landmark_loc = landmark_map[subject_ID]
					landmark_measeurement_data = [landmark_loc, measurment_range, bearing]

					[s, orinetations, sigma_s] = centralized_system.state_update('landmark observation update', landmark_measeurement_data, meas_index)
					
					#recording datas
					j = meas_index * 2
					robot_pos = s[j:j+2,0]
					sigma_s_trace = centralized_system.get_trace_state_variance(meas_index)

					est_result_file_arr[meas_index].write(str(t)+ '\t'+ str(robot_pos.item(0,0)) + '\t' + str(robot_pos.item(1,0)) + '\t' +str(sigma_s_trace) + ' obser\n')
					est_result = {'time':t, 'x_pos':robot_pos.item(0,0), 'y_pos':robot_pos.item(1,0), 'sigma_s_trace':sigma_s_trace}
					est_result_array[meas_index].append(est_result)

			else:
				#relative observation
				
				obser_index = int(subject_ID)-1
				relative_measeurement_data = [obser_index, measurment_range, bearing]
				
				if ((algo_name == 'LS-Cen') or ((meas_index+1 in robots_invalid_lm_obser) and (obser_index+1 in robots_valid_lm_obser) and (np.random.uniform() > comm_fail_rate))): 
					[s, orinetations, sigma_s] = centralized_system.state_update('relative observation update', relative_measeurement_data, meas_index)
					
					#recording datas
					j = meas_index * 2
					robot_pos = s[j:j+2,0]
					sigma_s_trace = centralized_system.get_trace_state_variance(meas_index)

					est_result_file_arr[meas_index].write(str(t)+ '\t'+ str(robot_pos.item(0,0)) + '\t' + str(robot_pos.item(1,0)) + '\t' +str(sigma_s_trace) + ' obser\n')
					est_result = {'time':t, 'x_pos':robot_pos.item(0,0), 'y_pos':robot_pos.item(1,0), 'sigma_s_trace':sigma_s_trace}
					est_result_array[meas_index].append(est_result)
					num_rela_obser +=1

					
				else:
					pass
				
			update_loc_dic(meas_index, meas_req_loc_dic, measurement_data)	
			
	for j in range(num_robots):
		est_result_file_arr[j].close()

	print ("Estimation Process Finished			# of relative observations: " + str(num_rela_obser))

	return est_result_array