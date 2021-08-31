import numpy as np
import numpy.matlib
import math
import os
import sys
from numpy import linalg as LA
from random import randint


sys.path.insert(0, 'Environmen_for_Localization_Simulation/functions/algo_EKF')

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

	return next_loc

def estimation_process_multirobot(landmark_map, start_time, start_state_arr, measurement_data, odometry_data, robot_list, duration, robots_valid_lm_obser, comm_fail_rate):
	
	algo_name = robot_list[0].get_algo_name()
	dataset_labels = robot_list[0].get_labels_set()
	general_estimation_file_name = 'est' +  algo_name


	print("Estimation process Start: "+algo_name)
	num_comm = 0

	robots_invalid_lm_obser = list(set(dataset_labels) - set(robots_valid_lm_obser))


	est_result_file_arr = []
	for i, robot in enumerate(robot_list): #text files to keep track of the robots' paths
	 	estimation_file_name = general_estimation_file_name+'%d' %robot.get_label()
		file="/Results_files/estimation_results/"+estimation_file_name+".txt"
		path=os.getcwd()+file
		est_result_file = open(path, "w")
		est_result_file_arr.append(est_result_file)

	num_robot = len(robot_list)
	est_result_array = [[] for i in range(num_robot)]

	# use loc array to keep track of the location(which line)in the data files
	odo_req_loc_dic = {} 
	meas_req_loc_dic = {}

	#feeding initialization info into each robot and request array
	#req_loc_dic= {robot_index: [loc, time]}

	for i, robot in enumerate(robot_list): 
		start_loc = 1
		odo_req_loc_dic[i] = [start_loc, odometry_data[i][start_loc]['time']]
		start_loc = 0
		meas_req_loc_dic[i] = [start_loc, measurement_data[i][start_loc]['time']]

		#[s, orinetations, sigma_s] = robot.get_status()
		sigma_s_trace = robot.get_trace_state_variance()
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
			delta_t = odometry_data[odo_index][odo_loc]['time'] - odometry_data[odo_index][odo_loc-1]['time'] #look at the previous time stamp to find the time interval
			velocity = odometry_data[odo_index][odo_loc]['velocity']
			orientation = odometry_data[odo_index][odo_loc]['orientation']
			propagation_data = [delta_t, velocity, orientation]
			[s, orinetations, sigma_s] = robot_list[odo_index].state_update('propagation update', propagation_data)

			#recording datas
			j = odo_index * 2 
			robot_pos = s[j:j+2,0]
			sigma_s_trace = robot_list[odo_index].get_trace_state_variance()

			est_result_file_arr[odo_index].write(str(t)+ '\t'+ str(robot_pos.item(0,0)) + '\t' + str(robot_pos.item(1,0)) + '\t' +str(sigma_s_trace) + '\n')
			est_result = {'time':t, 'x_pos':robot_pos.item(0,0), 'y_pos':robot_pos.item(1,0), 'sigma_s_trace':sigma_s_trace}
			est_result_array[odo_index].append(est_result)

			update_loc_dic(odo_index, odo_req_loc_dic, odometry_data)

		else:
			#measurement update
			pass
			
			t = meas_time
			measurment_range = measurement_data[meas_index][meas_loc]['measurment_range']
			subject_ID = measurement_data[meas_index][meas_loc]['subject_ID']
			bearing = measurement_data[meas_index][meas_loc]['bearing']
			
			if int(subject_ID) > 5:
				pass
				#landmark observation
				if meas_index+1 in robots_valid_lm_obser: 
					landmark_loc = landmark_map[subject_ID]
					landmark_measeurement_data = [landmark_loc, measurment_range, bearing]
					[s, orinetations, sigma_s] = robot_list[meas_index].state_update('landmark observation update', landmark_measeurement_data)
				
					#recording datas
					j = meas_index * 2
					robot_pos = s[j:j+2,0]
					sigma_s_trace = robot_list[meas_index].get_trace_state_variance()

					est_result_file_arr[meas_index].write(str(t)+ '\t'+ str(robot_pos.item(0,0)) + '\t' + str(robot_pos.item(1,0)) + '\t' +str(sigma_s_trace) + ' lm_obser\n')
					est_result = {'time':t, 'x_pos':robot_pos.item(0,0), 'y_pos':robot_pos.item(1,0), 'sigma_s_trace':sigma_s_trace}
					est_result_array[meas_index].append(est_result)


			else:
				pass
				#relative observation
				observee_index = int(subject_ID)-1
				relative_measeurement_data = [observee_index, measurment_range, bearing]
				[s, orinetations, sigma_s] = robot_list[meas_index].state_update('relative observation update', relative_measeurement_data)

				#recording datas
				j = meas_index * 2
				robot_pos = s[j:j+2,0]
				sigma_s_trace = robot_list[meas_index].get_trace_state_variance()

				est_result_file_arr[meas_index].write(str(t)+ '\t'+ str(robot_pos.item(0,0)) + '\t' + str(robot_pos.item(1,0)) + '\t' +str(sigma_s_trace) + ' rel_obser\n')
				est_result = {'time':t, 'x_pos':robot_pos.item(0,0), 'y_pos':robot_pos.item(1,0), 'sigma_s_trace':sigma_s_trace}
				est_result_array[meas_index].append(est_result)


				if (meas_index+1 in robots_invalid_lm_obser) and (observee_index+1 in robots_valid_lm_obser) and (np.random.uniform() > comm_fail_rate):
					comm_robot_index = meas_index
					sender_robot_index = observee_index


					[sender_s, sender_orientaions, sender_state_variance] = robot_list[sender_robot_index].get_status()
					comm_data = [sender_s, sender_state_variance]

					[s, orinetations, sigma_s] = robot_list[comm_robot_index].state_update('communication', comm_data)

					#recording datas 
					j = comm_robot_index * 2
					robot_pos = s[j:j+2,0]
					sigma_s_trace = robot_list[comm_robot_index].get_trace_state_variance()

					est_result_file_arr[comm_robot_index].write(str(t)+ '\t'+ str(robot_pos.item(0,0)) + '\t' + str(robot_pos.item(1,0)) + '\t' +str(sigma_s_trace) + ' comm w/ robot' + str(meas_index+1) +'\n')
					est_result = {'time':t, 'x_pos':robot_pos.item(0,0), 'y_pos':robot_pos.item(1,0), 'sigma_s_trace':sigma_s_trace}
					est_result_array[comm_robot_index].append(est_result)

					num_comm+=1
				
			update_loc_dic(meas_index, meas_req_loc_dic, measurement_data)
		

	for j in range(len(robot_list)):
		est_result_file_arr[j].close()


	print ("Estimation Process Finished 	# of comm: " + str(num_comm))

	return est_result_array