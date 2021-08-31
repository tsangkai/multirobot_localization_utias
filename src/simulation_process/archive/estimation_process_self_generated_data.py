import numpy as np
from numpy import random 
from numpy import matrix
from math import cos, sin, atan2, sqrt
import os
import sys
from robot_parameters import *


d_max = 25
delta_t = 0.5


class Landmark:
	def __init__(self, index, position):
		self.index = index
		self.position = position


def within_map(loc):
	center_point = [0,0]
	if sqrt(pow(loc[0]-center_point[0], 2)+pow(loc[1]-center_point[1], 2)) > d_max:
		return False
	else:
		return True

def generate_propgation_data(current_loc, current_orientation):
	# assuming orientation is perfect 
	[meas_v, meas_a_v] = [random.uniform(-max_v,max_v), random.uniform(-max_a_v, max_a_v)]
	actual_v  = meas_v + random.normal(0, sqrt(var_u_v))
	pre_update_position = [current_loc[0] + cos(current_orientation)*actual_v*delta_t, current_loc[1] + sin(current_orientation)*actual_v*delta_t]

	while(not within_map(pre_update_position)):
		[meas_v, meas_a_v] = [random.uniform(-max_v,max_v), random.uniform(-max_a_v, max_a_v)]
		actual_v  = meas_v + random.normal(0, sqrt(var_u_v))
		pre_update_position = [current_loc[0] + cos(current_orientation)*actual_v*delta_t, current_loc[1] + sin(current_orientation)*actual_v*delta_t]
	
	orientation = current_orientation + meas_a_v*delta_t
	actual_loc = pre_update_position
	return [actual_v, meas_v, orientation, actual_loc]

def generate_measurement_data(observer_loc, observee_loc, observer_orientation):
	delta_x = observer_loc[0] - observee_loc[0]
	delta_y = observer_loc[1] - observee_loc[1]
	dis = sqrt(delta_y*delta_y + delta_x*delta_x) + random.normal(0, sqrt(var_dis))
	bearing = atan2(delta_y, delta_x) + random.normal(0, sqrt(var_angle)) - observer_orientation
	return [dis, bearing]

def estimation_process_multirobot_generated_data(robots_list, iteration, duration):
	random.seed( 9 )
	sigma_tr_arr = [0] * duration
	sigma_th_tr_arr = [0] * duration
	error_arr = [0] * duration

	for i in range(iteration):
		print 'iteration CSK'
		print i

		initial_state = matrix([1, 1, 1, 2, 2, 1, -1, -1, 1, 3], dtype=float).T
		robots_act_loc = initial_state
		initial_oriantation = matrix([0, 0, 0, 0, 0], dtype=float).T
		for j, robot in enumerate(robots_list): 
			robot.set_state(initial_state)
			robot.set_orientations(initial_oriantation)

		M = 1 # number of landmark
		landmarks = [None] * M
		for m in range(M):
			landmarks[m] = Landmark(m, matrix([0.01, 0.02], dtype=float).getT())
		
		for t in range(duration):
			for j, robot in enumerate(robots_list): 
				[act_v, meas_v, orientation, actual_loc] = generate_propgation_data(robot.get_pos(), robot.get_own_orientation())
				robots_act_loc[j:j+2] = actual_loc
				propagation_data = [delta_t, meas_v, orientation]
				[s, orinetations, sigma_s] = robot.state_update('propagation update', propagation_data)
				#th_sigma_s = robot.bound_update('propagation update', propagation_data)

			#robot 0 observes landmark
			robot_idx = 0
			[measurment_range, bearing] = generate_measurement_data(robots_list[robot_idx].get_pos(), landmarks[0].position, robots_list[robot_idx].get_own_orientation())
			landmark_measeurement_data = [landmarks[0].position, measurment_range, bearing]
			[s, orinetations, sigma_s] = robots_list[robot_idx].state_update('landmark observation update', landmark_measeurement_data)
			#th_sigma_s = robot.state_update('landmark observation update', landmark_measeurement_data)


			# for robot 2, it obverses robot 0 and robot 1 
			robot_idx = 2
			observee_idx = 0
			[measurment_range, bearing] = generate_measurement_data(robots_list[robot_idx].get_pos(), robots_list[observee_idx].get_pos(), robots_list[robot_idx].get_own_orientation())		
			relative_measeurement_data = [observee_idx , measurment_range, bearing]
			[s, orinetations, sigma_s] = robots_list[robot_idx].state_update('relative observation update', relative_measeurement_data)
			#th_sigma_s = robot.state_update('relative observation update', relative_measeurement_data)


			observee_idx = 1
			[measurment_range, bearing] = generate_measurement_data(robots_list[robot_idx].get_pos(), robots_list[observee_idx].get_pos(), robots_list[robot_idx].get_own_orientation())		
			relative_measeurement_data = [observee_idx , measurment_range, bearing]
			[s, orinetations, sigma_s] = robots_list[robot_idx].state_update('relative observation update', relative_measeurement_data)
			#th_sigma_s = robot.bound_update('relative observation update', relative_measeurement_data)

			# for robot 3, it observes landmarks and robot 4
			robot_idx = 3
			[measurment_range, bearing] = generate_measurement_data(robots_list[robot_idx].get_pos(), landmarks[0].position, robots_list[robot_idx].get_own_orientation())
			landmark_measeurement_data = [landmarks[0].position, measurment_range, bearing]
			[s, orinetations, sigma_s] = robots_list[robot_idx].state_update('landmark observation update', landmark_measeurement_data)
			#th_sigma_s = robot.bound_update('landmark observation update', landmark_measeurement_data)

			observee_idx = 4
			[measurment_range, bearing] = generate_measurement_data(robots_list[robot_idx].get_pos(), robots_list[observee_idx].get_pos(), robots_list[robot_idx].get_own_orientation())		
			relative_measeurement_data = [observee_idx, measurment_range, bearing]
			[s, orinetations, sigma_s] = robots_list[robot_idx].state_update('relative observation update', relative_measeurement_data)
			#th_sigma_s = robot.bound_update('relative observation update', relative_measeurement_data)

			# communication
			#robot 2 get comm info from robot 3
			sender_idx = 3 
			robot_idx = 2
			comm_data = [robots_list[sender_idx].get_status()[0], robots_list[sender_idx].get_status()[2]]
			[s, orinetations, sigma_s] = robots_list[robot_idx].state_update('communication', comm_data)
			th_comm_data = [robots_list[sender_idx].get_status()[0], robots_list[sender_idx].get_th_state_variance()]
			#th_sigma_s = robots_list[robot_idx].bound_update('communication', comm_data)

			#robot 0 get comm info from robot 2
			sender_idx = 2
			robot_idx = 0
			comm_data = [robots_list[sender_idx].get_status()[0], robots_list[sender_idx].get_status()[2]]
			[s, orinetations, sigma_s] = robots_list[robot_idx].state_update('communication', comm_data)
			th_comm_data = [robots_list[sender_idx].get_status()[0], robots_list[sender_idx].get_th_state_variance()]
			th_sigma_s = robots_list[robot_idx].bound_update('communication', comm_data)


			# error calculation
			s = 0
			for j, robot in enumerate(robots_list): 
				s += pow(robot.get_pos()[0] - robots_act_loc[j],2) + pow(robot.get_pos()[0] - robots_act_loc[j+1],2)
			
			s = sqrt(s*0.2)
			error_arr[t] = error_arr[t] + s*(1/float(iteration))

			# covariance error
			sigma_tr_arr[t] = sigma_tr_arr[t] + sqrt(0.2*robots_list[0].get_trace_state_variance())*(1/float(iteration))

			#sigma_th_tr_arr[t] = sigma_th_tr_arr[t] + math.sqrt(0.2*robots_list[0].get_bound)*(1/float(iteration))


	return [error_arr, sigma_tr_arr] 

