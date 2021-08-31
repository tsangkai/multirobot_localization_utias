import numpy as np
from numpy import matrix
from math import cos, sin, atan2, sqrt
from localization_algo_framework import ekf_algo_framework

def rot_mtx(theta):
	return matrix([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

class GS_CI_Bound(ekf_algo_framework):
	def __init__(self, algo_name):
		self.algo_name = algo_name
		self.d_max = 2 # max measurement distance 
		self.d_var = 0.2
		self.bearing_var = 0.2
		self.var_v = 0.25

	def state_variance_init(self, num_robots):
		return 0.04*np.matrix(np.identity(2*num_robots), dtype = float)

	def calculate_trace_state_variance(self, robot_data):
		[s, orinetations, sigma_s, index] = robot_data
		i = 2*index
		trace_state_var = np.trace(sigma_s[i:i+2, i:i+2])
		return np.trace(sigma_s)

	#def propagation_update(self, s, th_sigma_s, sigma_odo, index, odo_freq):
	def propagation_update(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		sigma_odo = sensor_covariance

		num_robots = int(len(s)/2)
		i = 2*index
		#delta_t = 1/odo_freq
		delta_t = measurement_data[0]

		for j in range(num_robots):
			jj = 2*j

		if j==index:
			sigma_s[jj:jj+2, jj:jj+2] += sigma_odo[0,0]*np.identity(2)*delta_t*delta_t
		else:
			#var_v = pow(2*self.d_max, 2)/12
			var_v = self.var_v
			sigma_s[jj:jj+2, jj:jj+2] += var_v*np.identity(2)*delta_t*delta_t
		
		#return th_sigma_s
		return [s, orinetations, sigma_s]


	#def absolute_obser_update(self, s, th_sigma_s, index):
	def absolute_obser_update(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		sigma_ob = sensor_covariance

		num_robots = int(len(s)/2)
		i = 2*index
		H_i = np.matrix(np.zeros((2,2*num_robots)))
		H_i[0, i] = -1
		H_i[1, i+1] = -1

		d_max = self.d_max
		var_dis = self.d_var
		var_phi = self.bearing_var

		sigma_th_z =  np.matrix(max(var_dis, d_max*d_max*var_phi)*np.identity(2))
		sigma_s = (sigma_s.getI() + H_i.getT() * sigma_th_z.getI() * H_i).getI()

		return [s, orinetations, sigma_s]


	#def relative_obser_update(self, s, th_sigma_s, index, obser_index):
	#i obser j 
	def relative_obser_update(self, robot_data, sensor_data):
		#when robot i observes robot j 
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		sigma_ob = sensor_covariance

		num_robots = int(len(s)/2)

		i = 2*index
		obser_index = measurement_data[0] 
		j = obser_index * 2 	

		H_ij  = np.matrix(np.zeros((2,2*num_robots)))
		H_ij[0, i] = -1
		H_ij[1, i+1] = -1
		H_ij[0, j] = 1
		H_ij[1, j+1] = 1

		d_max = self.d_max
		var_dis = self.d_var
		var_phi = self.bearing_var

		sigma_th_z =  np.matrix(max(var_dis, d_max*d_max*var_phi)*np.identity(2))
		sigma_s = (sigma_s.getI() + H_ij.getT() * sigma_th_z.getI() * H_ij).getI()
		return [s, orinetations, sigma_s]


	#def communication(self, s, th_sigma_s, index, measurement_data):
	def communication(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[comm_data, comm_variance] = sensor_data
		[sender_idx, comm_robot_s, comm_robot_sigma_s]=comm_data


		e = 0.8
		
		sigma_s = ( e*sigma_s.getI() + (1-e)*comm_robot_sigma_s.getI()).getI()		# from TK's gs-ci

		return [s, orinetations, sigma_s]
