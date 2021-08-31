import numpy as np
from numpy import matrix
from math import cos, sin, atan2, sqrt
from localization_algo_framework import ekf_algo_framework

def rot_mtx(theta):
	return matrix([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])


class EKF_GS_BOUND(ekf_algo_framework):
	def __init__(self, algo_name):
		ekf_algo_framework.__init__(self, algo_name)
		self.var_v = 0.04
		self.var_range = 0.7
		self.var_bearing = 0.3
		self.max_range  = 3.5 

	def state_variance_init(self, num_robots):
		return 0.1*np.matrix(np.identity(2*num_robots), dtype = float)

	def calculate_trace_state_variance(self, robot_data):
		[s, orinetations, sigma_s, index] = robot_data
		i = 2*index
		trace_state_var = np.trace(sigma_s[i:i+2, i:i+2])
		return trace_state_var

	def propagation_update(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		sigma_odo = sensor_covariance

		var_v = self.var_v # variance of the velocity

		i = 2*index
		num_robots = int(len(s)/2)
		delta_t = measurement_data[0]
		v = measurement_data[1]
		orinetations[index] = measurement_data[2]
		self_theta = orinetations[index]

		Q = sigma_odo
		Q[1,1] = Q[1,1]*v*v
		W = delta_t*rot_mtx(self_theta)

		s[i,0] = s[i,0] + cos(self_theta)*v*delta_t   #x
		s[i+1,0] = s[i+1,0] + sin(self_theta)*v*delta_t #y

		for j in range(num_robots):
			jj = 2*j
			sigma_s[jj:jj+2, jj:jj+2] += delta_t*delta_t*var_v*np.identity(2)

		return [s, orinetations, sigma_s]

	def absolute_obser_update(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		
		sigma_ob = sensor_covariance
		num_robots = int(len(s)/2)
		self_theta = orinetations[index]
		'''
		landmark_loc = measurement_data[0] 
		dis = measurement_data[1]  
		phi = measurement_data[2]
		'''

		i = 2*index
		H_i  = np.matrix(np.zeros((2,2*num_robots)), dtype = float)
		H_i[0, i] = -1
		H_i[1, i+1] = -1
		H = rot_mtx(self_theta).getT()*H_i

		var_range = self.var_range
		var_range = self.var_range
		range_max = self.max_range
		var_bearing = self.var_bearing

		R = np.matrix(max(var_range, range_max, var_bearing)*np.identity(2))
		sigma_s = (sigma_s.getI()+H.getT()*R.getI()*H).getI()

		return [s, orinetations, sigma_s]

	
	def relative_obser_update(self, robot_data, sensor_data):
		#when robot i observes robot j 

		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		sigma_ob = sensor_covariance

		num_robots = int(len(s)/2)
		self_theta = orinetations[index]

		i = index * 2 
		obser_index = measurement_data[0] 
		#dis = measurement_data[1]  
		#phi = measurement_data[2]
		j = obser_index * 2 
		
		#z = matrix([dis*cos(phi), dis*sin(phi)]).getT()
		
		H_ij  = np.zeros((2,2*num_robots))
		H_ij[0, i] = -1
		H_ij[1, i+1] = -1
		H_ij[0, j] = 1
		H_ij[1, j+1] = 1
		H = rot_mtx(self_theta).getT()*H_ij 

		var_range = self.var_range
		var_range = self.var_range
		range_max = self.range_max
		var_bearing = self.var_bearing
		
		R = np.matrix(max(var_range, range_max, var_bearing)*np.identity(2))
		sigma_s = (sigma_s.getI()+H.getT()*R.getI()*H).getI()

		return [s, orinetations, sigma_s]

	def communication(self, robot_data, sensor_data):
		
		[s, orinetations, sigma_s, index] = robot_data
		[comm_data, comm_variance] = sensor_data
		[sender_idx, comm_robot_s, comm_robot_sigma_s]=comm_data

		#sent to robot i from robot k
		
		num_robots = int(len(s)/2)
		self_theta = orinetations[index]
		i = index * 2 
		k = sender_idx*2
		e =  0.8 # (iii+1)*0.01
		#e = comm_e

		H_k = np.zeros((2,2*num_robots))
		H_k[0, i] = -1
		H_k[1, i+1] = -1
		H_k[0, k] = 1
		H_k[1, k+1] = 1
		H = rot_mtx(self_theta).getT()*H_k 

		var_range = 0.25
		d_max = pow(0.05,2)
		var_bearing = pow(2.0 / 180, 2) 
  
		R = np.matrix(max(var_range, d_max, var_bearing)*np.identity(2))
		sig_inv = e*sigma_s.getI() + (1-e)*(H.getT()*R.getI()*H)
		sigma_s = sig_inv.getI()

		return [s, orinetations, sigma_s]