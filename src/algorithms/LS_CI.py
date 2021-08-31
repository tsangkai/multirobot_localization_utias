# coding: utf-8

#local state CI (LS-CI) - “Decentralized multi-robot cooperative localization using covariance intersection,” L. C. Carrillo-Arce, E. D. Nerurkar, J. L. Gordillo, and S. I. Roumeliotis
import numpy as np
from numpy import matrix
from numpy import linalg
from math import cos, sin, atan2, sqrt
from algorithms.EKF import ekf_algo_framework

def rot_mtx(theta):
	return matrix([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

class LS_CI(ekf_algo_framework):
	def __init__(self, algo_name):
		ekf_algo_framework.__init__(self, algo_name)

	def state_variance_init(self, num_robots):
		return 0.04*np.matrix(np.identity(2*num_robots), dtype = float)

	def calculate_trace_state_variance(self, robot_data):
		[s, orinetations, sigma_s, index] = robot_data
		i = 2*index
		trace_state_var = np.trace(sigma_s[i:i+2, i:i+2])
		return np.trace(sigma_s)

	#def propagation_update(self, s, orinetations, sigma_s, input_data, sigma_odo, index):
	def propagation_update(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		sigma_odo = sensor_covariance

		num_robots = int(len(s)/2)
		delta_t = measurement_data[0]
		v = measurement_data[1]
		orinetations[index] = measurement_data[2]
		self_theta = orinetations[index]

		i = 2*index
		s[i,0] = s[i,0] + cos(self_theta)*v*delta_t   #x
		s[i+1,0] = s[i+1,0] + sin(self_theta)*v*delta_t #y

		var_u_v = sigma_odo[0,0]
		#sigma_s[i:i+2, i:i+2] = sigma_s[i:i+2, i:i+2]+ delta_t*delta_t*rot_mtx(self_theta)*matrix([[var_u_v, 0],[0, 0]])*rot_mtx(self_theta).T
		sigma_s[i:i+2, i:i+2] = sigma_s[i:i+2, i:i+2]+ delta_t*delta_t*rot_mtx(self_theta)*sigma_odo*rot_mtx(self_theta).T
		return [s, orinetations, sigma_s]


	#def absolute_obser_update(self, s, orinetations, sigma_s, input_data, sigma_ob, index):
	def absolute_obser_update(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		sigma_ob = sensor_covariance

		num_robots = int(len(s)/2)
		self_theta = orinetations[index]

		i = 2*index

		local_s = s[i:i+2]
		local_sigma = sigma_s[i:i+2,i:i+2]

		landmark_loc = measurement_data[0]
		meas_range = measurement_data[1]
		bearing = measurement_data[2]

		var_dis = sigma_ob[0,0]
		var_phi = sigma_ob[1,1]

		H = rot_mtx(self_theta).getT()*matrix([[-1,0],[0,-1]], dtype=float)

		local_s = s[i:i+2]
		local_sigma = sigma_s[i:i+2,i:i+2]

		landmark_loc = np.matrix(landmark_loc).getT()
		z = np.matrix([meas_range*cos(bearing), meas_range*sin(bearing)]).getT() # actual measurement
		delta_x = float(landmark_loc[0] - s.item(i,0))
		delta_y = float(landmark_loc[1] - s.item(i+1,0))
		z_hat = rot_mtx(self_theta).getT()*(np.matrix([delta_x, delta_y]).getT())  # shifted to robot frame


		sigma_z = rot_mtx(bearing) * sigma_ob * rot_mtx(bearing).getT()
		sigma_invention = H * local_sigma * H.getT()  + sigma_z
		kalman_gain = local_sigma*H.getT()*sigma_invention.getI()

		s[i:i+2] = local_s + kalman_gain*(z - z_hat)

		sigma_s[i:i+2,i:i+2] = local_sigma - kalman_gain*H*local_sigma

		return [s, orinetations, sigma_s]


	#def relative_obser_update(self, s, orinetations, sigma_s, input_data, sigma_ob, index):
	def relative_obser_update(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		sigma_ob = sensor_covariance
				#i obser j
		num_robots = int(len(s)/2)
		self_theta = orinetations[index]

		obser_index = measurement_data[0]
		meas_range = measurement_data[1]
		bearing = measurement_data[2]

		i = index * 2
		j = obser_index * 2

		H_ij  = np.zeros((2,2*num_robots))
		H_ij[0, i] = -1
		H_ij[1, i+1] = -1
		H_ij[0, j] = 1
		H_ij[1, j+1] = 1

		H = rot_mtx(self_theta).getT()*H_ij

		dis = meas_range
		phi = bearing

		z = matrix([[dis*cos(phi)],[dis*sin(phi)]])

		hat_j = z + s[i:i+2]

		sigma_i = sigma_s[i:i+2,i:i+2]
		sigma_j = sigma_s[j:j+2,j:j+2]

		e = 0.5

		sigma_j_next_inv = e*sigma_j.getI() + (1-e)*sigma_i.getI()

		s[j:j+2] = sigma_j_next_inv.getI()*(e*sigma_j.getI()*s[j:j+2] + (1-e)*sigma_i.getI()*hat_j)
		sigma_s[j:j+2,j:j+2] = sigma_j_next_inv.getI()
		return [s, orinetations, sigma_s]



