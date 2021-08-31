# coding: utf-8

#local state CI (LS-CI) - “Decentralized multi-robot cooperative localization using covariance intersection,” L. C. Carrillo-Arce, E. D. Nerurkar, J. L. Gordillo, and S. I. Roumeliotis
import numpy as np
from numpy import matrix
from numpy import linalg
from math import cos, sin, atan2, sqrt, pi
from localization_algo_framework import ekf_algo_framework
import scipy


def rot_mtx(theta):
	return matrix([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

class EKF_LS_CI2(ekf_algo_framework):
	def __init__(self, algo_name):
		ekf_algo_framework.__init__(self, algo_name)

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
		num_robots = int(len(s)/2)

		delta_t = measurement_data[0]
		v = measurement_data[1]
		orinetations[index] = measurement_data[2]
		self_theta = orinetations[index]

		i = 2*index
		local_s = s[i:i+2]	
		local_P = sigma_s[i:i+2,i:i+2]

		local_s[0,0] = local_s[0,0] + cos(self_theta)*v*delta_t #x
		local_s[1,0] = local_s[1,0] + sin(self_theta)*v*delta_t #y

		Q = sigma_odo
		W = delta_t*rot_mtx(self_theta)
		local_P = local_P + W*Q*W.getT() # A is identity matrix 

		s[i:i+2] = local_s
		sigma_s[i:i+2, i:i+2] = local_P

		return [s, orinetations, sigma_s]


	def absolute_obser_update(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data		
		sigma_ob = sensor_covariance
		
		num_robots = int(len(s)/2)
		self_theta = orinetations[index]

		landmark_loc = measurement_data[0] 
		meas_range = measurement_data[1]  
		bearing = measurement_data[2] 

		i = 2*index

		local_s = s[i:i+2]
		local_P = sigma_s[i:i+2,i:i+2]
		R = sigma_ob

		z = np.matrix([meas_range,bearing]).getT()
		lx = landmark_loc[0]
		ly = landmark_loc[1]
		x = local_s[0,0]
		y = local_s[1,0]

		z_hat_0 = sqrt((lx-x)*(lx-x)+(ly-y)*(ly-y))
		z_hat_1 = (atan2((ly-y), (lx-x))-self_theta)%pi
		if abs(z_hat_1-pi) < abs(z_hat_1):
			z_hat_1 = z_hat_1-pi
		z_hat = np.matrix([z_hat_0, z_hat_1]).getT()

		h11 = (-lx + x)/sqrt((lx - x)**2 + (ly - y)**2)
		h12 = (-ly + y)/sqrt((lx - x)**2 + (ly - y)**2)
		h21 = -(-ly + y)/((lx - x)**2 + (ly - y)**2)
		h22 = -(lx - x)/((lx - x)**2 + (ly - y)**2)
		H = np.matrix([[h11, h12],[h21, h22]])

		sigma_invention = H * local_P * H.getT() + R # V is a identity matrix
		K = local_P*H.getT()*sigma_invention.getI()
		local_s = local_s + K*(z - z_hat)
		local_P = local_P - K*H*local_P

		s[i:i+2] = local_s 
		sigma_s[i:i+2,i:i+2] = local_P
		
		return [s, orinetations, sigma_s]


	def relative_obser_update(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		sigma_ob = sensor_covariance
		
		#a obser b 
		num_robots = int(len(s)/2)
		self_theta = orinetations[index]

		obser_index = measurement_data[0] 
		meas_range = measurement_data[1]  
		bearing = measurement_data[2] 

		a_i = index * 2
		b_i = obser_index * 2 

		R = sigma_ob
		s_a = s[a_i:a_i+2]
		P_a = sigma_s[a_i:a_i+2,a_i:a_i+2]
		P_ab = sigma_s[a_i:a_i+2,b_i:b_i+2]

		s_b = s[b_i:b_i+2]
		P_b = sigma_s[b_i:b_i+2,b_i:b_i+2]

		x_a = s_a[0,0]
		y_a = s_a[1,0]
		x_b = s_b[0,0]
		y_b = s_b[1,0]

		z = np.matrix([meas_range,bearing]).getT()
		z_hat_0 = sqrt((x_b-x_a)*(x_b-x_a)+(y_b-y_a)*(y_b-y_a))
		z_hat_1 = (atan2((y_b-y_a), (x_b-x_a))-self_theta)%pi
		if abs(z_hat_1-pi) < abs(z_hat_1):
		    z_hat_1 = z_hat_1-pi
		z_hat = np.matrix([z_hat_0, z_hat_1]).getT()

		H_a = np.matrix(np.zeros((2,2)))
		H_a[0,0] = (x_a - x_b)/sqrt((-x_a + x_b)**2 + (-y_a + y_b)**2)
		H_a[0,1] = (y_a - y_b)/sqrt((-x_a + x_b)**2 + (-y_a + y_b)**2)
		H_a[1,0] = -(y_a - y_b)/((-x_a + x_b)**2 + (-y_a + y_b)**2)
		H_a[1,1] = -(-x_a + x_b)/((-x_a + x_b)**2 + (-y_a + y_b)**2)
		
		H_b = np.matrix(np.zeros((2,2)))
		H_b[0,0] = (-x_a + x_b)/sqrt((-x_a + x_b)**2 + (-y_a + y_b)**2)
		H_b[0,1] = (-y_a + y_b)/sqrt((-x_a + x_b)**2 + (-y_a + y_b)**2)
		H_b[1,0] = (y_a - y_b)/((-x_a + x_b)**2 + (-y_a + y_b)**2)
		H_b[1,1] = (-x_a + x_b)/((-x_a + x_b)**2 + (-y_a + y_b)**2)

		r = z - z_hat
		S = R + H_a * P_a * H_a.getT() + H_b * P_b * H_b.getT() - H_a * P_ab * H_b.getT() - H_b * P_ab * H_a.getT()
		S_sqrt = np.asmatrix(scipy.linalg.sqrtm(np.matrix(S)))

		D_arr = [0]*num_robots
		
		D_arr[index] = (P_ab * H_b.getT() - P_a * H_a.getT())*S_sqrt.getI()
		D_arr[obser_index] = (P_b * H_b.getT() - P_ab * H_a.getT())*S_sqrt.getI()
		
		for robot_index_j in range(num_robots):
			j = robot_index_j*2
			if robot_index_j != index and robot_index_j != obser_index:	
				D_arr[robot_index_j] = sigma_s[j:j+2,b_i:b_i+2]*H_b.getT()*S_sqrt.getI()-sigma_s[j:j+2,a_i:a_i+2]*H_a.getT()*S_sqrt.getI()
			s[j:j+2] = s[j:j+2]+D_arr[robot_index_j]*r
			sigma_s[j:j+2,j:j+2] = sigma_s[j:j+2,j:j+2]-D_arr[robot_index_j]*D_arr[robot_index_j].getT()

		# error....

		'''
		for robot_index_j in range(num_robots):
			j = robot_index_j*2
			for robot_index_l in range(num_robots):
				l = robot_index_l*2
				if l != j:
					sigma_s[j:j+2,l:l+2] = sigma_s[j:j+2,l:l+2]-D_arr[robot_index_j]*D_arr[robot_index_l].getT()
		'''
		return [s, orinetations, sigma_s]