import numpy as np
from numpy import matrix
from math import cos, sin, atan2, sqrt
from localization_algo_framework import ekf_algo_framework

def rot_mtx(theta):
	return matrix([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])


class EKF_GS_CI(ekf_algo_framework):
	def __init__(self, algo_name):
		ekf_algo_framework.__init__(self, algo_name)

	def state_variance_init(self, num_robots):
		return 0.02*np.matrix(np.identity(2*num_robots), dtype = float)

	def calculate_trace_state_variance(self, robot_data):
		[s, orinetations, sigma_s, index] = robot_data
		i = 2*index
		trace_state_var = np.trace(sigma_s[i:i+2, i:i+2])
		return trace_state_var
		
	def propagation_update(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		sigma_odo = sensor_covariance

		var_v = 0.1

		num_robots = int(len(s)/2)
		delta_t = measurement_data[0]
		v = measurement_data[1]
		orinetations[index] = measurement_data[2]
		self_theta = orinetations[index]

		i = 2*index

		s[i,0] = s[i,0] + cos(self_theta)*v*delta_t   #x
		s[i+1,0] = s[i+1,0] + sin(self_theta)*v*delta_t #y

		for j in range(num_robots):
			jj = 2*j
			if j==index:
				#print(np.matmul(sigma_odo,rot_mtx(self_theta).getT()))
				sigma_s[jj:jj+2, jj:jj+2] += delta_t*delta_t*rot_mtx(self_theta)*sigma_odo*rot_mtx(self_theta).getT()
			else:
				sigma_s[jj:jj+2, jj:jj+2] += delta_t*delta_t*var_v*np.identity(2)

		return [s, orinetations, sigma_s]


	def absolute_obser_update(self, robot_data, sensor_data):
		[s, orinetations, sigma_s, index] = robot_data
		[measurement_data, sensor_covariance] = sensor_data
		

		sigma_ob = sensor_covariance
		num_robots = int(len(s)/2)
		self_theta = orinetations[index]

		landmark_loc = measurement_data[0] 
		dis = measurement_data[1]  
		phi = measurement_data[2]

		i = 2*index
		H_i  = np.matrix(np.zeros((2,2*num_robots)), dtype = float)
		H_i[0, i] = -1
		H_i[1, i+1] = -1
		H = rot_mtx(self_theta).getT()*H_i

		
		#z_hat= rot_mtx(self_theta).getT() * (landmark_loc + H_i*s)
		delta_x = landmark_loc[0] - s.item(i,0)
		delta_y = landmark_loc[1] - s.item(i+1,0)
		z_hat = rot_mtx(self_theta).getT()*(np.matrix([delta_x, delta_y]).getT())
		z = matrix([dis*cos(phi), dis*sin(phi)]).getT()

		sigma_z = rot_mtx(phi) * sigma_ob * rot_mtx(phi).getT() 

		sigma_invention = H * sigma_s * H.getT()  + sigma_z
		kalman_gain = sigma_s*H.getT()*sigma_invention.getI()
		
		s = s + kalman_gain*(z - z_hat)
		sigma_s = sigma_s - kalman_gain*H*sigma_s

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
		meas_range = measurement_data[1]  
		bearing = measurement_data[2] 
		j = obser_index * 2 

		z = np.matrix([meas_range*cos(bearing), meas_range*sin(bearing)]).getT() # actual measurement

		H_ij  = np.zeros((2,2*num_robots))
		H_ij[0, i] = -1
		H_ij[1, i+1] = -1
		H_ij[0, j] = 1
		H_ij[1, j+1] = 1
		H = rot_mtx(self_theta).getT()*H_ij 

		z_hat = H * s

		sigma_ob[1,1] = sigma_ob[1,1]*meas_range*meas_range

		V = rot_mtx(bearing)
		R = V*sigma_ob*V.getT() #sigma_z

		sigma_invention = H*sigma_s*H.getT() + R
		kalman_gain = sigma_s*H.getT()*sigma_invention.getI()

		s = s + kalman_gain*(z - z_hat)
		print('inside ekf gs ci')
		print(s)
		sigma_s = sigma_s - kalman_gain*H*sigma_s

		return [s, orinetations, sigma_s]
	
	def communication(self, robot_data, sensor_data):
		
		[s, orinetations, sigma_s, index] = robot_data
		[comm_data, comm_e] = sensor_data

		comm_robot_s = comm_data[0]
		comm_robot_sigma_s = comm_data[1]

		e =  0.23 # (iii+1)*0.01
		#e = comm_e

		sig_inv = e*sigma_s.getI() + (1-e)*comm_robot_sigma_s.getI()
		s = sig_inv.getI() * (e*sigma_s.getI()*s + (1-e)*comm_robot_sigma_s.getI()*comm_robot_s)
		sigma_s = sig_inv.getI()

		return [s, orinetations, sigma_s]
