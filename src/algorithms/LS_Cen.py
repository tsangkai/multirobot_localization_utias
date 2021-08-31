import numpy as np
import math
from math import cos, sin, atan2, sqrt
from numpy import dot
import scipy.linalg as linalg
#import parameters
from algorithms.EKF import ekf_algo_framework

def rot_mtx(theta):
    return np.matrix([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])


class LS_Cen(ekf_algo_framework):

    def __init__(self, algo_name):
        ekf_algo_framework.__init__(self, algo_name)

    def state_variance_init(self, num_robots):
        return 0.04*np.matrix(np.identity(2*num_robots), dtype = float)

    def calculate_trace_state_variance(self, robot_data):
        [s, orinetations, sigma_s, index] = robot_data
        i = 2*index
        trace_state_var = np.trace(sigma_s[i:i+2, i:i+2])
        return np.trace(sigma_s)

    #ef propagation_update(self, s, orinetations, sigma_s, input_data, sigma_odo, index):
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

        G = rot_mtx(self_theta)
        W = sigma_odo
        W[0,0] = sigma_odo[0,0]*delta_t*delta_t
        #W[1,1] = sigma_odo[1,1]*delta_t*delta_t*v*v
        W[1,1] = sigma_odo[1,1]*delta_t*delta_t
        Q_ii = G * W * G.getT()
        Q = np.zeros((2*num_robots,2*num_robots))
        Q[i:i+2, i:i+2] = Q_ii
        sigma_s  = sigma_s + Q  #errors in the odometric measurements of the robots are uncorrelated, it's a diagonal matrix

        return [s, orinetations, sigma_s]



    #def absolute_obser_update(self, s, orinetations, sigma_s, input_data, sigma_ob, index):
    def absolute_obser_update(self, robot_data, sensor_data):
        [s, orinetations, sigma_s, index] = robot_data
        [measurement_data, sensor_covariance] = sensor_data
        sigma_ob = sensor_covariance
        num_robots = int(len(s)/2)
        self_theta = orinetations[index]
        i = 2*index
        landmark_loc = measurement_data[0]
        meas_range = measurement_data[1]
        bearing = measurement_data[2]

        ###################
        z = np.matrix([meas_range*cos(bearing), meas_range*sin(bearing)]).getT() # actual measurement
        # for estimated measurement
        delta_x = landmark_loc[0] - s.item(i,0)
        delta_y = landmark_loc[1] - s.item(i+1,0)
        z_hat = rot_mtx(self_theta).getT()*(np.matrix([delta_x, delta_y]).getT())  # shifted to robot frame

        H_i = np.matrix(np.zeros((2,2*num_robots)))
        H_i[0, i] = -1
        H_i[1, i+1] = -1
        H = rot_mtx(self_theta).getT()*H_i

        #sigma_ob[1,1] = sigma_ob[1,1]*meas_range*meas_range
        V = rot_mtx(bearing)
        R =  V*sigma_ob*V.getT() #sigma_z

        sigma_invention = H * sigma_s * H.getT() + R
        kalman_gain = sigma_s*H.getT()*sigma_invention.getI()
        s = s + kalman_gain*(z - z_hat)
        sigma_s = sigma_s - kalman_gain*H*sigma_s
        ##################

        return [s, orinetations, sigma_s]



    #def relative_obser_update(self, s, orinetations, sigma_s, input_data, sigma_ob, index):
    def relative_obser_update(self, robot_data, sensor_data):
        #when robot i observes robot j

        [s, orinetations, sigma_s, index] = robot_data
        [measurement_data, sensor_covariance] = sensor_data
        sigma_ob = sensor_covariance

        num_robots = int(len(s)/2)
        self_theta = orinetations[index]


        ###############
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

        #sigma_ob[1,1] = sigma_ob[1,1]*meas_range*meas_range

        V = rot_mtx(bearing)
        R = V*sigma_ob*V.getT()

        sigma_invention = H* sigma_s*H.getT() + R
        kalman_gain = sigma_s*H.getT()*sigma_invention.getI()
        s = s + kalman_gain*(z - z_hat)
        sigma_s = sigma_s - kalman_gain*H*sigma_s
        #################

        return [s, orinetations, sigma_s]
