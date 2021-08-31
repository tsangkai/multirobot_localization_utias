import numpy as np
import math
from math import cos, sin, atan2, sqrt, pi
from numpy import dot
import scipy.linalg as linalg
#import parameters
from localization_algo_framework import ekf_algo_framework

def rot_mtx(theta):
    return np.matrix([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
 

class Centralized_EKF2(ekf_algo_framework):

    def __init__(self, algo_name):
        ekf_algo_framework.__init__(self, algo_name)

    def state_variance_init(self, num_robots):
        return 0.01*np.matrix(np.identity(2*num_robots), dtype = float)

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
        s[i,0] = s[i,0] + cos(self_theta)*v*delta_t   #x
        s[i+1,0] = s[i+1,0] + sin(self_theta)*v*delta_t #y
        
        Q = sigma_odo
        Q[1,1] = Q[1,1]*v*v
        P = sigma_s[i:i+2, i:i+2]
        W = delta_t*rot_mtx(self_theta)
        W[0,1] = sigma_odo[0,1]*v
        W[1,1] = sigma_odo[1,1]*v

        P = P + W*Q*W.getT() # A is identity matrix 

        sigma_s[i:i+2, i:i+2] = P

        return [s, orinetations, sigma_s]



    #def absolute_obser_update(self, s, orinetations, sigma_s, input_data, sigma_ob, index):
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
        P = sigma_s
        sigma_ob[1,1] = sigma_ob[1,1]*meas_range*meas_range
        R = sigma_ob
        z = np.matrix([meas_range,bearing]).getT()
        lx = landmark_loc[0]
        ly = landmark_loc[1]
        x = s[i,0]
        y = s[i+1,0]

        z_hat_0 = sqrt((lx-x)*(lx-x)+(ly-y)*(ly-y))
        z_hat_1 = (atan2((ly-y), (lx-x))-self_theta)%pi
        if abs(z_hat_1-pi) < abs(z_hat_1):
            z_hat_1 = z_hat_1-pi

        z_hat = np.matrix([z_hat_0, z_hat_1]).getT()


        H = np.matrix(np.zeros((2,2*num_robots)))
        h11 = (-lx + x)/sqrt((lx - x)**2 + (ly - y)**2)
        h12 = (-ly + y)/sqrt((lx - x)**2 + (ly - y)**2)
        h21 = -(-ly + y)/((lx - x)**2 + (ly - y)**2)
        h22 = -(lx - x)/((lx - x)**2 + (ly - y)**2)
        H[0,i] = h11
        H[0,i+1] = h12
        H[1,i] = h21
        H[1,i+1] = h22
        

        sigma_invention = H * P * H.getT() + R
        kalman_gain = P*H.getT()*sigma_invention.getI()
        s = s + kalman_gain*(z - z_hat)
        P = P - kalman_gain*H*P
        sigma_s = P
        
        return [s, orinetations, sigma_s]



    def relative_obser_update(self, robot_data, sensor_data):
        #when robot i observes robot j 

        [s, orinetations, sigma_s, index] = robot_data
        [measurement_data, sensor_covariance] = sensor_data
        sigma_ob = sensor_covariance   
        num_robots = int(len(s)/2)
        self_theta = orinetations[index]
        
        obser_index = measurement_data[0] 
        meas_range = measurement_data[1]  
        bearing = measurement_data[2] 

        i = 2*index
        P = sigma_s
        sigma_ob[1,1] = sigma_ob[1,1]*meas_range*meas_range
        R = sigma_ob
        z = np.matrix([meas_range,bearing]).getT()
        j = obser_index * 2 

        x_i = s[i,0]
        y_i = s[i+1,0]
        x_j = s[j,0]
        y_j = s[j+1,0]

        z_hat_0 = sqrt((x_j-x_i)*(x_j-x_i)+(y_j-y_i)*(y_j-y_i))
        z_hat_1 = (atan2((y_j-y_i), (x_j-x_i))-self_theta)%pi
        if abs(z_hat_1-pi) < abs(z_hat_1):
            z_hat_1 = z_hat_1-pi

        z_hat = np.matrix([z_hat_0, z_hat_1]).getT()

        H = np.matrix(np.zeros((2,2*num_robots)))
        H[0,i] = (x_i - x_j)/sqrt((-x_i + x_j)**2 + (-y_i + y_j)**2)
        H[0,i+1] = (y_i - y_j)/sqrt((-x_i + x_j)**2 + (-y_i + y_j)**2)
        H[1,i] = -(y_i - y_j)/((-x_i + x_j)**2 + (-y_i + y_j)**2)
        H[1,i+1] = -(-x_i + x_j)/((-x_i + x_j)**2 + (-y_i + y_j)**2)

        H[0,j] = (-x_i + x_j)/sqrt((-x_i + x_j)**2 + (-y_i + y_j)**2)
        H[0,j+1] = (-y_i + y_j)/sqrt((-x_i + x_j)**2 + (-y_i + y_j)**2)
        H[1,j] = (y_i - y_j)/((-x_i + x_j)**2 + (-y_i + y_j)**2)
        H[1,j+1] = (-x_i + x_j)/((-x_i + x_j)**2 + (-y_i + y_j)**2)

        sigma_invention = H * P * H.getT() + R
        kalman_gain = P*H.getT()*sigma_invention.getI()
        s = s + kalman_gain*(z - z_hat)
        P = P - kalman_gain*H*P
        sigma_s = P
        
        
        return [s, orinetations, sigma_s]
