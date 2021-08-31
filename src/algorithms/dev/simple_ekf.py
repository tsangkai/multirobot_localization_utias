import numpy as np
import math
from math import cos, sin, atan2, sqrt, pi
from numpy import dot, arctan
from numpy.linalg import inv
import scipy.linalg as linalg
import sys
#import parameters
from localization_algo_framework import ekf_algo_framework
#from sympy import *

import IPython


def rot_mtx(theta):
    return np.matrix([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
 
# TODO - Multiple robots
class Simple_EKF(ekf_algo_framework):

    # TODO - Change documentation: we don't need init function for orientation or for state anymore
    # initialization of state & orientation is done in robot_centralized.py
    # we only need init function for state_variance_init
    def __init__(self, algo_name):
        # TODO - We define our own sensor-covariance based upon user input for noise (IGNORE Passed covariance - this is based on real-world) this was the source of unkown noise
        # Make this dependent on user input, rather than hard-coded
        self.sigma_odo = 0.01
        self.sigma_ob = 0.01*np.matrix(np.identity(2,float))
        ekf_algo_framework.__init__(self, algo_name)
        
    def state_variance_init(self, num_robots):
        return 0.01*np.matrix(np.identity(2*num_robots), dtype = float)

    def calculate_trace_state_variance(self, robot_data):
        sigma_s = robot_data[2]
        return np.trace(sigma_s)

    def propagation_update(self, robot_data, sensor_data):
        
        # state : [x,y], orientations : [o1,o2, ...], state_variance, robot_idx (useful if multiple robots)
        [s, orientations, sigma_s, index] = robot_data
        
        # measurement data : [delta_t, v, orientation]
        odometry_data = sensor_data[0]

        delta_t = odometry_data[0]
        v = odometry_data[1]
        self_theta = odometry_data[2]
        orientations[index] = self_theta

        if delta_t < 0:
            sys.exit('delta_t: '+ str(delta_t))
        
        # State update
        s[0,0] += cos(self_theta)*v*delta_t #x
        s[1,0] += sin(self_theta)*v*delta_t #y
    
        Q = self.sigma_odo
        P = sigma_s

        W = delta_t*np.matrix([cos(self_theta), sin(self_theta)]).getT()
        P = P  + W*Q*W.getT() # A is identity matrix 
        
        # state variance update
        sigma_s = P

        return [s, orientations, sigma_s]

    def absolute_obser_update(self, robot_data, sensor_data):

        # state : [x,y], orientations : [o1,o2, ...], state_variance, robot_idx (useful if multiple robots
        [s, orientations, sigma_s, index] = robot_data
        
        measurement_data = sensor_data[0]
        
        landmark_loc = measurement_data[0] 
        meas_range = measurement_data[1]  
        bearing = measurement_data[2] 

        P = sigma_s
        R = self.sigma_ob
        z = np.matrix([meas_range,bearing]).getT()
        lx = landmark_loc[0]
        ly = landmark_loc[1]
        x = s[0,0]
        y = s[1,0]

        self_theta = orientations[index][0,0]

        z_hat_0 = sqrt((lx-x)**2+(ly-y)**2) # measurement distance
        z_hat_1 = atan2((ly-y), (lx-x)) - self_theta

        # -pi to pi
        while (z_hat_1 < -pi):
            z_hat_1 += 2*pi
        while (z_hat_1 > pi):
            z_hat_1 -= 2*pi

        z_hat = np.matrix([z_hat_0, z_hat_1]).getT()
        
        h11 = -(lx - x)/sqrt((lx - x)**2 + (ly - y)**2)
        h12 = -(ly - y)/sqrt((lx - x)**2 + (ly - y)**2)
        h21 = (ly - y)/((lx - x)**2 + (ly - y)**2)
        h22 = (x - lx)/((lx - x)**2 + (ly - y)**2)
        
        H = np.matrix([[h11, h12],[h21, h22]])

        # Compute Kalman Gain
        K = P*H.getT() * (H*P*H.getT() + R).getI() # V is the identity matrix
        # State correction
        s_update = K*(z - z_hat)
        s = s + s_update
        
        # state variance correction
        P = (np.matrix(np.identity(2,float)) - K*H)*P 
        sigma_s = P

        return [s, orientations, sigma_s]