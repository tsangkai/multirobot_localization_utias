# -*- coding: utf-8 -*-

import numpy as np
from numpy import random
import sys
from math import atan2, sqrt, pi
import random
from math import cos
from math import sin
import math
import os.path
'''
sys.path.insert(0, '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/functions/requests')
import request_response
'''

def find_nearest_time_idx(l, value):
    if len(l) != 0:
        array = np.asarray(l)
        idx = (np.abs(array-value)).argmin()
        return idx
    else:
        return None


def find_next_time_idx(array, start_idx, value):
    i = start_idx
    try:
        array[i]
    except IndexError:
        i = -1
        return i
    while array[i] < value:
        i = i+1
        try:
            array[i]
        except IndexError:
            i = -1
            break
    return i

def linear_interpolation(end0, end1, x):
    [x0, y0] = end0
    [x1, y1] = end1
    if x1 == x0:
        y = y0
    else:
        y = y0+(x-x0)*(y1-y0)/(x1-x0)
    return y


class Sim_Dataset_Manager:

    def __init__(self, dataset_name):
        self.name = dataset_name

    def circular_path_4_robots(self, duration, op_freqs = [[2, 1, 1], [2, 1, 1], [2, 1, 1], [2, 1, 1]], lm_dict = {6:[0,0]}):

        self.robot_lables = [1,2,3,4]
        self.duration = duration
        self.landmark_map = lm_dict
        self.op_freqs = np.array(op_freqs) #['prop', 'lm_obser', 'rel_obser', 'comm'] for each robot
        self.num_op = 3
        self.op_time_interval = 1/  self.op_freqs

        self.start_time = 0
        self.starting_states = {}
        self.starting_states[1] = [self.start_time, 1, 0, pi/2]
        self.starting_states[2] = [self.start_time, 0, 1, pi]
        self.starting_states[3] = [self.start_time, -1, 0, 3*pi/2]
        self.starting_states[4] = [self.start_time, 0, -1, 2*pi]

        self.num_robots = len(self.robot_lables)
        self.measurement_data = [[] for i in range(self.num_robots)]
        self.odometry_data = [[] for i in range(self.num_robots)]
        self.groundtruth_data = [[] for i in range(self.num_robots)]
        #noise
        self.noise_v = 0.01
        self.noise_range = 0.1
        self.noise_bearing = 0.035


        print ('******** Initialization Completed ********')
        print("self.start_time: ", self.start_time)
        print("self.starting_states ", self.starting_states)

        self.current_time = self.start_time

        self.current_states = {}
        for label in self.robot_lables:
            self.current_states[label] = {'x': self.starting_states[label][1], 'y': self.starting_states[label][2], 'orientation': self.starting_states[label][3]}

        self.time_arr = np.zeros((self.num_robots, self.num_op))
        self.dataset_data = {'odometry': self.odometry_data, 'measurement': self.measurement_data, 'groundtruth': self.groundtruth_data}
        return self.start_time, self.starting_states, self.dataset_data, self.time_arr


    def observation_target(self, rbt_lable, op_idx):
        target = None
        if rbt_lable == 1:
            if op_idx == 1:
                target = 6

        elif rbt_lable == 2:
            if op_idx == 2:
                target = 1

        elif rbt_lable == 3:
            if op_idx == 1:
                target = 6
            elif op_idx == 2:
                target = 4

        return target

    def get_start_time(self):
        return self.start_time

    def get_starting_states(self):
        return self.starting_states

    def get_duration(self):
        return self.duration

    def get_landmark_map(self):
        return self.landmark_map

    def get_start_moving_times(self):
        return np.full(self.num_robots, self.start_time)


    def respond(self, req, current_time, need_specific_time = False):
        np.random.seed(0)
        message = req.get_message()
        valid_respond = False
        if need_specific_time:
            valid_respond = False
        else:

            rbt_idx, op_idx = np.unravel_index(self.time_arr.argmin(), self.time_arr.shape)
            req_time = self.time_arr[rbt_idx, op_idx]
            message['robot_index'] = rbt_idx
            message['time'] = req_time

            robot_actual_state = self.current_states[self.robot_lables[rbt_idx]]
            gt_x = robot_actual_state['x']
            gt_y = robot_actual_state['y']
            gt_orientation =  robot_actual_state['orientation']


            if op_idx == 0: #propagation
                req_type = 'odometry'
                actual_v = np.random.uniform(0, 0.5) # between 0 to 0.5 m/s
                input_v = actual_v + np.random.normal(0, self.noise_v)
                actual_av = 0.02
                delta_t = self.op_time_interval[rbt_idx, op_idx]
                gt_x = gt_x + actual_v*delta_t*cos(gt_orientation)
                gt_y = gt_y + actual_v*delta_t*sin(gt_orientation)
                gt_orientation =  gt_orientation + actual_av*delta_t

                message['data'] = {'time':req_time, 'velocity': input_v, 'angular velocity': actual_av, 'orientation': gt_orientation, 'delta_t': delta_t}
                valid_respond = True

            elif op_idx == 1: # landmark observation
                req_type = 'measurement'
                observed_label = self.observation_target(self.robot_lables[rbt_idx], op_idx)
                if observed_label != None:
                    [lm_x, lm_y]= self.landmark_map[6]
                    delta_x = lm_x - gt_x
                    delta_y = lm_y - gt_y
                    meas_range = sqrt(delta_y**2+delta_x**2) + np.random.normal(0, self.noise_range)
                    bearing =  math.atan2(delta_y, delta_x) - gt_orientation + np.random.normal(0, self.noise_bearing)
                    message['data'] = {'time':req_time, 'subject_ID':6, 'measurment_range': meas_range, 'bearing':bearing}
                    valid_respond = True


            elif op_idx == 2: # relative observation
                req_type = 'measurement'
                observed_label = self.observation_target(self.robot_lables[rbt_idx], op_idx)
                if observed_label != None:
                    #observed_label = self.robot_lables[(rbt_idx+1)%self.num_robots]
                    obser_x = self.current_states[observed_label]['x']
                    obser_y = self.current_states[observed_label]['y']
                    delta_x = obser_x - gt_x
                    delta_y = obser_y - gt_y
                    meas_range = sqrt(delta_y**2+delta_x**2) + np.random.normal(0, self.noise_range)
                    bearing =  math.atan2(delta_y, delta_x) - gt_orientation + np.random.normal(0, self.noise_bearing)
                    message['data'] = {'time':req_time, 'subject_ID': observed_label, 'measurment_range': meas_range, 'bearing':bearing}
                    valid_respond = True
                    #print(self.robot_lables[rbt_idx], observed_label)
            message['groundtruth'] =  {'time':req_time, 'x_pos':gt_x, 'y_pos':gt_y, 'orientation':gt_orientation}
            self.current_states[self.robot_lables[rbt_idx]] = {'x': gt_x, 'y': gt_y, 'orientation': gt_orientation}

            req.set_message(message)
            req.set_type(req_type)


            #time update
            self.current_time = self.time_arr.min()
            self.time_arr[rbt_idx, op_idx] += self.op_time_interval[rbt_idx, op_idx]


        return valid_respond, self.current_time, req


    def get_robot_groundtruth(self, gt_time, robot_index):
        robot_actual_state = self.current_states[self.robot_lables[robot_index]]
        gt_x = robot_actual_state['x']
        gt_y = robot_actual_state['y']
        gt_orientation =  robot_actual_state['orientation']

        return  {'time':self.current_time, 'x_pos':gt_x, 'y_pos':gt_y, 'orientation':gt_orientation}


    def dataset_reset(self):

        self.num_robots = len(self.robot_lables)
        self.measurement_data = [[] for i in range(self.num_robots)]
        self.odometry_data = [[] for i in range(self.num_robots)]
        self.groundtruth_data = [[] for i in range(self.num_robots)]

        print ('******** Initialization Completed ********')
        print("self.start_time: ", self.start_time)
        print("self.starting_states ", self.starting_states)

        self.current_time = self.start_time

        self.current_states = {}
        for label in self.robot_lables:
            self.current_states[label] = {'x': self.starting_states[label][1], 'y': self.starting_states[label][2], 'orientation': self.starting_states[label][3]}

        self.time_arr = np.zeros((self.num_robots, self.num_op))
        return self.start_time, self.starting_states, self.dataset_data, self.time_arr
