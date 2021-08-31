#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr  5 15:24:42 2018

@author: william
"""

import sys
import numpy as np
sys.path.insert(0, '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/functions/requests')
import request_response
sys.path.insert(0, '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/functions/dataset_manager')
from existing_dataset import Dataset
from state_recorder import StatesRecorder

dataset_path = '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/datasets/MRCLAM_Dataset1/'
dataset_labels = [1,2,3,4,5]
testing_dataset = Dataset('testing', dataset_path, dataset_labels)
lm_map = testing_dataset.create_landmark_map()
print(lm_map)
st,staring_states,d, t_arr = testing_dataset.initialization_MRCLAMDatasets(20)
sr = StatesRecorder('sr',dataset_labels)
sr.set_stating_states(staring_states)


print('*********************************')
g_req = request_response.Request_response(None, None)
c_t,rsp = testing_dataset.respond(g_req, st)
print(g_req.get_message())
print(g_req.get_type())

robot_state = {'x_pos': 1, 'y_pos':1, 'trace of state variance': 0.1 }
sr.record_state(rsp, robot_state)


print('*********************************')
g_req = request_response.Request_response(None, None)
c_t,rsp = testing_dataset.respond(g_req, st)
print(g_req.get_message())
print(g_req.get_type())

robot_state = {'x_pos': 2, 'y_pos':1, 'trace of state variance': 0.2 }
sr.record_state(rsp, robot_state)

print('*********************************')
g_req = request_response.Request_response(None, None)
c_t,rsp = testing_dataset.respond(g_req, st)
print(g_req.get_message())
print(g_req.get_type())

robot_state = {'x_pos': 3, 'y_pos':1, 'trace of state variance': 0.3 }
sr.record_state(rsp, robot_state)

#
print('#############')
print(dataset_labels.index(4))
print(sr.get_loc_err_arr(1))
print(sr.get_trace_sigma_s_arr(1))
print(sr.get_time_arr(1))

'''
r_d=sr.get_recorded_data()
r_d_a = np.array(r_d[1])
print(r_d_a[:,0])

'''
