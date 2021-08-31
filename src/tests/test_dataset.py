#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Wed Feb 14 16:13:43 2018

@author: william
"""
from existing_dataset import Dataset

import sys
sys.path.insert(0, '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/functions/requests')
import request_response

"""Testing branch for dataset related classes and functions"""

# testing for DataSet class

dataset_path = '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/datasets/MRCLAM_Dataset1/'
dataset_labels = [1,2,3,4,5]
testing_dataset = Dataset('testing', dataset_path, dataset_labels)
lm_map = testing_dataset.create_landmark_map()
print(lm_map)
st,_,d,t_arr = testing_dataset.initialization_MRCLAMDatasets(20)

'''
for i, label in enumerate(dataset_labels):
    print(d['odometry'][i][-1]['time'])
    print(d['measurement'][i][-1]['time'])
    print(d['groundtruth'][i][-1]['time'])
'''
'''
prop_req= request_response.Prop_req_resp(None, None)
print(prop_req.get_message())
print(prop_req.get_type())
'''


g_req= request_response.Request_response(None, None)
print(g_req.get_message())
print(g_req.get_type())
'''
c_t = st
prev_time = 0
while True:
    print('***************** ****************')
    g_req= request_response.Request_response(None, None)
    prev_time = c_t
    valid, c_t, rsp = testing_dataset.respond(g_req, c_t)
    print('current time: ', c_t)

    g_req= request_response.Meas_req_resp(None, None)
    prev_time = c_t
    valid, c_t, rsp = testing_dataset.respond(g_req, c_t)
    print('current time-M: ', c_t)

    if c_t < prev_time:
        print('Time inconsistant!')
        break

    if valid ==  False :
        print('Done!')
        break

'''
