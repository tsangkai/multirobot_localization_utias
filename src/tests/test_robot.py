#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  8 16:47:50 2018

@author: william
"""
import sys
sys.path.insert(0, '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/functions/robots')
import robot_system
sys.path.insert(0, '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/functions/localization_algos')
import centralized_ekf 
sys.path.insert(0, '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/functions/dataset_manager')
from existing_dataset import Dataset

dataset_path = '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/datasets/MRCLAM_Dataset1/'
dataset_labels = [1,2,3,4,5]
testing_dataset = Dataset('testing', dataset_path, dataset_labels)
lm_map = testing_dataset.create_landmark_map()
print(lm_map)
st,staring_states,d, t_arr = testing_dataset.initialization_MRCLAMDatasets(20)

dataset_labels = [1,2,3,4,5]
loc_algo = centralized_ekf.centralized_EKF('cen_ekf')
r = robot_system.RobotSystem('r',dataset_labels,loc_algo, None, distr_sys = False)
r.set_states(staring_states)


