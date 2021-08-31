#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 16 16:26:29 2019

@author: william
"""

import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), "."))
from dataset_manager.realworld_dataset_manager import RW_Dataset_Manager


# load algorithms 
sys.path.append(os.path.join(os.path.dirname(__file__), "localization_algos"))

# dataset_path = "/Users/shengkangchen/Documents/CoLo/CoLo-D/CoLo-Datasets/official_dataset3/"
dataset_path = "../CoLo-D/CoLo-Datasets/official_dataset1/" # for desktop Ubuntu

robot_labels = [1,2,3]
duration = 180 # duration for the simulation in sec
testing_dataset = RW_Dataset_Manager('testing')
start_time, starting_states, dataset_data, time_arr = testing_dataset.load_datasets(dataset_path, robot_labels, duration)
testing_dataset.dataset_analysis(duration)