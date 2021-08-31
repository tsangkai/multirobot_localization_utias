#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 10 17:43:58 2018

@author: william
"""

import os, sys
import getpass
sys.path.append(os.path.join(os.path.dirname(__file__), "."))
from dataset_manager.existing_dataset import Dataset
from simulation_process.sim_manager import  SimulationManager
from robots.robot_system import RobotSystem
from simulation_process.state_recorder import StatesRecorder
from data_analysis.data_analyzer import Analyzer
from data_analysis.realtime_plot import animate_plot
from pprint import pprint

# load algorithms 
sys.path.append(os.path.join(os.path.dirname(__file__), "localization_algos"))
from centralized_ekf import Centralized_EKF # works


#dataset_path = '/home/william/catkin_ws/ros_colo_dataset/'
#dataset_path = '/home/william/UTIAS-dataset/MRCLAM_Dataset3/'
compname = getpass.getuser()
dataset_path = "/home/"+ compname +"/CoLo/CoLo-D/CoLo-Datasets/official_dataset2/"

dataset_labels = [1,2,3]
duration = 120 # duration for the simulation in sec
testing_dataset = Dataset('testing')
start_time, starting_states, dataset_data, time_arr = testing_dataset.load_MRCLAMDatasets(dataset_path, dataset_labels, duration, delay_start = 0)
loc_algo = Centralized_EKF('algo')
robot = RobotSystem('robot', dataset_labels, loc_algo, distr_sys = False)

sim = SimulationManager('Centralized_EKF')
state_recorder = StatesRecorder('recorder',dataset_labels)
sim.sim_process_native(dataset_labels, testing_dataset, robot, state_recorder, simple_plot = False)


analyzer = Analyzer('analyzer', dataset_labels)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder, plot_graphs = False)
robot_loc_time_unit = analyzer.robot_location_at_unit_time_interval(state_recorder)

print("Animation Start: ")
animate_plot(dataset_labels, state_recorder, analyzer, lm = testing_dataset.get_landmark_map())
