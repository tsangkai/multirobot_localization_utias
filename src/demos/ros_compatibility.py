#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 15 15:45:54 2018

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

# load algorithms 
sys.path.append(os.path.join(os.path.dirname(__file__), "localization_algos"))
from centralized_ekf import Centralized_EKF

compname = getpass.getuser()
dataset_path = "/home/"+ compname +"/CoLo/CoLo-D/CoLo-Datasets/official_dataset3/"


dataset_labels = [1,2,3]
duration = 310 # duration for the simulation in sec
testing_dataset = Dataset('testing')
start_time, starting_states, dataset_data, time_arr = testing_dataset.load_MRCLAMDatasets(dataset_path, dataset_labels, duration, delay_start = 0)
loc_algo = Centralized_EKF('algo')
robot = RobotSystem('robot', dataset_labels, loc_algo, distr_sys = False)

sim = SimulationManager('CoLo: Centralized_EKF')
state_recorder = StatesRecorder('recorder',dataset_labels)
sim.sim_process_native(dataset_labels, testing_dataset, robot, state_recorder, simple_plot = True)


analyzer = Analyzer('analyzer', dataset_labels)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder, plot_graphs = True)
robot_loc_time_unit = analyzer.robot_location_at_unit_time_interval(state_recorder)
data_in_time_order = state_recorder.get_data_in_time_order()
update_in_time_order = state_recorder.get_updata_type_in_time_order()
update_type_arr = state_recorder.get_update_type_arr()
recorded_data = state_recorder.get_recorded_data()
#Format: recorded_dataline = [time, robot_label, est_x_pos, est_y_pos, trace_state_var, gt_x_pos, gt_y_pos, loc_err] 
