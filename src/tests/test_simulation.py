#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  8 17:55:06 2018

@author: william
"""

import os, sys
import getpass
sys.path.append(os.path.join(os.path.dirname(__file__), "."))
from dataset_manager.realworld_dataset_manager import RW_Dataset_Manager
from simulation_process.sim_manager import  SimulationManager
from robots.robot_system import RobotSystem
from simulation_process.state_recorder import StatesRecorder
from data_analysis.data_analyzer import Analyzer
from data_analysis.realtime_plot import animate_plot

from pprint import pprint

# load algorithms 
sys.path.append(os.path.join(os.path.dirname(__file__), "localization_algos"))
from centralized_ekf import Centralized_EKF # works
from simple_ekf import Simple_EKF
from ekf_ls_bda import EKF_LS_BDA
from ekf_ls_ci import EKF_LS_CI
from ekf_gs_bound import EKF_GS_BOUND
from ekf_gs_ci2 import EKF_GS_CI2

compname = getpass.getuser()
#dataset_path = "/home/"+ compname +"/CoLo/CoLo-D/UTIAS-Datasets/MRCLAM_Dataset3/"


dataset_path = "/Users/shengkangchen/Documents/CoLo/CoLo-D/CoLo-Datasets/official_dataset1/"
robot_labels = [1,2,3]
duration = 120 # duration for the simulation in sec
testing_dataset = RW_Dataset_Manager('testing')
start_time, starting_states, dataset_data, time_arr = testing_dataset.load_datasets(dataset_path, robot_labels, duration)


analyzer = Analyzer('analyzer', robot_labels)

loc_algo = EKF_LS_BDA('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('sim LS-BDA')
state_recorder_bda = StatesRecorder('LS-BDA', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_bda, simple_plot = True)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_bda, plot_graphs = True)

##############################################################################

testing_dataset.dataset_reset()
loc_algo = Centralized_EKF('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('Centralized_EKF')
state_recorder_cen = StatesRecorder('Cen-EKF', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_cen, simple_plot = True)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_cen, plot_graphs = True)

analyzer.algos_comparison([state_recorde_gs_ci, state_recorder_cen])

