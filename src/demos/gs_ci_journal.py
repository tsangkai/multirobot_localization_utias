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
from dataset_manager.simulated_dataset_manager import Sim_Dataset_Manager
from simulation_process.sim_manager import  SimulationManager
from robots.robot_system import RobotSystem
from simulation_process.state_recorder import StatesRecorder
from data_analysis.data_analyzer import Analyzer
from data_analysis.realtime_plot import animate_plot

# load algorithms 
sys.path.append(os.path.join(os.path.dirname(__file__), "localization_algos"))
from ekf_gs_ci2 import EKF_GS_CI2
from gs_ci_bound import GS_CI_Bound





##############################################################################



# dataset_path = "/Users/shengkangchen/Documents/CoLo/CoLo-D/CoLo-Datasets/official_dataset3/"
dataset_path = "/home/william/CoLo/CoLo-D/CoLo-Datasets/official_dataset1/" # for desktop Ubuntu

robot_labels = [1,2,3]
duration = 180 # duration for the simulation in sec
testing_dataset = RW_Dataset_Manager('testing')
start_time, starting_states, dataset_data, time_arr = testing_dataset.load_datasets(dataset_path, robot_labels, duration)
analyzer = Analyzer('analyzer', robot_labels)

loc_algo = EKF_GS_CI2('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)
sim = SimulationManager('sim gs_ci')
state_recorder = StatesRecorder('gs_ci', robot_labels)

sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder, simple_plot = False)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder, plot_graphs = True)

#animate_plot(robot_labels, state_recorder, analyzer, testing_dataset.get_landmark_map())

##############################################################################

testing_dataset.dataset_reset()
loc_algo = GS_CI_Bound('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)

sim = SimulationManager('sim gs_ci_bound')
state_recorder_bound = StatesRecorder('gs_ci_bound', robot_labels, state_var_only = True)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_bound, simple_plot = False)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_bound, plot_graphs = True)

analyzer.algos_comparison([state_recorder, state_recorder_bound], only_trace = ['gs_ci_bound'])
