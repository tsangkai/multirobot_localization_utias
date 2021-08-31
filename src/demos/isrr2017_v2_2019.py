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

# load algorithms 
sys.path.append(os.path.join(os.path.dirname(__file__), "localization_algos"))
from centralized_ekf import Centralized_EKF # works
from ekf_ls_bda import EKF_LS_BDA
from ekf_ls_ci import EKF_LS_CI
from ekf_gs_ci2 import EKF_GS_CI2
from ekf_gs_sci2 import EKF_GS_SCI2


# dataset_path = "/Users/shengkangchen/Documents/CoLo/CoLo-D/CoLo-Datasets/official_dataset3/"
dataset_path = "/home/william/CoLo/CoLo-D/CoLo-Datasets/official_dataset3/" # for desktop Ubuntu

robot_labels = [1,2,3]
duration = 180 # duration for the simulation in sec
testing_dataset = RW_Dataset_Manager('testing')
start_time, starting_states, dataset_data, time_arr = testing_dataset.load_datasets(dataset_path, robot_labels, duration)
analyzer = Analyzer('analyzer', robot_labels)

loc_algo = EKF_GS_CI2('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)
sim = SimulationManager('sim gs_ci')
state_recorder = StatesRecorder('GS_CI', robot_labels)

sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder, simple_plot = False)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder, plot_graphs = True)

#animate_plot(robot_labels, state_recorder, analyzer, testing_dataset.get_landmark_map())

##############################################################################

testing_dataset.dataset_reset()
loc_algo = Centralized_EKF('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('sim cen_ekf')
state_recorder_LS_cen = StatesRecorder('LS_cen', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_cen, simple_plot = False)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_cen, plot_graphs = True)


##############################################################################

testing_dataset.dataset_reset()
loc_algo = EKF_LS_BDA('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('sim ls_bda')
state_recorder_LS_BDA = StatesRecorder('LS_BDA', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_BDA, simple_plot = False)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_BDA, plot_graphs = True)


##############################################################################

testing_dataset.dataset_reset()
loc_algo = EKF_LS_CI('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('sim ls_ci')
state_recorder_LS_CI = StatesRecorder('LS_CI', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_CI, simple_plot = False)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_CI, plot_graphs = True)

##############################################################################

testing_dataset.dataset_reset()
loc_algo = EKF_GS_SCI2('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)

sim = SimulationManager('sim gs_sci')
state_recorder_GS_SCI = StatesRecorder('GS_SCI', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_GS_SCI, simple_plot = False)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_GS_SCI, plot_graphs = True)

analyzer.algos_comparison([state_recorder, state_recorder_LS_cen, state_recorder_LS_BDA, state_recorder_LS_CI, state_recorder_GS_SCI], only_trace = ['gs_ci_bound'])
