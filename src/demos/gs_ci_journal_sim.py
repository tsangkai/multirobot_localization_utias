#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  8 17:55:06 2018

@author: william
"""

import os, sys
import getpass
sys.path.append(os.path.join(os.path.dirname(__file__), "."))
from dataset_manager.simulated_dataset_manager_w import Sim_Dataset_Manager
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
from gs_ci_bound import GS_CI_Bound
from ekf_gs_sci2 import EKF_GS_SCI2


##############################################################################
duration = 100
robot_labels = [1,2,3,4]
testing_dataset = Sim_Dataset_Manager('testing')
start_time, starting_states, dataset_data, time_arr = testing_dataset.circular_path_4_robots(duration)
analyzer = Analyzer('analyzer', robot_labels)

loc_algo = EKF_GS_CI2('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)
sim = SimulationManager('sim gs_ci')
state_recorder = StatesRecorder('gs_ci', robot_labels)

sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder, simple_plot = False)
data_in_time_order= state_recorder.get_data_in_time_order()

loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder, plot_graphs = False, selected_labels = [1])

#analyzer.trajectory_plot(state_recorder)


##############################################################################

testing_dataset.dataset_reset()
loc_algo = GS_CI_Bound('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)

sim = SimulationManager('sim gs_ci_bound')
state_recorder_bound = StatesRecorder('gs_ci_bound', robot_labels, state_var_only = True)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_bound, simple_plot = False)
loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_bound, plot_graphs = False, selected_labels = [1])
##############################################################################


testing_dataset.dataset_reset()
loc_algo = Centralized_EKF('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('sim cen_ekf')
state_recorder_LS_cen = StatesRecorder('LS_cen', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_cen, simple_plot = False)
loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_cen, plot_graphs = False, selected_labels = [1] )

##############################################################################

testing_dataset.dataset_reset()
loc_algo = EKF_LS_BDA('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('sim ls_bda')
state_recorder_LS_BDA = StatesRecorder('LS_BDA', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_BDA, simple_plot = False)
loc_err_per_run, state_err_per_run, trace_per_run, time_arr =  analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_BDA, plot_graphs = False,  selected_labels = [1])


##############################################################################

testing_dataset.dataset_reset()
loc_algo = EKF_LS_CI('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('sim ls_ci')
state_recorder_LS_CI = StatesRecorder('LS_CI', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_CI, simple_plot = False)
loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_CI, plot_graphs = False,  selected_labels = [1])

##############################################################################

testing_dataset.dataset_reset()
loc_algo = EKF_GS_SCI2('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)

sim = SimulationManager('sim gs_sci')
state_recorder_GS_SCI = StatesRecorder('GS_SCI', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_GS_SCI, simple_plot = False)
loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_GS_SCI, plot_graphs = False,  selected_labels = [1])

analyzer.algos_comparison([state_recorder, state_recorder_LS_cen, state_recorder_LS_BDA, state_recorder_LS_CI, state_recorder_GS_SCI, state_recorder_bound], only_trace = ['gs_ci_bound'], selected_labels = [1])



#analyzer.algos_comparison([state_recorder], selected_labels = [1])