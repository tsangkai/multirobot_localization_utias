#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 10 15:37:32 2018

@author: william
"""

import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), "../"))
from dataset_manager.existing_dataset import Dataset
from simulation_process.sim_manager import  SimulationManager
from robots.robot_system import RobotSystem
from simulation_process.state_recorder import StatesRecorder
from data_analysis.data_analyzer import Analyzer
from data_analysis.realtime_plot import animate_plot
from pprint import pprint

# load algorithms 
sys.path.append(os.path.join(os.path.dirname(__file__), "localization_algos"))
from centralized_ekf2 import Centralized_EKF2
from simple_ekf import Simple_EKF
from ekf_ls_bda import EKF_LS_BDA
from ekf_gs_bound import EKF_GS_BOUND
from ekf_gs_ci2 import EKF_GS_CI2

#need to verify these algo
'''
from ekf_ls_ci import EKF_LS_CI
from ekf_ls_ci2 import EKF_LS_CI2
from ekf_gs_ci import EKF_GS_CI
from ekf_gs_sci2 import EKF_GS_SCI2

'''


dataset_path = '/home/william/UTIAS-dataset/MRCLAM_Dataset3/'

dataset_labels = [1,2,3]
duration = 200 # duration for the simulation in sec
testing_dataset = Dataset('testing')
start_time, starting_states, dataset_data, time_arr = testing_dataset.load_MRCLAMDatasets(dataset_path, dataset_labels, duration, synthetic = False)

freqs0 = [[10, 10, 10],[1, 1, 1],[0.5, 0.5, 0.5]]
freqs1 = [[10, 10, 10],[4, 4, 4],[0.5, 0.5, 0.5]]



loc_algo = EKF_GS_CI2('algo')
robot = RobotSystem('robot gs ci', dataset_labels, loc_algo, distr_sys = True)

sim = SimulationManager('sim')
state_recorder = StatesRecorder('gs ci schedule freqs0',dataset_labels)
sim.sim_process_schedule(dataset_labels, testing_dataset, robot, state_recorder, freqs0, simple_plot = True)


##############################################################################
testing_dataset.dataset_reset()
robot = RobotSystem('robot gs ci', dataset_labels, loc_algo, distr_sys = True)

sim1 = SimulationManager('sim')
state_recorder1 = StatesRecorder('gs ci schedule freq1',dataset_labels)
sim1.sim_process_schedule(dataset_labels, testing_dataset, robot, state_recorder1, freqs1, simple_plot = True)

##############################################################################
testing_dataset.dataset_reset()
robot = RobotSystem('robot gs ci', dataset_labels, loc_algo, distr_sys = True)

sim_n = SimulationManager('sim')
state_recorder_n = StatesRecorder('gs ci naive ',dataset_labels)
sim_n.sim_process_naive(dataset_labels, testing_dataset, robot, state_recorder_n, simple_plot = True)

print("simulation completetd")

##############################################################################
'''
testing_dataset.dataset_reset()
bound_algo = EKF_GS_BOUND('algo')
robot_bound = RobotSystem('robot bound', dataset_labels, bound_algo, distr_sys = True)

sim_b = SimulationManager('sim')
state_recorder_bound = StatesRecorder('gs ci bound',dataset_labels)
sim_b.sim_process_schedule(dataset_labels, testing_dataset, robot_bound, state_recorder_bound, freqs)


##############################################################################
testing_dataset.dataset_reset()
cen_ekf_algo = Centralized_EKF2('algo')
robot_cen = RobotSystem('robot cen', dataset_labels, cen_ekf_algo, distr_sys = False)

sim_c = SimulationManager('sim')
state_recorder_c= StatesRecorder('cen ekf',dataset_labels)
sim_c.sim_process_naive(dataset_labels, testing_dataset, robot_cen, state_recorder_c)



##############################################################################
testing_dataset.dataset_reset()
bda_algo = EKF_LS_BDA('algo')
robot_bda = RobotSystem('robot bda', dataset_labels, bda_algo, distr_sys = False)

sim_bda = SimulationManager('sim')
state_recorder_bda= StatesRecorder('bda',dataset_labels)
sim_bda.sim_process_naive(dataset_labels, testing_dataset, robot_bda, state_recorder_bda)
'''


analyzer = Analyzer('analyzer', dataset_labels)
analyzer.algos_comparison([state_recorder, state_recorder1, state_recorder_n], only_trace=['gs ci bound'])
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder)
robot_loc_time_unit = analyzer.robot_location_at_unit_time_interval(state_recorder)

print("start animation")
animate_plot(dataset_labels, state_recorder, analyzer)
