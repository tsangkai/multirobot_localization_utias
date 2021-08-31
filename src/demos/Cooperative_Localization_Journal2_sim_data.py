
import os, sys
import getpass
from math import pi, sqrt
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), "."))
from dataset_manager.realworld_dataset_manager import RW_Dataset_Manager
from dataset_manager.simulated_dataset_manager import SimulatedDataSetManager
from simulation_process.sim_manager import  SimulationManager
from robots.robot_system import RobotSystem
from simulation_process.state_recorder import StatesRecorder
from data_analysis.data_analyzer import Analyzer
from data_analysis.realtime_plot import animate_plot

import comparison_plot

# load algorithms 
sys.path.append(os.path.join(os.path.dirname(__file__), "localization_algos"))
from centralized_ekf import Centralized_EKF # works
from ekf_ls_bda import EKF_LS_BDA
from ekf_ls_ci import EKF_LS_CI
from ekf_gs_ci2 import EKF_GS_CI2
from gs_ci_bound import GS_CI_Bound
from ekf_gs_sci2 import EKF_GS_SCI2

np.random.seed(1)

landmarks = {10: [0,5] , 11: [20,3], 12: [-30,0], 13: [-13, -12] }

robot_labels = [1,2,3,4,5]
boundary = 60 
duration = 120
testing_dataset = SimulatedDataSetManager('testing', boundary, landmarks, duration, robot_labels, 
velocity_noise=sqrt(0.0125), angular_velocity_noise=0.001, measurement_range_noise=sqrt(0.1), bearing_noise=sqrt(2)*pi/180, 
robot_fov=2*pi/3, delta_t=0.2)
start_time, starting_states, dataset_data, time_arr = testing_dataset.simulate_dataset('random', test=False, velocity=1, velocity_spread=0.5)

analyzer = Analyzer('analyzer', robot_labels)

# TODO - implment ISSR 2017 topography
robots_cant_observe_lm = None
graph_name = 'Algo Comparison'

#############################################################################

loc_algo = EKF_GS_CI2('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)
sim = SimulationManager('sim gs_ci')
state_recorder_GS_CI2 = StatesRecorder('GS_CI', robot_labels)

sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_GS_CI2, simple_plot = False ,  robots_cant_observe_lm = robots_cant_observe_lm)
loc_err_per_run, state_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_GS_CI2, plot_graphs = False)

##############################################################################
'''

loc_algo = GS_CI_Bound('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)

sim = SimulationManager('sim gs_ci_bound')
state_recorder_bound = StatesRecorder('GS_CI_bound', robot_labels, state_var_only = True)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_bound, simple_plot = False, robots_cant_observe_lm = robots_cant_observe_lm)
loc_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_bound, plot_graphs = False)
'''
##############################################################################


loc_algo = Centralized_EKF('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('sim cen_ekf')
state_recorder_LS_cen = StatesRecorder('LS_cen', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_cen, simple_plot = False, robots_cant_observe_lm = robots_cant_observe_lm)
loc_err_per_run, state_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_cen, plot_graphs = False)

##############################################################################


loc_algo = EKF_LS_BDA('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('sim ls_bda')
state_recorder_LS_BDA = StatesRecorder('LS_BDA', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_BDA, simple_plot = False, robots_cant_observe_lm = robots_cant_observe_lm)
loc_err_per_run, state_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_BDA, plot_graphs = False)


##############################################################################


loc_algo = EKF_LS_CI('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('sim ls_ci')
state_recorder_LS_CI = StatesRecorder('LS_CI', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_CI, simple_plot = False, robots_cant_observe_lm = robots_cant_observe_lm)
loc_err_per_run, state_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_CI, plot_graphs = False)

##############################################################################


loc_algo = EKF_GS_SCI2('algo')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)

sim = SimulationManager('sim gs_sci')
state_recorder_GS_SCI = StatesRecorder('GS_SCI', robot_labels)
sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_GS_SCI, simple_plot = False, robots_cant_observe_lm = robots_cant_observe_lm)
loc_err_per_run, state_err_per_run, trace_per_run, t_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_GS_SCI, plot_graphs = False)

analyzer.algos_comparison([state_recorder_GS_CI2, state_recorder_LS_cen, state_recorder_LS_BDA, state_recorder_LS_CI, state_recorder_GS_SCI], graph_name = graph_name)
