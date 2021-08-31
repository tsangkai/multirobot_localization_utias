#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Apr 17 2019

@author: Sagar
"""

import os, sys
import getpass
import IPython
import numpy as np
from math import pi, sqrt
from matplotlib import pyplot as plt
from matplotlib import animation
import IPython

# sys.path.append(os.path.join(os.path.dirname(__file__), "."))
from dataset_manager.realworld_dataset_manager import RW_Dataset_Manager
from dataset_manager.simulated_dataset_manager import SimulatedDataSetManager
from simulation_process.sim_manager import  SimulationManager
from robots.robot_system import RobotSystem
from simulation_process.state_recorder import StatesRecorder
from data_analysis.data_analyzer import Analyzer
from data_analysis.realtime_plot import animate_plot

from pprint import pprint

# load algorithms 
# NOTE
# GS = global state -> distributed system
# LS = localized state  -> centralized system.
sys.path.append(os.path.join(os.path.dirname(__file__), "localization_algos"))
from centralized_ekf import Centralized_EKF # works
from simple_ekf import Simple_EKF
from ekf_ls_bda import EKF_LS_BDA
from ekf_ls_ci import EKF_LS_CI
from ekf_gs_bound import EKF_GS_BOUND
from ekf_gs_ci2 import EKF_GS_CI2


# Template code for running/debugging simulated data

# Set a random seed for replication
np.random.seed(1)

# Initialize landmark map, landmark IDs must be larger than 5
# Format {LandmarkID: [x,y]}
landmarks = {11: [1,0], 12: [0,1], 13: [-1,0], 14: [0,-1]}

# Initalize Robots, can be 1-5
robot_labels = [1]

# Specify simulation parameters:
# Units are [m], [s], and [rad]
duration = 120
delta_t = 0.2

v_noise = 0.05
w_noise = 0.0001
r_noise = 0.05
phi_noise = sqrt(2)*pi/180
fov = 2*pi

v = 0.1
sigma_v = 0.1

# Instantiate Simulated Dataset Manager
testing_dataset = SimulatedDataSetManager('name', landmarks, duration, robot_labels, 
velocity_noise=v_noise, angular_velocity_noise=w_noise, measurement_range_noise=r_noise, bearing_noise=phi_noise,
robot_fov=fov, delta_t=delta_t)

# Generate the data
start_time, starting_states, dataset_data, time_arr = testing_dataset.simulate_dataset('random', test=False, velocity=v, velocity_spread=sqrt(sigma_v))

# Create Data Analyzer
analyzer = Analyzer('analyzer', robot_labels)

# Specify Loc Algo and create RobotSystem
loc_algo =  EKF_GS_CI2('EKF_GS_CI2')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)

# Create the simulation manager and state recorder
sim = SimulationManager('sim')
state_recorder = StatesRecorder('EKF_GS_CI2', robot_labels)

# Run the simulation
end_time = sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder, simple_plot = False, comm=False, simulated_comm = False)

# Analyze the data
loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder, plot_graphs = False)


# View an animated display of robot movement and loc algo performance
# animate_plot(robot_labels, state_recorder, analyzer, testing_dataset.get_landmark_map())




'''
# Input: loc1=[x1,y1], loc2=[x2,y2]
# Output: Euclidean distance between two points
def calc_distance(loc1, loc2):

    x1 = loc1[0]
    y1 = loc1[1]

    x2 = loc2[0]
    y2 = loc2[1]

    distance = sqrt((x1-x2)**2 + (y1-y2)**2)
    return distance

np.random.seed(1)

# landmark IDs must be bigger than 5
# landmarks = {11: [120,2]}
landmarks = {11: [0,-1], 12: [1,0], 13: [1,1], 14: [0,1]}
# landmarks = {11: [1,0], 12: [0,1], 13: [-1,0], 14: [0,-1]}

robot_labels = [1]
duration = 120

delta_t = 0.2

# NOTE - for documentation: you need at least 3 robots to run communication


testing_dataset = SimulatedDataSetManager('testing', landmarks, duration, robot_labels, 
velocity_noise=.05, angular_velocity_noise=0.00001, measurement_range_noise=.05, bearing_noise=sqrt(2) *pi/180
,robot_fov=2*pi, delta_t=delta_t)

start_time, starting_states, dataset_data, time_arr = testing_dataset.simulate_dataset('circle', test=False, 
velocity=.1, velocity_spread=0.0)

analyzer = Analyzer('analyzer', robot_labels)

loc_algo =  Centralized_EKF('Centralized_EKF')
robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

sim = SimulationManager('sim')
state_recorder = StatesRecorder('Centralized_EKF', robot_labels)

end_time = sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder, simple_plot = True, comm=False, simulated_comm = False)

loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder, plot_graphs = False)

# animate_plot(robot_labels, state_recorder, analyzer, testing_dataset.get_landmark_map())
'''


'''
# Real-World DataSet
# dataset_path = "D:/LEMUR/CoLo/CoLo-D/CoLo-Datasets/official_dataset1/"
# testing_dataset = RW_Dataset_Manager('testing')
# start_time, starting_states, dataset_data, time_arr = testing_dataset.load_datasets(dataset_path, robot_labels, duration,delay_start=20)

# Investigation of RMS/trace corresponding to meas_range
x_diffs = diffs['x_diff']
y_diffs = diffs['y_diff']

times = np.arange(0, 120, delta_t)

x_diffs = list(filter(lambda a: a != -1, x_diffs))
x_diffs = [np.asscalar(x) for x in x_diffs]
y_diffs = list(filter(lambda a: a != -1, y_diffs))
y_diffs = [np.asscalar(y) for y in y_diffs]

robot_x = [gt['x_pos'] for gt in dataset_data['groundtruth'][0]]
robot_y = [gt['y_pos'] for gt in dataset_data['groundtruth'][0]]

robot_locs = [ [robot_x[i], robot_y[i] ] for i in range(0, len(robot_x))]
distances = [calc_distance(landmarks[11], robot_loc) for robot_loc in robot_locs]


fig = plt.figure(2)

plt.plot(times, x_diffs, times, y_diffs)

plt.xlabel('t [s]')
plt.ylabel('|z-z_hat| [m]')

plt.legend(['x', 'y'], loc='upper right')

fig = plt.figure(3)

plt.plot(times, distances)
plt.xlabel('t [s]')
plt.ylabel('robot distance to landmark [m]')

plt.show()
'''