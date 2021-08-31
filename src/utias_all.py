'''
TRO2019
@author: kjchen
'''

import os, sys

# load data managers
sys.path.append(os.path.join(os.path.dirname(__file__), "dataset_manager"))
from dataset_manager.realworld_dataset_manager import RW_Dataset_Manager
from robots.robot_system import RobotSystem
from simulation_process.sim_manager import SimulationManager
from simulation_process.state_recorder import StatesRecorder
from data_analysis.data_analyzer import Analyzer
from data_analysis.realtime_plot import animate_plot

# load all algorithms
sys.path.append(os.path.join(os.path.dirname(__file__), "algorithms"))
from LS_Cen import LS_Cen
from LS_CI import LS_CI
from LS_BDA import LS_BDA
from GS_SCI import GS_SCI
from GS_CI import GS_CI


def all_algorithms_comp(dataset_path, robot_labels, duration, graph_name, robots_cant_observe_lm):

    testing_dataset = RW_Dataset_Manager('testing')
    start_time, starting_states, dataset_data, time_arr = testing_dataset.load_datasets(dataset_path, robot_labels, duration)
    analyzer = Analyzer('analyzer', robot_labels)

    loc_algo = GS_CI('algo')
    robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)
    sim = SimulationManager('sim gs_ci')
    state_recorder = StatesRecorder('GS_CI', robot_labels)

    sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder, simple_plot = False ,  robots_cant_observe_lm = robots_cant_observe_lm)
    loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder, plot_graphs = True)

    ##############################################################################

    testing_dataset.dataset_reset()
    loc_algo = LS_Cen('algo')
    robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

    sim = SimulationManager('sim cen_ekf')
    state_recorder_LS_cen = StatesRecorder('LS_cen', robot_labels)
    sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_cen, simple_plot = False, robots_cant_observe_lm = robots_cant_observe_lm)
    loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_cen, plot_graphs = False)

    ##############################################################################

    testing_dataset.dataset_reset()
    loc_algo = LS_BDA('algo')
    robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

    sim = SimulationManager('sim ls_bda')
    state_recorder_LS_BDA = StatesRecorder('LS_BDA', robot_labels)
    sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_BDA, simple_plot = False, robots_cant_observe_lm = robots_cant_observe_lm)
    loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_BDA, plot_graphs = False)


    ##############################################################################

    testing_dataset.dataset_reset()
    loc_algo = LS_CI('algo')
    robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = False)

    sim = SimulationManager('sim ls_ci')
    state_recorder_LS_CI = StatesRecorder('LS_CI', robot_labels)
    sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_LS_CI, simple_plot = False, robots_cant_observe_lm = robots_cant_observe_lm)
    loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_LS_CI, plot_graphs = False)

    ##############################################################################

    testing_dataset.dataset_reset()
    loc_algo = GS_SCI('algo')
    robot = RobotSystem('robot', robot_labels, loc_algo, distr_sys = True)

    sim = SimulationManager('sim gs_sci')
    state_recorder_GS_SCI = StatesRecorder('GS_SCI', robot_labels)
    sim.sim_process_native(robot_labels, testing_dataset, robot, state_recorder_GS_SCI, simple_plot = False, robots_cant_observe_lm = robots_cant_observe_lm)
    loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(state_recorder_GS_SCI, plot_graphs = False)

    analyzer.algos_comparison([state_recorder, state_recorder_LS_cen, state_recorder_LS_BDA, state_recorder_LS_CI, state_recorder_GS_SCI], graph_name = graph_name)

if __name__ == '__main__':
    datasets_path = "../data/utias/MRCLAM_Dataset"

    robot_labels = [1,2,3,4,5]
    duration = 600 # duration for the simulation in sec

    i = 6

    dataset_label = str(i)
    dataset_path = datasets_path + dataset_label + "/"
    graph_name = 'UTIAS_d'+dataset_label
    print(dataset_path)
    all_algorithms_comp(dataset_path, robot_labels, duration, graph_name, robots_cant_observe_lm = [])