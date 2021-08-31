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

if __name__ == '__main__':

    # algorithm initialization
    ls_cen = LS_Cen('ls_cen')
    ls_ci = LS_CI('ls_ci')
    ls_bda = LS_BDA('ls_bda')
    gs_sci = GS_SCI('gs_sci')
    gs_ci = GS_CI('gs_ci')

    # define dataset parameters
    dataset_paths = ["../data/utias/MRCLAM_Dataset" + str(nds) + "/" for nds in range(1,10)]
    robot_labels = [1, 2, 3, 4, 5]
    # robots_cant_observe_lm = [4, 5]
    robots_cant_observe_lm = None
    duration = 600

    # loop through the datasets
    for nds, ds in enumerate(dataset_paths):
        print("\n==== PROCESSING DATASET: " + ds + " ====")

        # load current dataset
        testing_dataset = RW_Dataset_Manager(ds)
        start_time, starting_states, dataset_data, time_arr = testing_dataset.load_datasets(ds, robot_labels, duration)
        analyzer = Analyzer('analyzer', robot_labels)

        # loop through the different algorithms
        sr_all = []
        for na, a in enumerate([ls_cen, ls_ci, ls_bda, gs_sci, gs_ci]):
            print("\n== PROCESSING ALGORITHM: " + str(a) + " ==")

            # indicate if the algorithm is distributed
            if a == ls_cen or a == ls_ci or a == ls_bda:
                distributed = False
            elif a == gs_sci or a == gs_ci:
                distributed = True

            testing_dataset.dataset_reset()
            robot = RobotSystem(str(a), robot_labels, a, distr_sys=distributed)
            sim = SimulationManager(str(a))
            sr = StatesRecorder(str(a), robot_labels)
            sim.sim_process_native(robot_labels, testing_dataset, robot, sr, simple_plot=False, robots_cant_observe_lm=robots_cant_observe_lm)
            loc_err_per_run, state_err_per_run, trace_per_run, time_arr = analyzer.calculate_loc_err_and_trace_state_variance_per_run(sr)

            sr_all.append(sr)

        # compare all algorithm
        analyzer.algos_comparison(sr_all, graph_name="UTIAS_d" + str(nds+1))

