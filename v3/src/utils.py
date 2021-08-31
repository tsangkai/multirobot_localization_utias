import sys
from pathlib import Path

import numpy as np
import math as m
import matplotlib.pyplot as plt

from robot_system import *

from algorithms.lscen import LocalStateCentralizedEquivalent
from algorithms.lsci import LocalStateCovarianceIntersection
from algorithms.lsbda import LocalStateBlockDiagonalApproximation
from algorithms.gssci import GlobalStateSplitCovarianceIntersection
from algorithms.gsci import GlobalStateCovarianceIntersection
from algorithms.lssci import LocalStateSplitCovarianceIntersection


def load_datasets(path, datasets):
    '''
    Loads the specified UTIAS MRCLAM dataset.

    Parameters
    ----------
    path : str
        The parent path to the UTIAS dataset.
    datasets : list
        The list of UTIAS MRCLAM datasets to load.

    Returns
    -------
    UTIAS : dict
        Data structure containing the full UTIAS dataset.

    '''

    UTIAS = dict()

    print('Loading UTIAS MRCLAM Datasets: ' + str(datasets) + '...', end=' ', flush=True)

    for d in datasets:

        dataset = dict()

        # load the dataset
        dataset_path = path + 'MRCLAM_Dataset' + str(d) + '/'

        # load Barcodes.dat
        dataset['Barcodes'] = list()
        with open(dataset_path + 'Barcodes.dat', 'r+') as f:
            for line in f:
                if line[0] != '#':
                    subject_num = int( line.split()[0] )
                    barcode_num = int( line.split()[1] )

                    dataset['Barcodes'].append( {'Subject #': subject_num, 'Barcode #': barcode_num} )

        # load Landmark_Groundtruth.dat
        dataset['Landmark_Groundtruth'] = list()
        with open(dataset_path + 'Landmark_Groundtruth.dat', 'r+') as f:
            for line in f:
                if line[0] != '#':
                    subject_num = int( line.split()[0] )
                    x = float( line.split()[1] )
                    y = float( line.split()[2] )
                    x_stddev = float( line.split()[3] )
                    y_stddev = float( line.split()[4] )

                    dataset['Landmark_Groundtruth'].append( {'Subject #': subject_num, 'x [m]': x, 'y [m]': y,
                                                             'x std-dev [m]': x_stddev, 'y std-dev [m]': y_stddev} )

        # load Robot data
        for r in range(NUM_ROBOTS):
            robot = 'Robot' + str(r+1)
            dataset[robot] = dict()
            dataset[robot]['Groundtruth'] = list()
            dataset[robot]['Measurement'] = list()
            dataset[robot]['Odometry'] = list()

            # RobotX_Groundtruth.dat
            with open(dataset_path + robot + '_Groundtruth.dat', 'r+') as f:
                for line in f:
                    if line[0] != '#':
                        time = float( line.split()[0] )
                        x = float( line.split()[1] )
                        y = float( line.split()[2] )
                        orientation = float( line.split()[3] )

                        dataset[robot]['Groundtruth'].append( {'Time [s]': time, 'x [m]': x, 'y [m]': y, 'orientation [rad]': orientation} )

            # RobotX_Measurement.dat
            with open(dataset_path + robot + '_Measurement.dat', 'r+') as f:
                for line in f:
                    if line[0] != '#':
                        time = float( line.split()[0] )
                        subject_num = int (line.split()[1] )
                        _range = float( line.split()[2] )
                        bearing = float( line.split()[3] )

                        dataset[robot]['Measurement'].append( {'Time [s]': time, 'Subject #': subject_num, 'range [m]': _range, 'bearing [rad]': bearing} )

            # RobotX_Odometry.dat
            with open(dataset_path + robot + '_Odometry.dat', 'r+') as f:
                for line in f:
                    if line[0] != '#':
                        time = float( line.split()[0] )
                        f_vel = float( line.split()[1] )
                        a_vel = float( line.split()[2] )

                        dataset[robot]['Odometry'].append( {'Time [s]': time, 'forward velocity [m/s]': f_vel, 'angular velocity [rad/s]': a_vel} )

        # append to UTIAS dataset
        UTIAS['MRCLAM_Dataset' + str(d)] = dataset

    print(u'\u2713\n')

    return UTIAS


def execute(UTIAS, algorithms, comm_type, comm_range, comm_rate, comm_prob_fail, dt, offset, duration):
    '''
    Executes every algorithm on every dataset.

    Parameters
    ----------
    UTIAS : dict
        Data structure of loaded UTIAS MRCLAM datasets.
    algorithms : str
        List of algorithms to process for each UTIAS MRCLAM dataset.
    comm_type : str
        The type of communication for distributive localization algorithms.
    comm_range : float
        Range of communication. Only useful if the communication type is "range".
    comm_rate : float
        Rate of communication (seconds). Set to 0 to default to the time precision.
    comm_prob_fail : float
        Probability of communication failure for each algorithm, between 0 and 1.
    dt : float
        The time precision of each iteration (seconds).
    offset : float
        The offset of the execution start time in the dataset (seconds).
    duration : float
        The execution time starting from the offset. Set to zero to use the entire dataset.

    Returns
    -------
    results : dict
        Dictionary containing the results of each algorithm for each dataset.

    '''

    results = dict()

    for MRCLAM in UTIAS:

        results[MRCLAM] = dict()

        for alg in algorithms:

            print('Dataset {} // Algorithm {}'.format(MRCLAM[-1], alg.upper()))

            # generate the results for every dataset using each algorithm
            results[MRCLAM][alg] = generate_results(UTIAS[MRCLAM], alg, comm_type, comm_range, comm_rate, comm_prob_fail, dt, offset, duration)

            sys.stdout.write("\033[F")
            print('Dataset {} // Algorithm {} {}'.format(MRCLAM[-1], alg.upper(), u'\u2713'))

        print('')

    return results


def generate_results(dataset, alg, comm_type, comm_range, comm_rate, comm_prob_fail, dt, offset, duration):
    '''
    Generates the results for the specified MRCLAM dataset and algorithm.

    Parameters
    ----------
    dataset : dict
        Loaded MRCLAM dataset.
    alg : str
        The algorithm to run on the dataset.
    comm_type : str
        The type of communication for distributive localization algorithms.
    comm_range : float
        Range of communication. Only useful if the communication type is "range".
    comm_rate : float
        Rate of communication (seconds). Set to 0 to default to the time precision.
    comm_prob_fail : float
        Probability of communication failure for each algorithm, between 0 and 1.
    dt : float
        The time precision of each iteration (seconds).
    offset : float
        The offset of the execution start time in the dataset (seconds).
    duration : float
        The execution time starting from the offset. Set to zero to use the entire dataset.

    Returns
    -------
    result: dict
        A dictionary containing the results. Currently holds the error plots against ground truth.

    '''

    # team settings
    team_settings = {'dt': dt, 'offset': offset, 'duration': duration, 'comm_type': comm_type, 'comm_range': comm_range, 'comm_rate': comm_rate, 'comm_prob_fail': comm_prob_fail}

    # get dataset start and end times
    start_time, end_time = u.get_times(dataset, offset, duration)

    # run algorithm
    if alg == 'lscen':
        robot_system = CentralizedTeam(dataset, team_settings)
        LSCEN = LocalStateCentralizedEquivalent(robot_system, dataset)
        LSCEN.run(start_time, end_time)
        robot_system = LSCEN.robot_system

    elif alg == 'lsci':
        robot_system = CentralizedTeam(dataset, team_settings)
        LSCI = LocalStateCovarianceIntersection(robot_system, dataset)
        LSCI.run(start_time, end_time)
        robot_system = LSCI.robot_system

    elif alg == 'lsbda':
        robot_system = CentralizedTeam(dataset, team_settings)
        LSBDA = LocalStateBlockDiagonalApproximation(robot_system, dataset)
        LSBDA.run(start_time, end_time)
        robot_system = LSBDA.robot_system

    elif alg == 'gssci':
        robot_system = DistributiveTeam(dataset, team_settings)
        GSSCI = GlobalStateSplitCovarianceIntersection(robot_system, dataset)
        GSSCI.run(start_time, end_time)
        robot_system = GSSCI.robot_system

    elif alg == 'gsci':
        robot_system = DistributiveTeam(dataset, team_settings)
        GSCI = GlobalStateCovarianceIntersection(robot_system, dataset)
        GSCI.run(start_time, end_time)
        robot_system = GSCI.robot_system
        
    elif alg == 'lssci':
        robot_system = CentralizedTeam(dataset, team_settings)
        LSSCI = LocalStateSplitCovarianceIntersection(robot_system, dataset)
        LSSCI.run(start_time, end_time)
        robot_system = LSSCI.robot_system

    else:
        sys.exit('Invalid algorithm input!')

    return robot_system


def get_times(dataset, offset, duration):
    '''
    Retrieves the start and end time of a dataset.

    Parameters
    ----------
    dataset : dict
        The UTIAS MRCLAM dataset.
    offset : float
        The starting time offset.
    duration : float
        The duration of the test time, starting from the offset. If None, then use the entire dataset.

    Returns
    -------
    start_time : float
        The latest start time in Unix epoch.
    end_time : float
        The earliest end time in Unix epoch.

    '''

    start_times, end_times = list(), list()

    for r in range(NUM_ROBOTS):

        start_times.append(dataset['Robot' + str(r+1)]['Groundtruth'][0]['Time [s]'])
        start_times.append(dataset['Robot' + str(r+1)]['Measurement'][0]['Time [s]'])
        start_times.append(dataset['Robot' + str(r+1)]['Odometry'][0]['Time [s]'])

        end_times.append(dataset['Robot' + str(r+1)]['Groundtruth'][-1]['Time [s]'])
        end_times.append(dataset['Robot' + str(r+1)]['Measurement'][-1]['Time [s]'])
        end_times.append(dataset['Robot' + str(r+1)]['Odometry'][-1]['Time [s]'])

    start_time = np.max(start_times)
    end_time = np.min(end_times)

    if offset:
        start_time += offset

    if duration:
        end_time = start_time + duration

    return start_time, end_time


def analyze(results, settings, save=False):
    '''
    Analyzes (and plots) the results for each dataset with each algorithm.

    Parameters
    ----------
    results : dict
        Contains robot systems for each algorithm for each dataset.
    settings : list
        Contains the settings of the results, i.e. {comm_type, comm_range, comm_rate, dt, offset, duration}
    save : bool, optional
        Saves the plots as files to disk. The default is False.

    Returns
    -------
    None.

    '''

    if save:
        print('Analyzing and plotting results...')
    else:
        print('Analyzing results...')


    # extract settings
    comm_type, comm_range, comm_rate, comm_prob_fail, dt, offset, duration = settings

    for MRCLAM in results:

        dataset = results[MRCLAM]

        # individual trajectory plots and L2 error plots
        for r in range(NUM_ROBOTS):

            # individual trajectory plots
            plt.figure()
            
            # plot landmarks
            l_x, l_y = list(), list()
            for l in range(len(results[MRCLAM][list(dataset.keys())[0]].landmarks)):
                l_xy = results[MRCLAM][list(dataset.keys())[0]].landmarks[l].xy
                l_x.append(l_xy[1])
                l_y.append(-1.0 * l_xy[0])
            
            plt.scatter(np.squeeze(l_x), np.squeeze(l_y), color='k', marker='^', label='Landmark')
                
            # plot barriers if dataset 9
            if int(MRCLAM[-1]) == int(9):
                plt.xlim([-6, 6])
                plt.ylim([-6, 2])
                
                plt.plot([l_x[0], l_x[3]], [l_y[0], l_y[3]], color='k', alpha=0.5)
                plt.plot([l_x[3], l_x[4]], [l_y[3], l_y[4]], color='k', alpha=0.5)
                plt.plot([l_x[4], l_x[9]], [l_y[4], l_y[9]], color='k', alpha=0.5)
                plt.plot([l_x[9], l_x[11]], [l_y[9], l_y[11]], color='k', alpha=0.5)
                plt.plot([l_x[11], l_x[12]], [l_y[11], l_y[12]], color='k', alpha=0.5)
                plt.plot([l_x[12], l_x[13]], [l_y[12], l_y[13]], color='k', alpha=0.5)
                plt.plot([l_x[13], l_x[14]], [l_y[13], l_y[14]], color='k', alpha=0.5)
                plt.plot([l_x[14], l_x[6]], [l_y[14], l_y[6]], color='k', alpha=0.5)
                plt.plot([l_x[6], l_x[5]], [l_y[6], l_y[5]], color='k', alpha=0.5)
                plt.plot([l_x[5], l_x[2]], [l_y[5], l_y[2]], color='k', alpha=0.5)
                plt.plot([l_x[2], l_x[0]], [l_y[2], l_y[0]], color='k', alpha=0.5)
                
                plt.plot([l_x[1], l_x[4]], [l_y[1], l_y[4]], color='k', alpha=0.5)
                
                plt.plot([l_x[7], l_x[8]], [l_y[7], l_y[8]], color='k', alpha=0.5)
                plt.plot([l_x[8], l_x[10]], [l_y[8], l_y[10]], color='k', alpha=0.5)

            # plot ground truth
            h = 'gt'
            x_gt = np.squeeze([results[MRCLAM][list(dataset.keys())[0]].robots[r].history[h][k]['x'] for k in range(len(results[MRCLAM][list(dataset.keys())[0]].robots[r].history[h]))])
            y_gt = np.squeeze([results[MRCLAM][list(dataset.keys())[0]].robots[r].history[h][k]['y'] for k in range(len(results[MRCLAM][list(dataset.keys())[0]].robots[r].history[h]))])
            plt.plot(y_gt, -1.0 * x_gt, color='k', label='GT')

            for alg in dataset:

                h = 'est'
                x_est = np.squeeze([results[MRCLAM][alg].robots[r].history[h][k]['x'] for k in range(len(results[MRCLAM][alg].robots[r].history[h]))])
                y_est = np.squeeze([results[MRCLAM][alg].robots[r].history[h][k]['y'] for k in range(len(results[MRCLAM][alg].robots[r].history[h]))])
                
                '''
                if alg == 'lscen':
                    plt.plot(x_est, y_est, '--', color='m', label='LS-Cen')
                elif alg == 'lsci':
                    plt.plot(x_est, y_est, '--', color='r', label='LS-CI')
                elif alg == 'lsbda':
                    plt.plot(x_est, y_est, '--', color='g', label='LS-BDA')
                elif alg == 'gssci':
                    plt.plot(x_est, y_est, '--', color='y', label='GS-SCI')
                elif alg == 'gsci':
                    plt.plot(x_est, y_est, color='b', label='GS-CI')
                '''
                
                if alg == 'lscen':
                    plt.plot(y_est, -1.0 * x_est, '--', color='m', label='LS-Cen')
                elif alg == 'lsci':
                    plt.plot(y_est, -1.0 * x_est, '--', color='r', label='LS-CI')
                elif alg == 'lsbda':
                    plt.plot(y_est, -1.0 * x_est, '--', color='g', label='LS-BDA')
                elif alg == 'gssci':
                    plt.plot(y_est, -1.0 * x_est, '--', color='y', label='GS-SCI')
                elif alg == 'lssci':
                    plt.plot(y_est, -1.0 * x_est, '--', color='y', label='LS-SCI')
                elif alg == 'gsci':
                    plt.plot(y_est, -1.0 * x_est, color='b', label='GS-CI')

                

            plt.title('Trajectory of Robot {}, Dataset {}'.format(str(r+1), MRCLAM[-1]), fontsize=16)
            plt.xlabel('x-axis [m]', fontsize=14)
            plt.ylabel('y-axis [m]', fontsize=14)
            plt.legend()

            if save:
                save_path = '../results/' + 'type-' + str(comm_type) + '_' + 'range-' + str(comm_range) + '_' + 'rate-' + str(comm_rate) + '_' + 'probfail-' + str(comm_prob_fail) + '_' 'dt-' + str(dt) + '_' + 'offset-' + str(offset) + '_' + 'duration-' + str(duration) + '/' + MRCLAM + '/'
                fig_name = 'robot' + str(r+1) + '_trajectory'
                Path(save_path).mkdir(parents=True, exist_ok=True)
                plt.savefig(save_path + fig_name + '.pdf')

                np.savetxt(save_path + fig_name + '_x.txt', y_est, fmt='%.5f', delimiter=',')
                np.savetxt(save_path + fig_name + '_y.txt', -1.0*x_est, fmt='%.5f', delimiter=',')

            # L2 error plots
            plt.figure()

            for alg in dataset:

                time = np.array([results[MRCLAM][alg].robots[r].history['gt'][k]['time'] for k in range(len(results[MRCLAM][alg].robots[r].history['gt']))])
                time_arr = time - np.min(time) + offset

                h = 'est'
                x_est = np.squeeze([results[MRCLAM][alg].robots[r].history[h][k]['x'] for k in range(len(results[MRCLAM][alg].robots[r].history[h]))])
                y_est = np.squeeze([results[MRCLAM][alg].robots[r].history[h][k]['y'] for k in range(len(results[MRCLAM][alg].robots[r].history[h]))])
                alg_error = np.sqrt((np.array(x_est) - np.array(x_gt)) ** 2 + (np.array(y_est) - np.array(y_gt)) ** 2)

                if alg == 'lscen':
                    plt.plot(time_arr, alg_error, color='m', label='LS-Cen')
                elif alg == 'lsci':
                    plt.plot(time_arr, alg_error, color='r', label='LS-CI')
                elif alg == 'lsbda':
                    plt.plot(time_arr, alg_error, color='g', label='LS-BDA')
                elif alg == 'gssci':
                    plt.plot(time_arr, alg_error, color='y', label='GS-SCI')
                elif alg == 'lssci':
                    plt.plot(time_arr, alg_error, color='y', label='LS-SCI')
                elif alg == 'gsci':
                    plt.plot(time_arr, alg_error, color='b', label='GS-CI')

            plt.title('L2 Error of Robot {}, Dataset {}'.format(str(r+1), MRCLAM[-1]), fontsize=16)
            plt.xlabel('Time [s]', fontsize=14)
            plt.ylabel('Error [m]', fontsize=14)
            plt.legend()

            if save:
                save_path = '../results/' + 'type-' + str(comm_type) + '_' + 'range-' + str(comm_range) + '_' + 'rate-' + str(comm_rate) + '_' + 'probfail-' + str(comm_prob_fail) + '_' 'dt-' + str(dt) + '_' + 'offset-' + str(offset) + '_' + 'duration-' + str(duration) + '/' + MRCLAM + '/'
                fig_name = 'robot' + str(r+1) + '_error'
                Path(save_path).mkdir(parents=True, exist_ok=True)
                plt.savefig(save_path + fig_name + '.pdf')


        # combined trajectory plot
        plt.figure()
        
        # plot landmarks
        l_x, l_y = list(), list()
        for l in range(len(results[MRCLAM][list(dataset.keys())[0]].landmarks)):
            l_xy = results[MRCLAM][list(dataset.keys())[0]].landmarks[l].xy
            l_x.append(l_xy[1])
            l_y.append(-1.0 * l_xy[0])
        
        plt.scatter(np.squeeze(l_x), np.squeeze(l_y), color='k', marker='^', label='Landmark')
            
        # plot barriers if dataset 9
        if int(MRCLAM[-1]) == int(9):
            plt.xlim([-6, 6])
            plt.ylim([-6, 2])
            
            plt.plot([l_x[0], l_x[3]], [l_y[0], l_y[3]], color='k', alpha=0.5)
            plt.plot([l_x[3], l_x[4]], [l_y[3], l_y[4]], color='k', alpha=0.5)
            plt.plot([l_x[4], l_x[9]], [l_y[4], l_y[9]], color='k', alpha=0.5)
            plt.plot([l_x[9], l_x[11]], [l_y[9], l_y[11]], color='k', alpha=0.5)
            plt.plot([l_x[11], l_x[12]], [l_y[11], l_y[12]], color='k', alpha=0.5)
            plt.plot([l_x[12], l_x[13]], [l_y[12], l_y[13]], color='k', alpha=0.5)
            plt.plot([l_x[13], l_x[14]], [l_y[13], l_y[14]], color='k', alpha=0.5)
            plt.plot([l_x[14], l_x[6]], [l_y[14], l_y[6]], color='k', alpha=0.5)
            plt.plot([l_x[6], l_x[5]], [l_y[6], l_y[5]], color='k', alpha=0.5)
            plt.plot([l_x[5], l_x[2]], [l_y[5], l_y[2]], color='k', alpha=0.5)
            plt.plot([l_x[2], l_x[0]], [l_y[2], l_y[0]], color='k', alpha=0.5)
            
            plt.plot([l_x[1], l_x[4]], [l_y[1], l_y[4]], color='k', alpha=0.5)
            
            plt.plot([l_x[7], l_x[8]], [l_y[7], l_y[8]], color='k', alpha=0.5)
            plt.plot([l_x[8], l_x[10]], [l_y[8], l_y[10]], color='k', alpha=0.5)
            
        for r in range(NUM_ROBOTS):

            # plot ground truth
            h = 'gt'
            x_gt = np.squeeze([results[MRCLAM][list(dataset.keys())[0]].robots[r].history[h][k]['x'] for k in range(len(results[MRCLAM][list(dataset.keys())[0]].robots[r].history[h]))])
            y_gt = np.squeeze([results[MRCLAM][list(dataset.keys())[0]].robots[r].history[h][k]['y'] for k in range(len(results[MRCLAM][list(dataset.keys())[0]].robots[r].history[h]))])
            plt.plot(y_gt, -1.0 * x_gt, color='k', label='GT' if r == 0 else '_nolegend_')

            for alg in dataset:

                h = 'est'
                x_est = np.squeeze([results[MRCLAM][alg].robots[r].history[h][k]['x'] for k in range(len(results[MRCLAM][alg].robots[r].history[h]))])
                y_est = np.squeeze([results[MRCLAM][alg].robots[r].history[h][k]['y'] for k in range(len(results[MRCLAM][alg].robots[r].history[h]))])

                '''
                if alg == 'lscen':
                    plt.plot(x_est, y_est, '--', color='m', label='LS-Cen' if r == 0 else '_nolegend_')
                elif alg == 'lsci':
                    plt.plot(x_est, y_est, '--', color='r', label='LS-CI' if r == 0 else '_nolegend_')
                elif alg == 'lsbda':
                    plt.plot(x_est, y_est, '--', color='g', label='LS-BDA' if r == 0 else '_nolegend_')
                elif alg == 'gssci':
                    plt.plot(x_est, y_est, '--', color='y', label='GS-SCI' if r == 0 else '_nolegend_')
                elif alg == 'gsci':
                    plt.plot(x_est, y_est, color='b', label='GS-CI' if r == 0 else '_nolegend_')
                '''

                if alg == 'lscen':
                    plt.plot(y_est, -1.0 * x_est, '--', color='m', label='LS-Cen' if r == 0 else '_nolegend_')
                elif alg == 'lsci':
                    plt.plot(y_est, -1.0 * x_est, '--', color='r', label='LS-CI' if r == 0 else '_nolegend_')
                elif alg == 'lsbda':
                    plt.plot(y_est, -1.0 * x_est, '--', color='g', label='LS-BDA' if r == 0 else '_nolegend_')
                elif alg == 'gssci':
                    plt.plot(y_est, -1.0 * x_est, '--', color='y', label='GS-SCI' if r == 0 else '_nolegend_')
                elif alg == 'lssci':
                    plt.plot(y_est, -1.0 * x_est, '--', color='y', label='LS-SCI' if r == 0 else '_nolegend_')
                elif alg == 'gsci':
                    plt.plot(y_est, -1.0 * x_est, color='b', label='GS-CI' if r == 0 else '_nolegend_')

        plt.title('Trajectories of Dataset {}'.format(MRCLAM[-1]), fontsize=16)
        plt.xlabel('x-axis [m]', fontsize=14)
        plt.ylabel('y-axis [m]', fontsize=14)
        plt.legend()

        if save:
            save_path = '../results/' + 'type-' + str(comm_type) + '_' + 'range-' + str(comm_range) + '_' + 'rate-' + str(comm_rate) + '_' + 'probfail-' + str(comm_prob_fail) + '_' 'dt-' + str(dt) + '_' + 'offset-' + str(offset) + '_' + 'duration-' + str(duration) + '/' + MRCLAM + '/'
            fig_name = 'trajectories'
            Path(save_path).mkdir(parents=True, exist_ok=True)
            plt.savefig(save_path + fig_name + '.pdf')

        # RMSE
        plt.figure()
        
        avg_rmse = list()
        std_rmse = list()

        for alg in dataset:

            cum_error = np.zeros(shape=len(time_arr))

            for r in range(NUM_ROBOTS):
                
                time = np.array([results[MRCLAM][alg].robots[r].history['gt'][k]['time'] for k in range(len(results[MRCLAM][alg].robots[r].history['gt']))])
                time_arr = time - np.min(time) + offset

                h = 'gt'
                x_gt = np.squeeze([results[MRCLAM][alg].robots[r].history[h][k]['x'] for k in range(len(results[MRCLAM][alg].robots[r].history[h]))])
                y_gt = np.squeeze([results[MRCLAM][alg].robots[r].history[h][k]['y'] for k in range(len(results[MRCLAM][alg].robots[r].history[h]))])

                h = 'est'
                x_est = np.squeeze([results[MRCLAM][alg].robots[r].history[h][k]['x'] for k in range(len(results[MRCLAM][alg].robots[r].history[h]))])
                y_est = np.squeeze([results[MRCLAM][alg].robots[r].history[h][k]['y'] for k in range(len(results[MRCLAM][alg].robots[r].history[h]))])

                cum_error += (np.array(x_est) - np.array(x_gt)) ** 2 + (np.array(y_est) - np.array(y_gt)) ** 2

            rmse_error = np.sqrt(cum_error * (1./NUM_ROBOTS))

            if alg == 'lscen':
                plt.plot(time_arr, rmse_error, color='m', label='LS-Cen')
            elif alg == 'lsci':
                plt.plot(time_arr, rmse_error, color='r', label='LS-CI')
            elif alg == 'lsbda':
                plt.plot(time_arr, rmse_error, color='g', label='LS-BDA')
            elif alg == 'gssci':
                plt.plot(time_arr, rmse_error, color='y', label='GS-SCI')
            elif alg == 'lssci':
                plt.plot(time_arr, rmse_error, color='y', label='LS-SCI')
            elif alg == 'gsci':
                plt.plot(time_arr, rmse_error, color='b', label='GS-CI')
            
            # average rmse error
            avg_rmse.append( np.mean(rmse_error) )
            std_rmse.append( np.std(rmse_error) )

        plt.title('RMSE of Dataset {}'.format(MRCLAM[-1]), fontsize=16)
        plt.xlabel('Time [s]', fontsize=14)
        plt.ylabel('Error [m]', fontsize=14)
        plt.legend()

        if save:
            save_path = '../results/' + 'type-' + str(comm_type) + '_' + 'range-' + str(comm_range) + '_' + 'rate-' + str(comm_rate) + '_' + 'probfail-' + str(comm_prob_fail) + '_' 'dt-' + str(dt) + '_' + 'offset-' + str(offset) + '_' + 'duration-' + str(duration) + '/' + MRCLAM + '/'
            fig_name = 'rmse'
            Path(save_path).mkdir(parents=True, exist_ok=True)
            plt.savefig(save_path + fig_name + '.pdf')
            
            np.savetxt(save_path + 'avg_rmse.csv', avg_rmse, fmt='%.4f', delimiter=',')
            np.savetxt(save_path + 'std_rmse.csv', std_rmse, fmt='%.4f', delimiter=',')
            
        # RMTE
        plt.figure()

        for alg in dataset:

            cov = np.zeros(shape=(2*NUM_ROBOTS, 2*NUM_ROBOTS, len(time_arr)))

            for r in range(NUM_ROBOTS):
                
                time = np.array([results[MRCLAM][alg].robots[r].history['gt'][k]['time'] for k in range(len(results[MRCLAM][alg].robots[r].history['gt']))])
                time_arr = time - np.min(time) + offset
                
                c = np.array([results[MRCLAM][alg].robots[r].history['est'][k]['cov'] for k in range(len(results[MRCLAM][alg].robots[r].history['est']))])
                cov[2*r:2*r+2, 2*r:2*r+2, :] = np.moveaxis(c, 0, -1)

            rmte_error = np.sqrt(np.trace(cov) * (1./NUM_ROBOTS))

            if alg == 'lscen':
                plt.plot(time_arr, rmte_error, color='m', label='LS-Cen')
            elif alg == 'lsci':
                plt.plot(time_arr, rmte_error, color='r', label='LS-CI')
            elif alg == 'lsbda':
                plt.plot(time_arr, rmte_error, color='g', label='LS-BDA')
            elif alg == 'gssci':
                plt.plot(time_arr, rmte_error, color='y', label='GS-SCI')
            elif alg == 'lssci':
                plt.plot(time_arr, rmte_error, color='y', label='LS-SCI')
            elif alg == 'gsci':
                plt.plot(time_arr, rmte_error, color='b', label='GS-CI')

        plt.title('RMTE of Dataset {}'.format(MRCLAM[-1]), fontsize=16)
        plt.xlabel('Time [s]', fontsize=14)
        plt.ylabel('Error [m]', fontsize=14)
        plt.legend()

        if save:
            save_path = '../results/' + 'type-' + str(comm_type) + '_' + 'range-' + str(comm_range) + '_' + 'rate-' + str(comm_rate) + '_' + 'probfail-' + str(comm_prob_fail) + '_' 'dt-' + str(dt) + '_' + 'offset-' + str(offset) + '_' + 'duration-' + str(duration) + '/' + MRCLAM + '/'
            fig_name = 'rmte'
            Path(save_path).mkdir(parents=True, exist_ok=True)
            plt.savefig(save_path + fig_name + '.pdf')

        # close all figures
        plt.close('all')

    if save:
        sys.stdout.write("\033[F")
        print('Analyzing and plotting results... {}\n'.format(u'\u2713'))
    else:
        sys.stdout.write("\033[F")
        print('Analyzing results... {}\n'.format(u'\u2713'))

def rot_mtx(theta):
    '''
    Counter-clockwise rotation matrix.

    Parameters
    ----------
    theta : float
        Angle in radians.

    Returns
    -------
    mtx : np.array
        2x2 counter-clockwise rotation matrix.

    '''

    mtx = np.array([[m.cos(theta), -m.sin(theta)], [m.sin(theta), m.cos(theta)]])

    return mtx


def exec_time(exec_start, exec_end):
    '''
    Converts total seconds into hours, minutes, and seconds.

    Parameters
    ----------
    exec_start : float
        Unix epoch execution start time.
    exec_end : float
        Unix epoch execution stop time.

    Returns
    -------
    hours : float
        Total hours.
    minutes : float
        Remaining minutes.
    seconds : float
        Remaining seconds.

    '''

    hours, rem = divmod(exec_end - exec_start, 3600)
    minutes, seconds = divmod(rem, 60)

    return hours, minutes, seconds

def plot_avg_rmse():
    
    lscen = [0.2577, 0.3259, 0.3728, 0.4010, 0.4058, 0.4302, 0.4376, 0.4341, 0.4325, 0.4326, 0.4353]
    lsci =  [0.2821, 0.2806, 0.3171, 0.3559, 0.393 , 0.4472, 0.5058, 0.5300, 0.5667, 0.5895, 0.6165]
    lsbda = [0.2702, 0.2591, 0.2702, 0.2804, 0.2892, 0.2915, 0.3210, 0.3225, 0.3441, 0.3537, 0.3792]
    gssci = [0.4467, 0.4965, 0.5016, 0.5267, 0.5077, 0.5108, 0.5288, 0.5230, 0.5346, 0.5352, 0.5376]
    gsci =  [0.3064, 0.3068, 0.3023, 0.3118, 0.3194, 0.3183, 0.3151, 0.3219, 0.3225, 0.3301, 0.3377]
    
    plt.figure(figsize=(10,4))
    
    prob_arr = np.arange(0.0, 1.1, 0.1)
    plt.plot(prob_arr, lscen, 'o-', color='m', label='LS-Cen')
    plt.plot(prob_arr, lsci, 'o-', color='r', label='LS-CI')
    plt.plot(prob_arr, lsbda, 'o-', color='g', label='LS-BDA')
    plt.plot(prob_arr, gssci, 'o-', color='y', label='GS-SCI')
    plt.plot(prob_arr, gsci, 'o-', color='b', label='GS-CI')

    plt.title('Average RMSE vs. Failure Rate', fontsize=16)
    plt.xlabel('Probability of Communication Failure', fontsize=14)
    plt.ylabel('Average RMSE [m]', fontsize=14)
    plt.xticks(np.arange(0.0,1.1,0.1))
    plt.legend()
    
    plt.savefig('average_rmse_vs_fail_rate.pdf')

    
    
