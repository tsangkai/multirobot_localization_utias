import numpy as np
import os
import matplotlib.pyplot as plt
import math
import sys


def find_nearest_time_idx(array, start_idx, value):
    #array[i][0] is the time
    i = start_idx
    if array[i]['time'] < value:
        while array[i]['time'] < value:
            i = i+1
        if array[i]['time']-value < value-array[i-1]['time']:
            return i
        else:
            return i-1
    else:
        return i


def plots_MSE_and_trace(dataset_labels, analysis_result_name, dataset_name, algo_name):

    fig = plt.figure()
    plt.suptitle('Correctness analysis  Algo: ' +
                 algo_name + ' Dataset: ' + dataset_name)
    fig1 = fig.add_subplot(211)
    fig2 = fig.add_subplot(212)
    for i, label in enumerate(dataset_labels):
        #initialization for all the lists for plotting
        dev_l_t = []
        dev_l_RMSE = []
        dev_l_trace = []
        file = "/Results_files/analysis_results/" + \
            analysis_result_name+str(label)+".txt"
        path = os.getcwd()+file
        with open(path, 'r') as analysis_file:
            s_dev = analysis_file.readline()
            starting_time = float(s_dev.split()[0])
            print starting_time
            while (s_dev):
                dev_l_t.append(float(s_dev.split()[0]) - starting_time)
                dev_l_RMSE.append(float(s_dev.split()[1]))
                dev_l_trace.append(float(s_dev.split()[2]))
                s_dev = analysis_file.readline()
        fig1.plot(dev_l_t, dev_l_RMSE, label='Robot %d' % label)
        fig2.plot(dev_l_t, dev_l_trace, label='Robot %d' % label)

    fig1.set_title('Root mean square error')
    fig1.set_xlabel('Time[s]')
    fig1.set_ylabel('RMS[m]')
    #fig1.set_ylim(0, 6)
    fig1.legend(loc=2)

    fig2.set_title('Trace of state variance')
    fig2.set_xlabel('Time [s]')
    fig2.set_ylabel('Sigma_s [m^2]')
    #fig2.set_ylim(0, 0.08)
    fig2.legend(loc=2)

    fig.subplots_adjust(hspace=0.8)
    plt.show()

    return 1


def plots_mean_MSE_and_trace(mean_time, mean_err, mean_trace, dataset_name, algo_name):

    fig = plt.figure()
    plt.suptitle('Correctness analysis  Algo: ' +
                 algo_name + ' Dataset: ' + dataset_name)
    fig1 = fig.add_subplot(211)
    fig2 = fig.add_subplot(212)

    fig1.plot(mean_time, mean_err)
    fig2.plot(mean_time, mean_trace)

    fig1.set_title('Avg. Root mean square error')
    fig1.set_xlabel('Time[s]')
    fig1.set_ylabel('RMS[m]')
    #fig1.set_ylim(0, 6)

    fig2.set_title('Avg. Trace of state variance')
    fig2.set_xlabel('Time [s]')
    fig2.set_ylabel('Sigma_s [m^2]')
    #fig2.set_ylim(0, 0.08)

    fig.subplots_adjust(hspace=0.8)
    plt.show()

    return 1


def plots_mean_MSE_and_trace_for_multiple_algos(figure_name, algos_set, all_mean_time, all_mean_err, all_mean_STD, dataset_name, comm_fail_rate):

    fig = plt.figure()
    plt.suptitle('Correctness analysis' + ' Dataset: ' +
                 dataset_name + ' comm failure rate: ' + str(comm_fail_rate))
    fig1 = fig.add_subplot(211)
    fig2 = fig.add_subplot(212)

    for i, algo_name in enumerate(algos_set):
        print algo_name
        if algo_name == 'GS-CI':
            fig1.plot(all_mean_time[i], all_mean_err[i],
                      label=algo_name, linewidth=3.0)
            fig2.plot(all_mean_time[i], all_mean_STD[i],
                      label=algo_name, linewidth=3.0)
        else:
            fig1.plot(all_mean_time[i], all_mean_err[i], label=algo_name)
            fig2.plot(all_mean_time[i], all_mean_STD[i], label=algo_name)

    fig1.set_title('Actual location error')
    fig1.set_xlabel('Time [s]')
    fig1.set_ylabel('RMS [m]')
    #fig1.set_ylim(0, 3)
    fig1.legend(loc=2)
    #plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=1)

    fig2.set_title('Trace of state deviation')
    fig2.set_xlabel('Time [s]')
    fig2.set_ylabel('STD [m]')
    #fig2.set_ylim(0, 1)
    fig2.legend(loc=2)

    fig.subplots_adjust(hspace=0.8)
    figure_name = figure_name + '.png'
    print figure_name
    plt.savefig(figure_name)

    plt.show()

    return 1


def plots_mean_MSE_and_trace_for_multiple_algos_w_bound(figure_name, algos_set, all_mean_time, all_mean_err, all_mean_STD, all_mean_bound, dataset_name, comm_fail_rate):

    fig = plt.figure()
    plt.suptitle('Correctness analysis' + ' Dataset: ' +
                 dataset_name + ' comm failure rate: ' + str(comm_fail_rate))
    fig1 = fig.add_subplot(211)
    fig2 = fig.add_subplot(212)

    for i, algo_name in enumerate(algos_set):
        print algo_name
        if algo_name == 'GS-CI':
            fig1.plot(all_mean_time[i], all_mean_err[i],
                      label=algo_name, linewidth=3.0)
            fig2.plot(all_mean_time[i], all_mean_STD[i],
                      label=algo_name, linewidth=3.0)
            fig2.plot(all_mean_time[i], all_mean_bound[i],
                      label='bound', linewidth=3.0)
        else:
            fig1.plot(all_mean_time[i], all_mean_err[i], label=algo_name)
            fig2.plot(all_mean_time[i], all_mean_STD[i], label=algo_name)

    fig1.set_title('Actual location error')
    fig1.set_xlabel('Time [s]')
    fig1.set_ylabel('RMS [m]')
    #fig1.set_ylim(0, 3)
    fig1.legend(loc=2)
    #plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=1)

    fig2.set_title('Trace of state deviation')
    fig2.set_xlabel('Time [s]')
    fig2.set_ylabel('STD [m]')
    #fig2.set_ylim(0, 1)
    fig2.legend(loc=2)

    fig.subplots_adjust(hspace=0.8)
    figure_name = figure_name + '.png'
    print figure_name
    plt.savefig(figure_name)

    plt.show()

    return 1


def plot_err_x(starting_time, dataset_labels, analysis_result_name, dataset_name):
    plt.figure(1)

    for i, label in enumerate(dataset_labels):
        list_t = []
        list_err_x = []
        file = "/Results_files/analysis_results/" + \
            analysis_result_name+str(label)+".txt"
        path = os.getcwd()+file
        with open(path, 'r') as analysis_file:
            s_dev = analysis_file.readline()
            while (s_dev):
                list_t.append(float(s_dev.split()[0])-starting_time)
                list_err_x.append(float(s_dev.split()[3]))
                s_dev = analysis_file.readline()
        plt.plot(list_t, list_err_x, label='Robot %d' % label)

    #plt.ylim(-2, 2)
    #plt.xlim(0, 600)

    plt.legend(loc=1)
    plt.title('Robot x-pos error    Dataset: ' + dataset_name)
    plt.xlabel('Time[s]')
    plt.ylabel('Robot x-pos error[m]')
    plt.show()
    pass


def plot_RMSE_and_trace_withrespectto_noise(list2D_avg_RMSE, list2D_avg_trace, noise_range):
    list_process_noise = np.arange(noise_range[0], noise_range[1], 0.2)
    print list_process_noise
    plt.figure(1)
    plt.suptitle(
        'Error analysis for single EKF for all robots with respect to measurement noise')

    fig1 = plt.subplot(211)
    for i in range(5):
        fig1.plot(list_process_noise,
                  list2D_avg_RMSE[i], label='Robot %d' % (i+1))

    plt.title('Root mean square error(RMSE)')
    plt.xlabel('Measurement noise')
    plt.ylabel('RMSE')
    #plt.ylim(1, 2)

    plt.legend(loc=1)

    fig2 = plt.subplot(212)
    for i in range(5):
        fig2.plot(list_process_noise,
                  list2D_avg_trace[i], label='Robot %d' % (i+1))

    plt.title('Trace of state variance')
    plt.xlabel('Measurement noise')
    plt.ylabel('Trace')
    plt.legend(loc=2)
    #plt.ylim(0, 2)

    plt.subplots_adjust(hspace=0.8)
    plt.show()

    return 1


def single_plot_w_bond(start_time, est_result_array, groundtruth_data, robot_index, algo_name):
    time_arr = []
    err_arr = []
    sigma_s_arr = []
    bound_arr = []
    i = robot_index
    g_idx = 0
    j = 0
    while j < len(est_result_array[i]):
        est_t = est_result_array[i][j]['time']
        g_idx = find_nearest_time_idx(groundtruth_data[i], g_idx, est_t)

        if abs(groundtruth_data[i][g_idx]['time']-est_result_array[i][j]['time']) > 0.5:
            sys.exit('groundtruth time not match!')

        gt_x_pos = groundtruth_data[i][g_idx]['x_pos']
        gt_y_pos = groundtruth_data[i][g_idx]['y_pos']
        est_x_pos = est_result_array[i][j]['x_pos']
        est_y_pos = est_result_array[i][j]['y_pos']
        sigma_s_trace = est_result_array[i][j]['sigma_s_trace']
        bound = est_result_array[i][j]['bound']

        err_sq = sum([(est_x_pos-gt_x_pos)*(est_x_pos-gt_x_pos),
                      (est_y_pos-gt_y_pos)*(est_y_pos-gt_y_pos)])
        err_sqrt = math.sqrt(err_sq)

        est_t = est_t - start_time
        #print est_t
        #print err_sq

        time_arr.append(est_t)
        err_arr.append(err_sq)
        sigma_s_arr.append(sigma_s_trace)
        bound_arr.append(bound)

        #print sigma_s_arr

        j = j+1

    print est_result_array[0][0]
    print est_result_array[0][-1]

    fig = plt.figure()
    plt.suptitle('GS-CI analysis w/ e = 0.5')
    fig1 = fig.add_subplot(211)
    fig2 = fig.add_subplot(212)

    fig1.plot(time_arr, err_arr, label=algo_name, linewidth=3.0)
    fig2.plot(time_arr, sigma_s_arr, label=algo_name, linewidth=3.0)
    #fig2.plot(time_arr, bound_arr, label = 'bound', linewidth=3.0)

    fig1.set_title('Actual location error')
    fig1.set_xlabel('Time [s]')
    fig1.set_ylabel('RMS [m]')
    #fig1.set_ylim(0, 1)
    fig1.legend(loc=2)
    #plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=1)

    fig2.set_title('Trace of state deviation')
    fig2.set_xlabel('Time [s]')
    fig2.set_ylabel('STD [m]')
    #fig2.set_ylim(0, 0.2)
    fig2.legend(loc=2)

    plt.show()

    return 1
