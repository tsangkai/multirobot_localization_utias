import numpy as np
import math
import os


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


class DataAnalyzer():
    def __init__(self, name):
        self.name = name

    def estimation_time_analysis_single(est_data):
        #calcaulate the root-mean-square deviation (RMSD) between estimation and ground truth and the trace of state variance at each time step.

        robot_num = str(robot_num)
        file = "/Datasets/"+dataset_name+"/Robot"+robot_num+"_Groundtruth.dat"
        path = os.getcwd()+file
        groundtruth_file = open(path, 'r+')

        file = "/Results_files/estimation_results/"+estimation_file_name+".txt"
        path = os.getcwd()+file
        est_result_file = open(path, "r+")

        file = "/Results_files/analysis_results/"+analysis_result_name+".txt"
        path = os.getcwd()+file
        analysis_file = open(path, "w")

        s_ground = groundtruth_file.readline()
        s_ground = groundtruth_file.readline()
        s_ground = groundtruth_file.readline()
        s_ground = groundtruth_file.readline()

        s_ground = groundtruth_file.readline()
        t_ground = float(s_ground.split()[0])

        s_est = est_result_file.readline()
        t_est = float(s_est.split()[0])

        while(s_est):
            s = [float(s_est.split()[1]), float(
                s_est.split()[2]), float(s_est.split()[3])]

            while t_ground <= t_est:
                s_ground = groundtruth_file.readline()
                t_ground = float(s_ground.split()[0])
                if t_ground >= t_est:
                    s_g = [float(s_ground.split()[1]),
                           float(s_ground.split()[2])]
                    break

            err_sq = [(s_g[0]-s[0])*(s_g[0]-s[0]), (s_g[1]-s[1])*(s_g[1]-s[1])]
            #err_sq[2] = min((s_g[2]-s[2])*(s_g[2]-s[2]), (s_g[2]+2*math.pi-s[2])*(s_g[2]+2*math.pi-s[2]), (s_g[2]-2*math.pi-s[2])*(s_g[2]-2*math.pi-s[2]))
            err_sqrt = math.sqrt(sum(err_sq))
            err_x = s_g[1]-s[1]
            analysis_file.write(str(t_est) + '\t' + str(err_sqrt) + '\t' + str(s_est.split(
            )[3]) + '\t' + str(err_x) + '\n')  # s_est.split()[3] is the trace of sigma_S

            s_est = est_result_file.readline()
            if(s_est):
                t_est = float(s_est.split()[0])

        groundtruth_file.close()
        est_result_file.close()
        analysis_file.close()

        return 1

    def estimation_time_analysis_multirobot(dataset_name, general_estimation_file_name, general_analysis_result_name, robot_list):
        for i, robot in enumerate(robot_list):
            robot_num = robot.get_label()
            estimation_file_name = general_estimation_file_name + '%d' % robot_num
            analysis_result_name = general_analysis_result_name + '%d' % robot_num
            estimation_time_analysis_single(
                dataset_name, robot_num, estimation_file_name, analysis_result_name)

    def avg_MSE_and_trace(analysis_result_name):

        #initialization for all the lists for plotting
        dev_l_t = []
        dev_l_RMSE = []
        dev_l_trace = []

        file = "/Results_files/analysis_results/"+analysis_result_name+".txt"
        path = os.getcwd()+file
        with open(path, 'r') as analysis_file:
            s_dev = analysis_file.readline()
            while (s_dev):
                dev_l_t.append(float(s_dev.split()[0]))
                dev_l_RMSE.append(float(s_dev.split()[1]))
                dev_l_trace.append(float(s_dev.split()[2]))
                s_dev = analysis_file.readline()

        avg_RMSE = float(sum(dev_l_RMSE)) / max(len(dev_l_RMSE), 1)
        avg_trace = float(sum(dev_l_trace)) / max(len(dev_l_trace), 1)
        print '******Analysis Results******'
        print 'Avg. RMSE: ' + str(avg_RMSE)
        print 'Avg. trace of state covariance: ' + str(avg_trace)
        print '*************\n'

        return [avg_RMSE, avg_trace]

    def average_correctness_analysis_per_run(dataset_name, dataset_labels, groundtruth_data, est_result_array):
        # #calcaulate the avg. root-mean-square deviation (RMSD) between estimation and groundtruth, and the avg. trace of state variance in each time interval(1s) for all robots.

        num_robot = len(dataset_labels)
        analysis_result_array = [[] for i in range(num_robot)]

        time_arr = []
        avg_RMSE_arr = []
        avg_trace_arr = []

        analysis_file_arr = []

        # analysis for each robot
        for i, robot_label in enumerate(dataset_labels):
            j = 0
            g_idx = 0
            while j < len(est_result_array[i]):
                est_t = est_result_array[i][j]['time']
                g_idx = find_nearest_time_idx(
                    groundtruth_data[i], g_idx, est_t)

                gt_x_pos = groundtruth_data[i][g_idx]['x_pos']
                gt_y_pos = groundtruth_data[i][g_idx]['y_pos']
                est_x_pos = est_result_array[i][j]['x_pos']
                est_y_pos = est_result_array[i][j]['y_pos']
                sigma_s_trace = est_result_array[i][j]['sigma_s_trace']

                err_sq = [(est_x_pos-gt_x_pos)*(est_x_pos-gt_x_pos),
                          (est_y_pos-gt_y_pos)*(est_y_pos-gt_y_pos)]
                err_sqrt = math.sqrt(sum(err_sq))

                analysis_result = {
                    'time': est_t, 'err_sqrt': err_sqrt, 'sigma_s_trace': sigma_s_trace}
                analysis_result_array[i].append(analysis_result)

                #analysis_file_arr[i].write(str(est_t)+ '\t' + str(err_sqrt) + '\t' + str(sigma_s_trace) +'\n')

                j = j+1

        start_time = est_result_array[0][0]['time'] + 1/2

        mean_err = []
        mean_time = []
        mean_STD = []

        loc_dic = {}
        for i, robot_label in enumerate(dataset_labels):
            start_loc = 0
            loc_dic[i] = start_loc

        avg_trace = np.zeros(num_robot)
        avg_err = np.zeros(num_robot)

        while(len(loc_dic) != 0):
            for i, robot_label in enumerate(dataset_labels):
                try:
                    loc = loc_dic[i]
                except KeyError:
                    continue
                try:
                    time_interval_start = analysis_result_array[i][loc]['time']
                except IndexError:
                    del loc_dic[i]
                    continue

                time_interval = 0.5  # in [s] for each time interval
                num_analyized_result = 0
                sum_trace = 0
                sum_err = 0
                while analysis_result_array[i][loc]['time'] < time_interval_start + time_interval:
                    sum_trace += analysis_result_array[i][loc]['sigma_s_trace']
                    sum_err += analysis_result_array[i][loc]['err_sqrt']
                    num_analyized_result += 1

                    loc += 1

                    try:
                        analysis_result_array[i][loc]
                    except IndexError:
                        del loc_dic[i]
                        break

                loc_dic[i] = loc
                avg_trace[i] = sum_trace/num_analyized_result
                avg_err[i] = sum_err/num_analyized_result

            all_robot_avg_err = sum(avg_err)/num_robot
            all_robot_avg_trace = sum(avg_trace)/num_robot
            avg_STD = math.sqrt(all_robot_avg_trace)

            mean_time.append(time_interval_start+1/2 - start_time)
            mean_err.append(all_robot_avg_err)
            mean_STD.append(avg_STD)

        return [mean_time, mean_err, mean_STD]

    def recording_data(algos_set, all_mean_time, all_mean_err, all_mean_STD, dataset_name):

        for i, algo_name in enumerate(algos_set):
            algo_analysis_file_name = dataset_name + '_algo_'+algo_name
            file = "/Results_files/analysis_results/"+algo_analysis_file_name+".txt"
            path = os.getcwd()+file
            algo_analysis_file = open(path, "w")
            algo_analysis_file.write(
                '#time[s] \t\t actual error[m] \t state STD[m] \n')

            for j in range(len(all_mean_time[i])):
                algo_analysis_file.write(str(all_mean_time[i][j]) + '\t' + str(
                    all_mean_err[i][j]) + '\t' + str(all_mean_STD[i][j]) + '\n')

            algo_analysis_file.close()

        return 1

    def average_correctness_analysis_per_run_w_bound(dataset_name, dataset_labels, groundtruth_data, est_result_array):
        # #calcaulate the avg. root-mean-square deviation (RMSD) between estimation and groundtruth, and the avg. trace of state variance in each time interval(1s) for all robots.

        num_robot = len(dataset_labels)
        analysis_result_array = [[] for i in range(num_robot)]

        time_arr = []
        avg_RMSE_arr = []
        avg_trace_arr = []

        analysis_file_arr = []

        # analysis for each robot
        for i, robot_label in enumerate(dataset_labels):
            j = 0
            g_idx = 0
            while j < len(est_result_array[i]):
                est_t = est_result_array[i][j]['time']
                g_idx = find_nearest_time_idx(
                    groundtruth_data[i], g_idx, est_t)

                gt_x_pos = groundtruth_data[i][g_idx]['x_pos']
                gt_y_pos = groundtruth_data[i][g_idx]['y_pos']
                est_x_pos = est_result_array[i][j]['x_pos']
                est_y_pos = est_result_array[i][j]['y_pos']
                sigma_s_trace = est_result_array[i][j]['sigma_s_trace']
                bound = est_result_array[i][j]['bound']

                err_sq = [(est_x_pos-gt_x_pos)*(est_x_pos-gt_x_pos),
                          (est_y_pos-gt_y_pos)*(est_y_pos-gt_y_pos)]
                err_sqrt = math.sqrt(sum(err_sq))

                analysis_result = {'time': est_t, 'err_sqrt': err_sqrt,
                                   'sigma_s_trace': sigma_s_trace, 'bound': bound}
                analysis_result_array[i].append(analysis_result)
                #analysis_file_arr[i].write(str(est_t)+ '\t' + str(err_sqrt) + '\t' + str(sigma_s_trace) +'\n')

                j = j+1

        start_time = est_result_array[0][0]['time'] + 1/2

        mean_err = []
        mean_time = []
        mean_STD = []
        mean_bound_sq = []

        loc_dic = {}
        for i, robot_label in enumerate(dataset_labels):
            start_loc = 0
            loc_dic[i] = start_loc

        avg_trace = np.array(np.zeros(num_robot))
        avg_bound = np.array(np.zeros(num_robot))
        avg_err = np.array(np.zeros(num_robot))

        while(len(loc_dic) != 0):
            for i, robot_label in enumerate(dataset_labels):
                try:
                    loc = loc_dic[i]
                except KeyError:
                    continue
                try:
                    time_interval_start = analysis_result_array[i][loc]['time']
                except IndexError:
                    del loc_dic[i]
                    continue

                time_interval = 0.5  # in [s] for each time interval
                num_analyized_result = 0
                sum_trace = 0
                sum_bound = 0
                sum_err = 0
                while analysis_result_array[i][loc]['time'] < time_interval_start + time_interval:
                    sum_trace += analysis_result_array[i][loc]['sigma_s_trace']
                    sum_bound += analysis_result_array[i][loc]['bound']
                    sum_err += analysis_result_array[i][loc]['err_sqrt']
                    num_analyized_result += 1

                    loc += 1
                    try:
                        analysis_result_array[i][loc]
                    except IndexError:
                        del loc_dic[i]
                        break

                loc_dic[i] = loc
                avg_trace[i] = sum_trace/num_analyized_result
                avg_bound[i] = sum_bound/num_analyized_result
                avg_err[i] = sum_err/num_analyized_result

            all_robot_avg_err = sum(avg_err)/num_robot
            all_robot_avg_trace = sum(avg_trace)/num_robot
            all_robot_avg_bound = sum(avg_bound)/num_robot
            avg_STD = math.sqrt(all_robot_avg_trace)
            avg_bound_sq = math.sqrt(all_robot_avg_bound)

            mean_time.append(time_interval_start+1/2 - start_time)
            mean_err.append(all_robot_avg_err)
            mean_STD.append(avg_STD)
            mean_bound_sq.append(avg_bound_sq)

        return [mean_time, mean_err, mean_STD, mean_bound_sq]
