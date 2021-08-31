import numpy as np
import math
import os
import matplotlib.pyplot as plt


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


class Analyzer():
    def __init__(self, name, dataset_labels):
        self.name = name
        self.dataset_labels = dataset_labels
        self.num_robots = len(self.dataset_labels)

    def set_dataset_label(self, dataset_labels):
        self.dataset_labels = dataset_labels

    def plot_loc_err_and_trace(self, loc_err, trace, time_arr, operations_distr, recorder_name=None):

        [lm_measurement_num, relative_measurment_num, comm_num] = operations_distr
        fig = plt.figure()
        plt.suptitle('Correctness analysis')

        fig1 = fig.add_subplot(411)
        fig2 = fig.add_subplot(412)
        fig3 = fig.add_subplot(413)
        fig4 = fig.add_subplot(414)

        fig1.plot(time_arr, loc_err, label=recorder_name)
        fig2.plot(time_arr, trace, label=recorder_name)
        #fig3.bar(time_arr, lm_measurement_num, label = "landmark observation")
        fig3.bar(time_arr, relative_measurment_num,
                 label="relative observation")
        fig3.bar(time_arr, lm_measurement_num,
                 bottom=relative_measurment_num, label="landmark observation")

        fig4.bar(time_arr, comm_num,  label="communication")

        fig1.set_title('Estimation deviation error')
        fig1.set_xlabel('Time[s]')
        fig1.set_ylabel('RMS[m]')
        #fig1.set_ylim(0, 6)
        #fig1.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        fig2.set_title('Trace of state covariance')
        fig2.set_xlabel('Time [s]')
        fig2.set_ylabel('Sigma_s [m^2]')
        #fig2.set_ylim(0, 0.08)
        #ig2.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        fig3.set_title('Observation distrubution [# operations/sec]')
        fig3.set_xlabel('Time[s]')
        fig3.set_ylabel('Num of obser')
        #fig1.set_ylim(0, 6)
        fig3.legend(loc='best')
        #fig3.tick_params(labelsize=14)

        fig4.set_title('communication distrubution [# operations/sec]')
        fig4.set_xlabel('Time[s]')
        fig4.set_ylabel('Num of obser')
        #fig1.set_ylim(0, 6)
        fig4.legend(loc='best')

        fig.subplots_adjust(hspace=1.2)
        plt.show()

    def calculate_loc_err_and_trace_state_variance_per_run(self, data_recorder, plot_graphs=False, unit_time_interval=1, selected_labels=None):
        #recorded_dataline = [time, robot_label, est_x_pos, est_y_pos, trace_state_var, gt_x_pos, gt_y_pos, loc_err, update type]

        data = data_recorder.get_recorded_data()
        recorder_name = data_recorder.get_name()

        update_in_time_order = data_recorder.get_updata_type_in_time_order()
        data_in_time_order = data_recorder.get_data_in_time_order()

        time_index = 0
        time = data_in_time_order[time_index][0]
        interval_start_time = time
        finish_flag = False

        loc_err_per_run = []
        trace_per_run = []
        state_err_per_run = []
        lm_measurement_num = []
        relative_measurment_num = []
        comm_num = []

        time_arr = []
        f_est = open("./tmp/tmp_est_1.txt", "w")
        f_gt = open("./tmp/tmp_gt_1.txt", "w")

        while True:
            loc_err_per_time_iterval = 0
            trace_per_time_iterval = 0
            state_err_per_time_interval = 0
            num_dataline_per_time_iterval = 0
            lm_measurement_count = 0
            relative_measurement_count = 0
            comm_count = 0

            while interval_start_time <= time < interval_start_time+unit_time_interval:
                try:
                    data_in_time_order[time_index]
                except IndexError:
                    finish_flag = True
                    break
                time = data_in_time_order[time_index][0]

                if update_in_time_order[time_index] == 'landmark observation':
                    lm_measurement_count += 1
                if update_in_time_order[time_index] == 'relative observation':
                    relative_measurement_count += 1
                if update_in_time_order[time_index] == 'communication':
                    comm_count += 1

                if selected_labels == None or data_in_time_order[time_index][1] in selected_labels:
                    loc_err_per_time_iterval += data_in_time_order[time_index][7]
                    trace_per_time_iterval += data_in_time_order[time_index][4]

                    state_err_per_time_interval += np.linalg.norm(data_in_time_order[time_index][8] - data_in_time_order[time_index][9])/self.num_robots

                    num_dataline_per_time_iterval += 1

                    f_est.write(str(data_in_time_order[time_index][0]) + ' ' + str(
                        data_in_time_order[time_index][1])+''+update_in_time_order[time_index] + '\n')
                    np.savetxt(f_est, data_in_time_order[time_index][8])
                    f_gt.write(str(data_in_time_order[time_index][0]) + ' ' + str(
                        data_in_time_order[time_index][1]) + '\n')
                    np.savetxt(f_gt, data_in_time_order[time_index][9])
                
                time_index += 1

            if finish_flag:
                break

            if num_dataline_per_time_iterval != 0:
                    loc_err_per_run.append(
                        loc_err_per_time_iterval/num_dataline_per_time_iterval)
                    trace_per_run.append(
                        trace_per_time_iterval/num_dataline_per_time_iterval)
                    state_err_per_run.append(
                        state_err_per_time_interval/num_dataline_per_time_iterval)

            else:
                loc_err_per_run.append(0)
                trace_per_run.append(0)
                state_err_per_run.append(0)
        

            lm_measurement_num.append(lm_measurement_count)
            relative_measurment_num.append(relative_measurement_count)
            comm_num.append(comm_count)

            time_arr.append(
                (interval_start_time+unit_time_interval+interval_start_time)/2)
            interval_start_time = interval_start_time+unit_time_interval

        print(data_recorder.name, ': ')
        print('Avg location deviation errors per run: ',
              sum(loc_err_per_run)/len(loc_err_per_run))
        print('Avg trace of state variances per run: ',
              sum(trace_per_run)/len(trace_per_run))
        print('Avg state estimation errors per run: ', sum(
            state_err_per_run)/len(state_err_per_run))

        if plot_graphs:
            self.plot_loc_err_and_trace(state_err_per_run, trace_per_run, time_arr, operations_distr=[
                                        lm_measurement_num, relative_measurment_num, comm_num], recorder_name=recorder_name)

        f_est.close()
        f_gt.close()
        
        return loc_err_per_run, state_err_per_run, trace_per_run, time_arr

    def robot_location_at_unit_time_interval(self, data_recorder, unit_time_interval=0.2):
        #recorded_dataline = [time, robot_label, est_x_pos, est_y_pos, trace_state_var, gt_x_pos, gt_y_pos, loc_err]
        data = data_recorder.get_recorded_data()
        update_in_time_order = data_recorder.get_updata_type_in_time_order()
        data_in_time_order = data_recorder.get_data_in_time_order()

        time_index = 0
        time = data_in_time_order[time_index][0]
        interval_start_time = time
        finish_flag = False

        robot_loc_time_unit = {}
        for i, label in enumerate(self.dataset_labels):
            robot_loc_time_unit[label] = {
                'time': [], 'est_x': [], 'est_y': [], 'gt_x': [], 'gt_y': []}

        while True:
            robot_sum_loc_per_time_unit = {}
            measurement_update = False
            for i, label in enumerate(self.dataset_labels):
                robot_sum_loc_per_time_unit[label] = {'est_x': 0, 'est_y': 0, 'gt_x': 0, 'gt_y': 0, 'num': 0}

            while interval_start_time <= time < interval_start_time+unit_time_interval:

                try:
                    data_in_time_order[time_index]
                except IndexError:
                    finish_flag = True
                    break

                time = data_in_time_order[time_index][0]

                if update_in_time_order[time_index] == 'measurement':
                    measurement_update = True

                robot_label = data_in_time_order[time_index][1]

                #get sum for est_x, est_y, gt_x, gt_y and num for each robor per time interval
                robot_sum_loc_per_time_unit[robot_label]['est_x'] += data_in_time_order[time_index][2]
                robot_sum_loc_per_time_unit[robot_label]['est_y'] += data_in_time_order[time_index][3]
                robot_sum_loc_per_time_unit[robot_label]['gt_x'] += data_in_time_order[time_index][5]
                robot_sum_loc_per_time_unit[robot_label]['gt_y'] += data_in_time_order[time_index][6]
                robot_sum_loc_per_time_unit[robot_label]['num'] += 1

                time_index += 1

            if finish_flag:
                break

            for i, robot_label in enumerate(self.dataset_labels):
                for j in ['est_x', 'est_y', 'gt_x', 'gt_y']:
                    if robot_sum_loc_per_time_unit[robot_label]['num'] != 0:
                        robot_loc_time_unit[robot_label][j].append(robot_sum_loc_per_time_unit[robot_label][j]/robot_sum_loc_per_time_unit[robot_label]['num'])
                    else:
                        robot_loc_time_unit[robot_label][j].append(
                            robot_loc_time_unit[robot_label][j][-1])  # use the previous value

                robot_loc_time_unit[robot_label]['time'].append(
                    (interval_start_time+unit_time_interval+interval_start_time)/2)

            interval_start_time = interval_start_time+unit_time_interval

        return robot_loc_time_unit

    def algos_comparison_graph(self, arr_loc_err, arr_trace, only_trace, graph_name, show_img=False):
        fig = plt.figure()
        fig1 = fig.add_subplot(211)
        fig2 = fig.add_subplot(212)

        for loc_err in arr_loc_err:
            fig1.plot(loc_err[0], loc_err[1], label=loc_err[2])
            print(np.mean(loc_err[1]))

        for trace in arr_trace:
            if only_trace != None and trace[2] in only_trace:
                fig2.plot(trace[0], trace[1], '--', label=trace[2])
            else:
                fig2.plot(trace[0], trace[1], label=trace[2])

        #fig1.set_title('RMSE')
        fig1.set_xlabel('Time [s]')
        #fig1.set_ylabel('RMS [m]')
        fig1.set_ylabel('RMSE')
        fig1.set_ylim(0, 3)
        fig1.legend(loc=1)
        fig1.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        #fig1.tick_params(labelsize=18)

        #fig2.set_title('RMTE')
        fig2.set_xlabel('Time [s]')
        #fig2.set_ylabel('Sigma_s [m^2]')
        fig2.set_ylabel('RMTE')
        fig2.set_ylim(0, 2)
        fig2.legend(loc=1)
        fig2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        #fig2.tick_params(labelsize=18)

        plt.subplots_adjust(hspace=.6)
        plt.savefig(graph_name+'.pdf', bbox_inches='tight')

        if show_img:
            plt.show()

    def algos_comparison(self, arr_data_recorder, only_trace=None, graph_name='Algorithm Comparison', selected_labels=None):
        print("************Algorithm Comparison***************")
        arr_loc_err = []
        arr_trace = []
        for data_recorder in arr_data_recorder:
            loc_err_per_run, state_err_per_run, trace_per_run, time_stamps = self.calculate_loc_err_and_trace_state_variance_per_run(
                data_recorder, plot_graphs=False, selected_labels=selected_labels)
            if only_trace == None or data_recorder.name not in only_trace:
                # Need to change back to loc_err_per_run from state_err_per_run
                arr_loc_err.append([time_stamps, state_err_per_run, data_recorder.name])
            arr_trace.append([time_stamps, trace_per_run, data_recorder.name])

        print('Plotting Comparison Graphs')
        self.algos_comparison_graph(arr_loc_err, arr_trace, only_trace, graph_name)
        return arr_loc_err, arr_trace

    def trajectory_plot(self, sr_array, robot_labels=None):
        
        fig = plt.figure()
        k=0

        for dr, data_recorder in enumerate(sr_array):

            if robot_labels == None:
                robot_labels = self.dataset_labels

            plt.title('Multirobot Trajectory Comparison', fontsize=14)
            robot_loc_time_unit = self.robot_location_at_unit_time_interval(data_recorder)

            '''
            for i, robot_label in enumerate(robot_labels):
                
                x = robot_loc_time_unit[robot_label]['est_x']
                y = robot_loc_time_unit[robot_label]['est_y']

                xy = np.squeeze( np.array(([x],[y])) )

                theta = np.radians(-100)
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c)))

                new_xy = np.matmul(R, xy)

                new_xy[0,:] += 4.7
                new_xy[1,:] += -0.1

                new_xy[0,:] -= 1.6
                new_xy[1,:] += 1.5

                new_xy[0,:] *= 1.25
                new_xy[1,:] *= 1.25

                new_xy[0,:] += 1.6
                new_xy[1,:] -= 1.5
                

                robot_loc_time_unit[robot_label]['est_x'] = new_xy[0,:]
                robot_loc_time_unit[robot_label]['est_y'] = new_xy[1,:]
            '''

            for i, robot_label in enumerate(robot_labels):

                    plt.plot(robot_loc_time_unit[i+1]['gt_x'], robot_loc_time_unit[i+1]['gt_y'],  color='k', label='Robot %d Ground Truth' % (i+1))
                    clr = 'C'+str(k)
                    plt.plot(robot_loc_time_unit[i+1]['est_x'], robot_loc_time_unit[i+1]['est_y'], '--',  color=clr, label='Robot %d estimation' % (i+1))

                    k+=1

        plt.xlabel('X Direction [m]', fontsize=12)
        plt.ylabel('Y Direction [m]', fontsize=12)
        #plt.xlim(-1.25, 1.0)
        #plt.ylim(-1.0, 1.25)
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.savefig('trajectory.pdf', bbox_inches='tight')
        plt.show()
