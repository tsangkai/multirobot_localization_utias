import matplotlib.pyplot as plt
from math import sqrt
import requests.request_response as request_response
import numpy as np
import os
import sys

#sys.path.insert(0, '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/functions/requests')
sys.path.append(os.path.join(os.path.dirname(__file__), "../requests"))


class StatesRecorder():
    """docstring for ClassName"""

    def __init__(self, name, dataset_labels, state_var_only=False):
        self.name = name
        self.dataset_labels = dataset_labels
        self.state_var_only = state_var_only
        self.recorded_data = {}
        self.data_in_time_order = []
        self.updata_type_in_time_order = []
        self.num_robots = len(self.dataset_labels)
        self.loc_err_arr = {}
        self.trace_sigma_s_arr = {}
        self.updata_type_arr = {}
        self.gt_states = np.matrix(np.zeros((2*self.num_robots, 1)))
        for i, label in enumerate(self.dataset_labels):
            self.recorded_data[label] = []
            self.loc_err_arr[label] = []
            self.trace_sigma_s_arr[label] = []
            self.updata_type_arr[label] = []

    def set_starting_state(self, stating_states):
        self.gt_states = np.matrix(np.zeros((2*self.num_robots, 1)))
        for i, label in enumerate(self.dataset_labels):
            x_pos = stating_states[label][1]
            y_pos = stating_states[label][2]
            self.gt_states[2*i, 0] = x_pos
            self.gt_states[2*i+1, 0] = y_pos

        for i, label in enumerate(self.dataset_labels):
            self.start_time = stating_states[label][0]
            time = 0
            x_pos = stating_states[label][1]
            y_pos = stating_states[label][2]
            initial_trace_state_var = 0.002 * label
            loc_err = 0
            recorded_dataline = [time, label, x_pos, y_pos, initial_trace_state_var,
                                 x_pos, y_pos, loc_err, self.gt_states.copy(), self.gt_states.copy()]

            self.data_in_time_order.append(recorded_dataline)
            self.recorded_data[label].append(recorded_dataline)
            self.loc_err_arr[label].append(0)
            self.trace_sigma_s_arr[label].append(initial_trace_state_var)
            self.updata_type_arr[label].append('ini')
            self.updata_type_in_time_order.append('ini')

    def get_name(self):
        return self.name

    def get_dataset_labels(self):
        return self.dataset_labels

    def record_state(self, req, robot_state):
        message = req.get_message()
        robot_idx = message['robot_index']
        time = message['time']-self.start_time
        gt = message['groundtruth']
        est_x_pos = float(robot_state['x_pos'])
        est_y_pos = float(robot_state['y_pos'])
        est_states = robot_state['est_states']
        trace_state_var = robot_state['trace of state variance']
        updata_type = robot_state['update_type']
        gt_x_pos = gt['x_pos']
        gt_y_pos = gt['y_pos']

        self.gt_states[2*robot_idx, 0] = gt_x_pos
        self.gt_states[2*robot_idx+1, 0] = gt_y_pos

        robot_label = self.dataset_labels[robot_idx]

        if self.state_var_only:
            loc_err = 0
            recorded_dataline = [time, robot_label, gt_x_pos, gt_y_pos, trace_state_var,
                                 gt_x_pos, gt_y_pos, loc_err, self.gt_states.copy(), self.gt_states.copy()]
        else:
            loc_err = sqrt((est_x_pos-gt_x_pos)*(est_x_pos-gt_x_pos) + (est_y_pos-gt_y_pos)*(est_y_pos-gt_y_pos))
            recorded_dataline = [time, robot_label, est_x_pos, est_y_pos, trace_state_var,
                                 gt_x_pos, gt_y_pos, loc_err, est_states.copy(), self.gt_states.copy()]

        #print(est_states)
        #print(self.gt_states)
        #warning (optional)
        '''
        if(trace_state_var<0):
            print('TIME: ', time+self.start_time)
            print(updata_type)
            print('neg trace: ', recorded_dataline)

        if(loc_err >= 1):
            print(updata_type)
            print('>1 m loc err: ',recorded_dataline)
            print(req.get_message())
        '''

        #print(self.gt_states)

        self.data_in_time_order.append(recorded_dataline)
        #print(self.data_in_time_order[-1])
        self.updata_type_in_time_order.append(updata_type)
        self.recorded_data[robot_label].append(recorded_dataline)
        self.loc_err_arr[robot_label].append(loc_err)
        self.trace_sigma_s_arr[robot_label].append(trace_state_var)
        self.updata_type_arr[robot_label].append(updata_type)

    def get_data_in_time_order(self):
        return self.data_in_time_order

    def get_updata_type_in_time_order(self):
        return self.updata_type_in_time_order

    def get_recorded_data(self):
        return self.recorded_data

    def get_time_arr(self, robot_id):
        time_arr = np.array(self.recorded_data[robot_id])[:, 0]
        return time_arr

    def get_loc_err_arr(self):
        return self.loc_err_arr

    def get_trace_sigma_s_arr(self):
        return self.trace_sigma_s_arr

    def get_update_type_arr(self):
        return self.updata_type_arr

    def simple_plot(self, plot_title=''):
        fig = plt.figure()
        plt.suptitle(plot_title + ' Correctness analysis')
        fig1 = fig.add_subplot(211)
        fig2 = fig.add_subplot(212)
        loc_err_arr = self.get_loc_err_arr()
        trace_sigma_s_arr = self.get_trace_sigma_s_arr()
        for i, label in enumerate(self.dataset_labels):
            time_arr = self.get_time_arr(label)
            fig1.plot(time_arr, loc_err_arr[label], label='Robot %d' % label)
            fig2.plot(
                time_arr, trace_sigma_s_arr[label], label='Robot %d' % label)
            print('Robot', label, 'loc err: ', sum(
                loc_err_arr[label])/len(loc_err_arr[label]))
            print('Robot', label, 'trace Sigma_s: ', sum(
                trace_sigma_s_arr[label])/len(trace_sigma_s_arr[label]))
        fig1.set_title('Estimation deviation error')
        fig1.set_xlabel('Time[s]')
        fig1.set_ylabel('RMS[m]')
        #fig1.set_ylim(0, 6)
        fig1.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        fig2.set_title('Trace of state variance')
        fig2.set_xlabel('Time [s]')
        fig2.set_ylabel('Sigma_s [m^2]')
        #fig2.set_ylim(0, 0.08)
        fig2.legend(loc='center left', bbox_to_anchor=(1, 0.5))

        fig.subplots_adjust(hspace=0.8)
        plt.show()

        return True
