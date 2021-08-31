# -*- coding: utf-8 -*-

import numpy as np
from numpy import random
import sys
from math import atan2, sqrt, pi
import random
from math import cos
from math import sin
import math
import os.path
'''
sys.path.insert(0, '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/functions/requests')
import request_response
'''

def find_nearest_time_idx(l, value):
    if len(l) != 0:
        array = np.asarray(l)
        idx = (np.abs(array-value)).argmin()
        return idx
    else:
        return None


def find_next_time_idx(array, start_idx, value):
    i = start_idx
    try:
        array[i]
    except IndexError:
        i = -1
        return i
    while array[i] < value:
        i = i+1
        try:
            array[i]
        except IndexError:
            i = -1
            break
    return i

def linear_interpolation(end0, end1, x):
    [x0, y0] = end0
    [x1, y1] = end1
    if x1 == x0:
        y = y0
    else:
        y = y0+(x-x0)*(y1-y0)/(x1-x0)
    return y

class RW_Dataset_Manager:

    def __init__(self, dataset_name):
        self.name = dataset_name

    def create_landmark_map(self):
    	### Build landmark map  ###
        self.landmark_map = {}
        path=self.dataset_path + "Landmark_Groundtruth.dat"
        landmark_file=open(path,'r+');
        s = landmark_file.readline()
        while(s):
            if(s[0]!='#'):
               landmark_location = [float(s.split( )[1]), float(s.split( )[2])]
               self.landmark_map.update({int(s.split( )[0]): landmark_location})
            s = landmark_file.readline()
        landmark_file.close()
        #print "lm  ", self.landmark_map, "  lm"
        return self.landmark_map


    def load_datasets(self, dataset_path, robot_lables, duration, adding_actifical_dataline = True, delay_start = 5):
        print ('******** Initialization Started ********')
        print ('add synthetic data: ', adding_actifical_dataline)
        self.dataset_path = dataset_path
        print("Absolute datapath: ")
        print(self.dataset_path)
        self.robot_lables = robot_lables
        self.adding_actifical_dataline = adding_actifical_dataline
        self.create_landmark_map()

        self.num_robots = len(self.robot_lables)
        self.measurement_data = [[] for i in range(self.num_robots)]
        self.odometry_data = [[] for i in range(self.num_robots)]
        self.gt_data_odo = [[] for i in range(self.num_robots)]
        self.gt_data_meas = [[] for i in range(self.num_robots)]
        self.groundtruth_data = [[] for i in range(self.num_robots)]

        #self.groundtruth_time= [[] for i in range(self.num_robots)]
        self.time_arr = {'odometry': [[] for i in range(self.num_robots)], 'measurement': [[] for i in range(self.num_robots)], 'groundtruth': [[] for i in range(self.num_robots)]}

        #initialization for MRCLAMDatasets: put files into dictionaries:
        self.duration = duration # some more time to avoid index error
        self.start_time_arr = []
        #finding the starting time:
        for i, label in enumerate(self.robot_lables):
            robot_num = str(label)
            groundtruth_path = self.dataset_path+"Robot"+robot_num+"_Groundtruth.dat"
            with open(groundtruth_path,'r+') as groundtruth_file:
                for line in groundtruth_file:
                    if str(line.split()[0]) != '#':
                        time = float(line.split()[0])
                        self.start_time_arr.append(time)
                        break
        self.start_time = max(self.start_time_arr) + delay_start
        self.end_time = self.start_time + self.duration
        print('Staring time: ', self.start_time)
        #finding starting states:
        self.starting_states = {}
        for i, label in enumerate(self.robot_lables):
            robot_num = str(label)
            groundtruth_path = self.dataset_path+"Robot"+robot_num+"_Groundtruth.dat"
            with open(groundtruth_path,'r+') as groundtruth_file:
                for line in groundtruth_file:
                    if line[0] != '#' and float(line.split()[0]) >= self.start_time:
                        time = round(float(line.split()[0]), 3)
                        x_pos = float(line.split()[1])
                        y_pos = float(line.split()[2])
                        orientation = float(line.split()[3])
                        self.starting_states[label] = [time, x_pos, y_pos, orientation]
                        break
        print('Staring states: ')
        print(self.starting_states)


        for i, label in enumerate(self.robot_lables):
            robot_num = str(label)

            groundtruth_path = self.dataset_path+"Robot"+robot_num+"_Groundtruth.dat"
            with open(groundtruth_path,'r+') as groundtruth_file:
                for line in groundtruth_file:
                    if line[0] != '#' and (self.end_time >= float(line.split()[0]) >= self.start_time):
                        time = round(float(line.split()[0]), 3)
                        x_pos = float(line.split()[1])
                        y_pos = float(line.split()[2])
                        orientation = float(line.split()[3])
                        groundtruth_info = {'time':time, 'x_pos':x_pos, 'y_pos':y_pos, 'orientation':orientation}
                        self.groundtruth_data[i].append(groundtruth_info)
                        self.time_arr['groundtruth'][i].append(time)


            meas_path = self.dataset_path+"Robot"+robot_num+"_Measurement_x.dat"
            with open(meas_path,'r+') as measure_file:
                for line in measure_file:
                    if line[0] != '#' and (self.end_time>= float(line.split()[0]) >= self.start_time):
                        time = round(float(line.split()[0]), 3)
                        subject_ID = int(line.split()[1])
                        measurment_range = float(line.split()[2])
                        bearing = float(line.split()[3])
                        meas_info = {'time':time, 'subject_ID':subject_ID, 'measurment_range': measurment_range, 'bearing':bearing}

                        g_idx = find_nearest_time_idx(self.time_arr['groundtruth'][i],time)
                        gt_x = self.groundtruth_data[i][g_idx]['x_pos']
                        gt_y = self.groundtruth_data[i][g_idx]['y_pos']
                        orientation = self.groundtruth_data[i][g_idx]['orientation']
                        matched_gt_info = {'time':time, 'x_pos':gt_x, 'y_pos':gt_y, 'orientation':orientation}

                        self.measurement_data[i].append(meas_info)
                        self.gt_data_meas[i].append(matched_gt_info)
                        self.time_arr['measurement'][i].append(time)


            odo_path = self.dataset_path+"Robot"+robot_num+"_Odometry.dat"
            #print(odo_path)
           
            with open(odo_path,'r+') as odometry_file:
                lines = odometry_file.readlines()
                for line_idx in range(0, len(lines)):



                    line = lines[line_idx]
                    if line[0] != '#' and (self.end_time >= float(line.split()[0]) >= self.start_time):
                        t = float(line.split()[0])
                        time = round(float(line.split()[0]), 3)
                        g_idx = find_nearest_time_idx(self.time_arr['groundtruth'][i],time)
                        velocity = float(line.split()[1])
                        a_v = float(line.split()[2])
                        orientation = self.groundtruth_data[i][g_idx]['orientation']
                        try:
                            next_line = lines[line_idx+1]
                            next_time = float(next_line.split()[0])
                            delta_t = next_time - time
                        except IndexError:
                            delta_t = 0
                        if delta_t < 0:
                            sys.exit('incorrect delta_t: '+ str(delta_t))

                        odo_info = {'time':time, 'velocity':velocity, 'angular velocity': a_v, 'orientation':orientation, 'delta_t': delta_t}
                        gt_x = self.groundtruth_data[i][g_idx]['x_pos']
                        gt_y = self.groundtruth_data[i][g_idx]['y_pos']
                        matched_gt_info = {'time':time, 'x_pos':gt_x, 'y_pos':gt_y, 'orientation':orientation}

                        self.odometry_data[i].append(odo_info)
                        self.gt_data_odo[i].append(matched_gt_info)
                        self.time_arr['odometry'][i].append(time)

        self.data_trackers = {'odometry': np.ones((self.num_robots,), dtype=np.int), 'measurement':np.ones((self.num_robots,), dtype=np.int)}
        # tracker for each robot for both each type of data to keep track of their location in the dataset
        self.odo_data_trackers = np.ones((self.num_robots,), dtype=np.int)
        self.dataset_data = {'odometry': self.odometry_data, 'measurement': self.measurement_data, 'groundtruth': self.groundtruth_data}
        self.dataset_matched_gt_data = {'odometry': self.gt_data_odo, 'measurement': self.gt_data_meas}
        print ('******** Initialization Completed ********')
        print("self.start_time: ", self.start_time)
        print("self.starting_states ", self.starting_states)
        #print(self.dataset_data['odometry'][1])
        return self.start_time, self.starting_states, self.dataset_data, self.time_arr


    def dataset_analysis(self, sampling_period):
    # this function is to find the characteristic of the dataset. Given name of the dataset and period of sampling (in second),
        analysis_result = {}

        odo_varibles = ['velocity', 'angular velocity']
        meas_varibles = ['measurment_range', 'bearing']

        for i, label in enumerate(self.robot_lables):
            t = self.start_time
            while t <= self.start_time+self.duration-sampling_period:
                for odo_variable in odo_varibles:
                    odo_seq = [robot_data[odo_variable] for robot_data in self.odometry_data[i] if t+sampling_period > robot_data['time'] >= t] # create a list of velocity data for each sampling period
                    odo_max = max(odo_seq)
                    odo_std = np.std(odo_seq)

                    print('time: ', t, 'robot ID:', label, odo_variable, '_max:', odo_max, odo_variable, '_std:', odo_std)

                for meas_varible in meas_varibles:
                    meas_seq = [robot_data[meas_varible] for robot_data in self.measurement_data[i] if t+sampling_period > robot_data['time'] >= t] # create a list of velocity data for each sampling period
                    meas_max = max(meas_seq)
                    meas_std = np.std(meas_seq)

                    print('time: ', t, 'robot ID:', label, meas_varible, '_max:', meas_max, meas_varible, '_std:', meas_std)

                t+=sampling_period

        return 1

    def dataset_reset(self):
        #self.data_trackers = {'odometry': np.ones((self.num_robots,), dtype=np.int),'measurement':np.ones((self.num_robots,), dtype=np.int), 'groundtruth':np.ones((self.num_robots,), dtype=np.int)}
        self.data_trackers = {'odometry': np.empty((self.num_robots,)),'measurement':np.empty((self.num_robots,)), 'groundtruth':np.empty((self.num_robots,))}

    def get_start_time(self):
        return self.start_time

    def get_starting_states(self):
        return self.starting_states

    def get_duration(self):
        return self.duration

    def get_landmark_map(self):
        return self.landmark_map

    def get_start_moving_times(self):
        start_moving_times = []
        time_idx = 0
        for i, label in enumerate(self.robot_lables):
            print(i)
            print(self.dataset_data['odometry'][i][0])
            #print(self.dataset_data['odometry'][i])
            start_moving_times.append(self.dataset_data['odometry'][i][time_idx]['time'])
            
        return start_moving_times

    def get_time_arr(self, data_catagory):
        # retunr an array of time shows time for next dataline for each robot given data catagory
        time_arr =[]
        for i, label in enumerate(self.robot_lables):
            time_idx = self.data_trackers[data_catagory][i]
            if time_idx == -1:
            	time_arr.append(self.start_time + self.duration + 10) # so that it will be out of range and not be selected
            else:
            	time_arr.append(self.dataset_data[data_catagory][i][time_idx]['time'])
        return time_arr

    def find_corresponding_ground_truth(self, robot_idx, time):
        data_catagory = 'groundtruth'
        gt_idx = find_nearest_time_idx(self.time_arr[data_catagory][robot_idx], time)
        try:
            self.dataset_data[data_catagory][robot_idx][gt_idx]
        except IndexError:
            print(robot_idx, 'Index ERR: ',  gt_idx)
        matched_gt_data = self.dataset_data[data_catagory][robot_idx][gt_idx]
        return matched_gt_data

    def trackers_sync(self, current_time):
        #print('sync current time: ', current_time )
        for catagory in ['odometry', 'measurement']:
            for robot_idx in range(self.num_robots):
                start_idx = self.data_trackers[catagory][robot_idx]
                if start_idx != -1:
                	time_idx = find_next_time_idx(self.time_arr[catagory][robot_idx], start_idx, current_time)
                	self.data_trackers[catagory][robot_idx] = time_idx

    def trackers_update(self, req_type, robot_idx):
        # update corrsponding trackers
        if self.data_trackers[req_type][robot_idx]!=-1:
            self.data_trackers[req_type][robot_idx]+=1
        #print(self.data_trackers)
        #self.data_trackers['groundtruth'][robot_idx]+=1

    def get_matched_dataline(self, req):
        message = req.get_message()

        if message['robot_index'] == None:
            if req.get_type() == None:
                odo_time_arr = self.get_time_arr('odometry')
                meas_time_arr = self.get_time_arr('measurement')
                if min(odo_time_arr)>min(meas_time_arr):
                    robot_idx = np.argmin(meas_time_arr)
                    req_type = 'measurement'
                    req.set_type(req_type)
                else:
                    robot_idx = np.argmin(odo_time_arr)
                    req_type = 'odometry'
                    req.set_type(req_type)
            else:
                req_type = req.get_type()
                time_arr = self.get_time_arr(req_type)
                robot_idx = np.argmin(time_arr)

            message['robot_index'] = robot_idx

        else:
            robot_idx = message['robot_index']
            if req.get_type() == None:
                odo_time_idx = self.data_trackers['odometry'][robot_idx]
                meas_time_idx = self.data_trackers['measurement'][robot_idx]
                if odo_time_idx != -1 and self.dataset_data['odometry'][robot_idx][odo_time_idx]['time'] >= self.dataset_data['measurement'][robot_idx][meas_time_idx]['time']:
                    req_type = 'measurement'
                    req.set_type(req_type)
                elif meas_time_idx != -1 and self.dataset_data['odometry'][robot_idx][odo_time_idx]['time'] < self.dataset_data['measurement'][robot_idx][meas_time_idx]['time']:
                    req_type = 'odometry'
                    req.set_type(req_type)
                else:
                     valid_dataline = False
            else:
                req_type = req.get_type()

        time_idx = self.data_trackers[req_type][robot_idx]
        if self.dataset_data[req_type][robot_idx][time_idx]['time'] > self.end_time or time_idx == -1:
            valid_dataline = False
        else:
            valid_dataline= True
            message['data'] = self.dataset_data[req_type][robot_idx][time_idx]
            message['groundtruth'] = self.dataset_matched_gt_data[req_type][robot_idx][time_idx]
        return valid_dataline, message, req_type, robot_idx, time_idx


    def create_synthetic_dataline(self, req, time_idx, meas_input_var):
        message = req.get_message()
        req_type = req.get_type()
        robot_idx = message['robot_index']
        req_time = message['time']
        if req_time > self.time_arr[req_type][robot_idx][time_idx]:
            try:
                prev_time_idx = time_idx
                post_time_idx = time_idx+1
                self.time_arr[req_type][robot_idx][post_time_idx]
            except IndexError:
                prev_time_idx = time_idx-1
                post_time_idx = time_idx

        else:
            prev_time_idx = time_idx-1
            post_time_idx = time_idx

        t0= self.time_arr[req_type][robot_idx][prev_time_idx]
        t1 = self.time_arr[req_type][robot_idx][post_time_idx]
        if t1-t0 > 1: # time interval is too big for linear iterpolation
            g_idx = find_nearest_time_idx(self.time_arr['groundtruth'][robot_idx],req_time)
            gt_x = self.groundtruth_data[robot_idx][g_idx]['x_pos']
            gt_y = self.groundtruth_data[robot_idx][g_idx]['y_pos']
            gt_orientation = self.groundtruth_data[robot_idx][g_idx]['orientation']
        else:
        #groudtruth = {'time':time, 'x_pos':gt_x, 'y_pos':gt_y, 'orientation':orientation}
            x0 = self.dataset_matched_gt_data[req_type][robot_idx][prev_time_idx]['x_pos']
            x1 = self.dataset_matched_gt_data[req_type][robot_idx][post_time_idx]['x_pos']
            gt_x = linear_interpolation([t0, x0], [t1, x1], req_time)
            y0 = self.dataset_matched_gt_data[req_type][robot_idx][prev_time_idx]['y_pos']
            y1 = self.dataset_matched_gt_data[req_type][robot_idx][post_time_idx]['y_pos']
            gt_y = linear_interpolation([t0, y0], [t1, y1], req_time)
            o0 = self.dataset_matched_gt_data[req_type][robot_idx][prev_time_idx]['orientation']
            o1 = self.dataset_matched_gt_data[req_type][robot_idx][post_time_idx]['orientation']
            gt_orientation = linear_interpolation([t0, o0], [t1, o1], req_time)

        message['groundtruth'] =  {'time':req_time, 'x_pos':gt_x, 'y_pos':gt_y, 'orientation':gt_orientation}

        if req_type == 'odometry':
            v0 = self.dataset_data[req_type][robot_idx][prev_time_idx]['velocity']
            v1 = self.dataset_data[req_type][robot_idx][post_time_idx]['velocity']
            velocity = linear_interpolation([t0, v0], [t1, v1], req_time)
            a_v0 = self.dataset_data[req_type][robot_idx][prev_time_idx]['angular velocity']
            a_v1 = self.dataset_data[req_type][robot_idx][post_time_idx]['angular velocity']
            a_v = linear_interpolation([t0, a_v0], [t1, a_v1], req_time)
            o0 = self.dataset_data[req_type][robot_idx][prev_time_idx]['orientation']
            o1 = self.dataset_data[req_type][robot_idx][post_time_idx]['orientation']
            orientation =  linear_interpolation([t0, o0], [t1, o1], req_time)

            message['data'] = {'time':req_time, 'velocity':velocity, 'angular velocity': a_v, 'orientation':orientation, 'delta_t': 0}
        else: #measurement:  meas_info = {'time':time, 'subject_ID':subject_ID, 'measurment_range': measurment_range, 'bearing':bearing}
            pass

            subject_ID = self.dataset_data[req_type][robot_idx][time_idx]['subject_ID']
            #bearing = self.dataset_data[req_type][robot_idx][time_idx]['bearing']
            #print(self.dataset_data[req_type][robot_idx][time_idx])
            var_dis = meas_input_var[0]
            var_phi = meas_input_var[1]
            if subject_ID > 5: # landmark
                [lx, ly] = self.landmark_map[subject_ID]
            else:
                if subject_ID not in self.robot_lables:
                    obj_index = (robot_idx+2)%(len(self.robot_lables))
                    subject_ID = self.robot_lables[obj_index]

                obj_index = self.robot_lables.index(subject_ID)
                matched_gt_data = self.find_corresponding_ground_truth(obj_index, req_time)
                lx = matched_gt_data['x_pos']
                ly = matched_gt_data['y_pos']

            np.random.seed(6)
            measurment_range = sqrt((lx-gt_x)*(lx-gt_x)+(ly-gt_y)*(ly-gt_y)) + float(np.random.normal(0, sqrt(var_dis), 1))
            bearing = (atan2((ly-gt_y), (lx-gt_x))-gt_orientation)%pi + float(np.random.normal(0, sqrt(var_phi), 1))
            if abs(bearing-pi) < abs(bearing):
                bearing = bearing-pi

            message['data'] = {'time':req_time, 'subject_ID':subject_ID, 'measurment_range': measurment_range, 'bearing':bearing}

        return message


    def load_most_recent_dataline(self, req, time_idx):
        message = req.get_message()
        req_type = req.get_type()
        robot_idx = message['robot_index']
        req_time = message['time']
        message['data'] = self.dataset_data[req_type][robot_idx][time_idx]
        message['groundtruth'] = self.dataset_matched_gt_data[req_type][robot_idx][time_idx]
        return message

    def get_dataline_at_specific_time(self, req, time_diff_limit = 0.2):
        message = req.get_message()
        req_type = req.get_type()
        meas_input_var = [0.01, 0.01]
        odo_input_var = [0.01, 0.01]
        robot_idx = message['robot_index']
        req_time = message['time']
        if req_time < self.dataset_data[req_type][robot_idx][0]['time']:
            g_idx = find_nearest_time_idx(self.time_arr['groundtruth'][robot_idx],req_time)
            gt_x = self.groundtruth_data[robot_idx][g_idx]['x_pos']
            gt_y = self.groundtruth_data[robot_idx][g_idx]['y_pos']
            gt_orientation = self.groundtruth_data[robot_idx][g_idx]['orientation']
            message['groundtruth'] =  {'time':req_time, 'x_pos':gt_x, 'y_pos':gt_y, 'orientation':gt_orientation}
            if req_type == 'odometry':
                message['data'] = {'time':req_time, 'velocity':0 , 'angular velocity': 0, 'orientation':gt_orientation, 'delta_t': 0}
            else:
                message['data'] = {'time':req_time, 'subject_ID':None, 'measurment_range': 0, 'bearing':0}
            valid_dataline = True
            time_idx = 0
            return valid_dataline, message, req_type, robot_idx, time_idx



        time_idx = find_nearest_time_idx(self.time_arr[req_type][robot_idx], req_time)
        respond_time = self.time_arr[req_type][robot_idx][time_idx]

        message['data'] = self.dataset_data[req_type][robot_idx][time_idx]
        message['groundtruth'] = self.dataset_matched_gt_data[req_type][robot_idx][time_idx]


        if abs(respond_time-req_time) > time_diff_limit and self.adding_actifical_dataline:
            message = self.create_synthetic_dataline(req, time_idx, meas_input_var)
        if req_time > self.end_time:
            valid_dataline = False
        else:
            valid_dataline= True
        return valid_dataline, message, req_type, robot_idx, time_idx


    def respond(self, req, current_time, need_specific_time = False):
        message = req.get_message()
        self.trackers_sync(current_time)
        if need_specific_time:
            valid_respond, message, req_type, robot_idx, time_idx = self.get_dataline_at_specific_time(req)
            current_time = message['time']
        else:
            valid_respond, message, req_type, robot_idx, time_idx = self.get_matched_dataline(req)
            current_time = self.dataset_data[req_type][robot_idx][time_idx]['time']
        if valid_respond:
            #load data
            message['time'] = current_time
            self.trackers_update(req_type, robot_idx)
            req.set_message(message)

        return valid_respond, current_time, req

    def get_robot_groundtruth(self, gt_time, robot_index):
        gt_time_idx = find_nearest_time_idx(self.time_arr['groundtruth'][robot_index], gt_time)
        gt = self.groundtruth_data[robot_index][gt_time_idx]

        return gt




