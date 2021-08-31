# -*- coding: utf-8 -*-

import numpy as np
from numpy import random
import sys
from math import atan2, sqrt, pi
import random
from math import cos
from math import sin
import math
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

def get_pos_within_rad(rad):
    rad = abs(rad)
    while True:
        x = random.uniform(-1*rad, rad)
        y = random.uniform(-1*rad, rad)
        if (((x**2) + (y**2))**.5 > rad):
            continue
        else:
            return x, y

def sanitize_angle(angle):
    if -3.1415 > angle:
        temp = abs(angle) - 3.1415
        angle = 3.1415 - temp       
    if angle > 3.1415:
        temp = abs(angle) - 3.1415
        angle = -3.1415 + temp
    return angle

class Dataset:
    
    def __init__(self, dataset_name):
        random.seed(a=100) #start a seed for the random data generator
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
            

    def load_MRCLAMDatasets(self, dataset_path, dataset_labels, duration, adding_actifical_dataline = True, delay_start = 10, synthetic = True):
        print ('******** Initialization Started ********')
        self.synthetic = synthetic
        print ('add synthetic data: ', self.synthetic)
        self.dataset_path = dataset_path
        self.dataset_labels = dataset_labels
        self.adding_actifical_dataline = adding_actifical_dataline
        self.create_landmark_map()

        self.num_robots = len(self.dataset_labels)
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
        for i, label in enumerate(self.dataset_labels):
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
        for i, label in enumerate(self.dataset_labels):
            robot_num = str(label)
            groundtruth_path = self.dataset_path+"Robot"+robot_num+"_Groundtruth.dat"
            with open(groundtruth_path,'r+') as groundtruth_file:
                for line in groundtruth_file:
                    if line[0] != '#' and float(line.split()[0]) >= self.start_time:
                        time = float(line.split()[0])
                        x_pos = float(line.split()[1])
                        y_pos = float(line.split()[2])
                        orientation = float(line.split()[3])
                        self.starting_states[label] = [time, x_pos, y_pos, orientation]
                        break
        print('Staring states: ')
        print(self.starting_states)
        
    
        for i, label in enumerate(self.dataset_labels):
            robot_num = str(label)

            groundtruth_path = self.dataset_path+"Robot"+robot_num+"_Groundtruth.dat"
            with open(groundtruth_path,'r+') as groundtruth_file:
                for line in groundtruth_file:
                    if line[0] != '#' and (self.end_time+2 >= float(line.split()[0]) >= self.start_time):
                        time = float(line.split()[0])
                        x_pos = float(line.split()[1])
                        y_pos = float(line.split()[2])
                        orientation = float(line.split()[3])
                        groundtruth_info = {'time':time, 'x_pos':x_pos, 'y_pos':y_pos, 'orientation':orientation}
                        self.groundtruth_data[i].append(groundtruth_info) 
                        self.time_arr['groundtruth'][i].append(time)


            meas_path = self.dataset_path+"Robot"+robot_num+"_Measurement_x.dat"
            with open(meas_path,'r+') as measure_file:
                for line in measure_file:
                    if line[0] != '#' and (self.end_time+2 >= float(line.split()[0]) >= self.start_time):
                        time = float(line.split()[0])
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
            with open(odo_path,'r+') as odometry_file:
                for line in odometry_file:
                    if line[0] != '#' and (self.end_time+2 >= float(line.split()[0]) >= self.start_time):
                        time = float(line.split()[0])
                        g_idx = find_nearest_time_idx(self.time_arr['groundtruth'][i],time) 
                        velocity = float(line.split()[1])
                        orientation = self.groundtruth_data[i][g_idx]['orientation']
                        #print "orientation ", orientation
                        odo_info = {'time':time, 'velocity':velocity, 'orientation':orientation}

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
        
        return self.start_time, self.starting_states, self.dataset_data, self.time_arr

    def dataset_reset(self):
        self.data_trackers = {'odometry': np.ones((self.num_robots,), dtype=np.int),'measurement':np.ones((self.num_robots,), dtype=np.int), 'groundtruth':np.ones((self.num_robots,), dtype=np.int)} 
        
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
        for i, label in enumerate(self.dataset_labels):
            start_moving_times.append(self.dataset_data['odometry'][i][time_idx]['time'])
        return start_moving_times

    def get_time_arr(self, data_catagory):
        # retunr an array of time shows time for next dataline for each robot given data catagory
        time_arr =[] 
        for i, label in enumerate(self.dataset_labels):
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
        for catagory in ['odometry', 'measurement', 'groundtruth']:
            for robot_idx in range(self.num_robots):
                start_idx = self.data_trackers[catagory][robot_idx]
                if start_idx != -1:
                    time_idx = find_next_time_idx(self.time_arr[catagory][robot_idx], start_idx, current_time)
                    self.data_trackers[catagory][robot_idx] = time_idx 
                    
    def trackers_update(self,req_type,robot_idx):
        # update corrsponding trackers
        self.data_trackers[req_type][robot_idx]+=1 
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
                time_idx = self.data_trackers['odometry'][robot_idx]
                if self.dataset_data['odometry'][robot_idx][time_idx]['time']>self.dataset_data['measurement'][robot_idx][time_idx]['time']:
                    req_type = 'measurement'
                    req.set_type(req_type)
                else:
                    req_type = 'odometry'
                    req.set_type(req_type)
            else:
                req_type = req.get_type()

        time_idx = self.data_trackers[req_type][robot_idx]
        if self.dataset_data[req_type][robot_idx][time_idx]['time'] > self.end_time:
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
            o0 = self.dataset_data[req_type][robot_idx][prev_time_idx]['orientation']
            o1 = self.dataset_data[req_type][robot_idx][post_time_idx]['orientation']
            orientation =  linear_interpolation([t0, o0], [t1, o1], req_time)
            message['data'] = {'time':req_time, 'velocity':velocity, 'orientation':orientation}
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
                if subject_ID not in self.dataset_labels:
                    obj_index = (robot_idx+2)%(len(self.dataset_labels))
                    subject_ID = self.dataset_labels[obj_index]

                obj_index = self.dataset_labels.index(subject_ID)
                matched_gt_data = self.find_correspoind_ground_truth(obj_index, req_time)
                lx = matched_gt_data['x_pos']
                ly = matched_gt_data['y_pos']

            measurment_range = sqrt((lx-gt_x)*(lx-gt_x)+(ly-gt_y)*(ly-gt_y)) + random.normal(0, sqrt(var_dis))
            # bearing not matching with closest dataline....
            bearing = (atan2((ly-gt_y), (lx-gt_x))-gt_orientation)%pi + random.normal(0, sqrt(var_dis))  
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

    def create_additional_dataline(self, req, time_idx, meas_input_var):
        if self.synthetic:
            message = self.create_synthetic_dataline(req, time_idx, meas_input_var)
        else:
            message = self.load_most_recent_dataline(req, time_idx)
        return message

    
    def get_dataline_at_specific_time(self, req, time_diff_limit = 0.1):
        message = req.get_message()
        req_type = req.get_type()
        robot_idx = message['robot_index']
        req_time = message['time']
        time_idx = find_nearest_time_idx(self.time_arr[req_type][robot_idx], req_time)
        respond_time = self.time_arr[req_type][robot_idx][time_idx]
        
        message['data'] = self.dataset_data[req_type][robot_idx][time_idx]
        message['groundtruth'] = self.dataset_matched_gt_data[req_type][robot_idx][time_idx]
        
        if abs(respond_time-req_time) > time_diff_limit and self.adding_actifical_dataline:
            meas_input_var = [0.1, 0.1]
            message = self.create_additional_dataline(req, time_idx, meas_input_var)

        if req_time > self.end_time:
            valid_dataline = False
        else:
            valid_dataline= True 
        return valid_dataline, message, req_type, robot_idx, time_idx


    def respond(self, req, current_time, need_specific_time = False):
        message = req.get_message()
        self.trackers_sync(current_time)
        valid_respond, req_type, robot_idx, time_idx = self.get_matched_dataline(req)
        if valid_respond:
            #load data
            current_time = self.dataset_data[req_type][robot_idx][time_idx]['time']
            message['time'] = self.dataset_data[req_type][robot_idx][time_idx]['time']
            message['data'] = self.dataset_data[req_type][robot_idx][time_idx]
            #load corresponding groundtruth
            time = self.dataset_data[req_type][robot_idx][time_idx]['time']
            matched_gt_data = self.find_corresponding_ground_truth(robot_idx, time)
            message['groundtruth'] = matched_gt_data
            self.trackers_update(req_type,robot_idx)
            req.set_message(message)

        return valid_respond, current_time, req


    def generate_random_landmark_map(self):
        """num_landmarks = random.randint(8, 20)
                
        self.landmark_map = {}     
        for i in range(num_landmarks):
            x_coord, y_coord = get_pos_within_rad(5)
            self.landmark_map[i+1] = [x_coord, y_coord]
            """
        self.landmark_map = {1: [-5.0, 0.0]}
        print("map", self.landmark_map)
        return self.landmark_map
                   
    # take an x and y from the robot and the first two params,
    # and a list of coords for the landmark as the second param
    # return the distance between them
    def landmark_distance(self, old_x, old_y, new_coords):
        new_x, new_y = new_coords[0], new_coords[1]
        return math.sqrt((old_x - new_x)**2 + (old_y - new_y)**2)
        
    def getBearing(self, x, y):
        return math.atan2(y, x)

    def getLineAngle(self, my_x, my_y, lm_x, lm_y, orientation):
        theta = self.getBearing(my_x - lm_x, my_y - lm_y)
        #need to account that you don't yet know which direction the angle is,
        #eg robot to lm or lmm to robot
        if (theta > 0) and (my_y > lm_y):
            theta = -3.1415 + theta 
        elif (theta < 0) and (my_y < lm_y):
            print("correcting")
            theta = 3.1415 + theta        
        
        angle_from_robot = sanitize_angle(theta - orientation)     
        
        return angle_from_robot
        
    def get_closest_landmark(self, x, y):
        closest_distance = 10
        closest_lm = 0
        for key, coords in self.landmark_map.items():
            distance = self.landmark_distance(x, y, coords)
            if (distance < closest_distance):
                closest_distance = distance
                closest_lm = key
                
        #print closest_lm, closest_distance, x, y, coords[0], coords[1]
        return closest_lm, closest_distance
        
    def get_second_closest_landmark(self, x, y):
        closest_lm, closest_distance = self.get_closest_landmark(x, y)
        second_closest_distance = 10
        second_closest_lm = 0
        for key, coords in self.landmark_map.items():
            distance = self.landmark_distance(x, y, coords)
            if (distance < second_closest_distance and not(key == closest_lm)):
                second_closest_distance = distance
                second_closest_lm = key
                
        return second_closest_lm, second_closest_distance
        
    def initialization_Random_Dataset(self, duration):
        print ('******** Random Initialization Started ********')
        self.generate_random_landmark_map()#
    
        self.num_robots = len(self.dataset_labels)
        self.measurement_data = [[] for i in range(self.num_robots)]
        self.odometry_data = [[] for i in range(self.num_robots)]
        self.groundtruth_data = [[] for i in range(self.num_robots)]
        self.time_arr = {'odometry': [[] for i in range(self.num_robots)], 'measurement': [[] for i in range(self.num_robots)], 'groundtruth': [[] for i in range(self.num_robots)]}

        self.duration = duration # some more time to avoid index error
        self.start_time_arr = []
            
            
        #finding the starting time:
        self.start_time = (1e9)                                 
        self.end_time = self.start_time + self.duration
        print('Staring time: ', self.start_time)
                    
        #finding starting states:
        self.starting_states = {}
        robot_movement_dict = {}
        for i, label in enumerate(self.dataset_labels):
            robot_num = str(label)
                
            #generate the position starts here
            x_pos, y_pos = get_pos_within_rad(5)
            orientation = 0#random.uniform(-3.1415, 3.1415)
                
            self.starting_states[label] = [self.start_time, x_pos, y_pos, orientation]
        print('Starting states: ')
        print(self.starting_states)            
        
        #
        for i, label in enumerate(self.dataset_labels):
            robot_num = str(label)
            
            #give the robot a random starting position
            velocity = random.uniform(0.0, 1.0)
            inits = self.starting_states[label]
            x_pos = inits[1]
            y_pos = inits[2]
            orientation = inits[3]
            this_robot = Robot_Movement_Generator(x_pos, y_pos, orientation, robot_num)
            robot_movement_dict[robot_num] = this_robot
                           
            
            measurement_file = open("random" + robot_num + "_measurement_x.dat", 'w')
            
            time = self.start_time
            ticks = 0
            while (time < self.end_time):# this is a fixed number of iterations right now
                #time moves at a constant step right now
                time_delta = 0.0195
                time = time + time_delta
                ticks = ticks + 1
                
                velocity, orientation, x_pos, y_pos = this_robot.run_tick(time, time_delta)
                
                #these are biased values that are approximated by the robot from the groundtruth
                odo_info = {'time':time, 'velocity':velocity, 'orientation':orientation}
                self.odometry_data[i].append(odo_info)
                self.time_arr['odometry'][i].append(time)# I think there may be a mistake here. This value isn't meant to be orientation, it should be angular velocity
                
                groundtruth_info = {'time':time, 'x_pos':x_pos, 'y_pos':y_pos, 'orientation':orientation} # groundtruth
                self.groundtruth_data[i].append(groundtruth_info) 
                self.time_arr['groundtruth'][i].append(time)
                
                #choose a random landmark and calculate the distance from it
                if ticks % 23 == 0:         
                    closest_landmark, closest_distance = self.get_closest_landmark(x_pos, y_pos)
                    if closest_distance < 5:
                        measurement_range = closest_distance #+ random.gauss(0.0, .001) #TODO: find a legitimate way to choose this sigma
                        bearing = self.getLineAngle(x_pos, y_pos, self.landmark_map[closest_landmark][0], self.landmark_map[closest_landmark][1], orientation)
                        meas_info = {'time':time, 'subject_ID':closest_landmark, 'measurment_range': measurement_range, 'bearing':bearing}
                        print(meas_info)
                        print(x_pos, y_pos, self.landmark_map[closest_landmark][0], self.landmark_map[closest_landmark][1], bearing)
                        measurement_file.write(str(time) + "\t" + str(closest_landmark) + "\t" + str(measurement_range) + "\t" + str(bearing) + "\n")
                        self.measurement_data[i].append(meas_info)
                        self.time_arr['measurement'][i].append(time)
                    second_closest_landmark, second_closest_distance = self.get_second_closest_landmark(x_pos, y_pos)
                    if second_closest_distance < 3:
                        measurement_range = second_closest_distance #+ random.gauss(0.0, .001) #TODO: find a legitimate way to choose this sigma
                        bearing = self.getLineAngle(x_pos, y_pos, self.landmark_map[second_closest_landmark][0], self.landmark_map[second_closest_landmark][1], orientation)
                        print(meas_info)
                        meas_info = {'time':time, 'subject_ID':second_closest_landmark, 'measurment_range': measurement_range, 'bearing':bearing}
                        measurement_file.write(str(time) + "\t" + str(second_closest_landmark) + "\t" + str(measurement_range) + "\t" + str(bearing) + "\n")
                        self.measurement_data[i].append(meas_info)
                        self.time_arr['measurement'][i].append(time)
                    
                
            measurement_file.close()
            this_robot.close_file()
        self.data_trackers = {'odometry': np.ones((self.num_robots,), dtype=np.int),'measurement':np.ones((self.num_robots,), dtype=np.int), 'groundtruth':np.ones((self.num_robots,), dtype=np.int)} 
        # tracker for each robot for both each type of data to keep track of their location in the dataset
        self.odo_data_trackers = np.ones((self.num_robots,), dtype=np.int)
        self.dataset_data = {'odometry': self.odometry_data, 'measurement': self.measurement_data, 'groundtruth': self.groundtruth_data}
        print ('******** Initialization Completed ********')
            
        return self.start_time, self.starting_states, self.dataset_data, self.time_arr
    
    
 
class Robot_Movement_Generator:
    def __init__(self, x, y, orientation, num):
        self.robot_num = num
        self.velocity = .04#random.uniform(0.0, .1)   
        self.max_velocity = 0#.1
        self.orientation = orientation
        self.angular_velocity = 0#.15
        self.max_angular_velocity = 0#.15
        self.x = x
        self.y = y
        self.sensor_noise = 0#.25
        self.odometry_file = open("random" + self.robot_num + "_odometry.dat", 'w')
        self.groundtruth_file = open("random"+ self.robot_num + "_groundtruth.dat", 'w')
    
    def close_file(self):
        self.odometry_file.close()
        self.groundtruth_file.close()
        
    def update_xy(self, time_delta):
        #check if the update is in bounds and if so do it 
        temp_x = self.x + cos(self.orientation) * self.velocity * (time_delta)
        temp_y = self.y + sin(self.orientation) * self.velocity * (time_delta)
        if ((temp_x**2 + self.y**2)**.5 < 5) and ((self.y**2 + temp_y**2)**.5 < 5):
            self.x = temp_x
            self.y = temp_y
        else:
            self.velocity = 0
        return self.x, self.y         

        
    def update_angular_velocity(self):
        self.angular_velocity = self.max_angular_velocity
        return self.angular_velocity
    
    def update_velocity(self):
        self.velocity = self.max_velocity
        return self.velocity
        
    def update_orientation(self, time_delta):
        self.orientation += self.update_angular_velocity() * time_delta
        self.orientation = sanitize_angle(self.orientation)
        return self.orientation 

        
    def run_tick(self, time, time_delta):
        #add biasing here, after we've already computed an accurate groundtruth
        v = self.update_velocity()+ random.gauss(0.0, self.sensor_noise)
        o = self.update_orientation(time_delta)+ random.gauss(0.0, self.sensor_noise)
        x, y = self.update_xy(time_delta)

                
        self.odometry_file.write(str(time) + "\t" + str(v) + "\t\t" + str(self.angular_velocity) + "\n")
            
        self.groundtruth_file.write(str(time) + "\t" + str(self.x) + "\t" + str(self.y) + "\t" + str(self.orientation) + "\n")
        return (v, o, x, y)
        
     
        
                       
        
# test script
"""
rgf = open('random1_groundtruth.dat', 'r')
rof = open('random1_odometry.dat', 'r')
end_time=1000000100
start_time = 1000000000

times = []
x_poss = []
y_poss = []
orientations = []

times2 = []
velocities = []
orientations2 = []
for line in rgf:
    if line[0] != '#' and (end_time+2 >= float(line.split()[0]) >= start_time):
        times.append(float(line.split()[0]))
        x_poss.append(float(line.split()[1]))
        y_poss.append(float(line.split()[2]))
        orientations.append(float(line.split()[3]))
        
for line in rof:
    if line[0] != '#' and (end_time+2 >= float(line.split()[0]) >= start_time):
        times2.append(float(line.split()[0]))
        velocities.append(float(line.split()[1]))
        orientations2.append(float(line.split()[2]))

for i in range(len(rof)):
    

print orientations, orientations2"""
