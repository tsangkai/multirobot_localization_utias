from math import pi
import numpy as np

import IPython

# Import from other directories in repository
import sys
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

from dataset_manager.data_generator import DataGenerator

def find_nearest_time_idx(l, value):
    if len(l) != 0:
        array = np.asarray(l)
        idx = (np.abs(array-value)).argmin()
        return idx
    else:
        return None

# TODO
# communication
class SimulatedDataSetManager:

    def __init__(self, dataset_name, landmarks, duration, robot_labels,
    velocity_noise = 0, angular_velocity_noise = 0, measurement_range_noise = 0, bearing_noise = 0,  communication_noise = 0,
    robot_fov = 2*pi, delta_t=1.0):

        self.name = dataset_name
        self.duration = duration
        self.robot_labels = robot_labels
        self.num_robots = len(robot_labels)
        self.start_time = 0.0
        self.end_time = self.start_time + duration
        self.start_moving_times = [self.start_time for label in robot_labels]

        # Used to alternate between odometry and measurement and cycle through multiple robots
        self.curr_request = 'odometry'
        self.curr_robot_idx = 0

        # time step of the simulation (secs)
        self.delta_t = delta_t

        # landmarks = {landmark ID: [x,y]} (m)
        self.landmark_map = landmarks
        for landmark_id in landmarks:
            if landmark_id < 5:
                raise Exception("Invalid landmark ID: landmark IDs must be bigger than 5.")

        # starting_states =	{robot label: [time, x_pos, y_pos, orientation], ... }
        # TODO - create a generate starting states function
        self.starting_states = {label : [self.start_time, i, 0, 0.0] for i,label in enumerate(self.robot_labels)}

        # groundtruth & odometry are of fixed length=duration
        # measurement_data is of variable length
        self.time_arr = {}
        self.groundtruth_data = [ [] for robot in range(self.num_robots)]
        self.odometry_data = [[] for robot in range(self.num_robots)]
        self.measurement_data = [[] for robot in range(self.num_robots)]
        self.dataset_data = []

        self.generator = DataGenerator(landmarks, duration, robot_labels, self.starting_states, self.start_time, self.delta_t,
        velocity_noise, angular_velocity_noise, measurement_range_noise, bearing_noise, communication_noise, robot_fov)

    # Input: robot_path (line, circle, etc.)
    # Output: Generates simulated data (gt, odo, meas)
    def simulate_dataset(self, path='random', test=False, velocity=1, velocity_spread = 0):
        if (path == 'line'):
            self.time_arr, self.groundtruth_data, self.odometry_data, self.measurement_data, self.dataset_data = self.generator.generate_straight_line_data(test, velocity, velocity_spread)
            return self.start_time, self.starting_states, self.dataset_data, self.time_arr
        elif(path=='circle'):
            self.time_arr, self.groundtruth_data, self.odometry_data, self.measurement_data, self.dataset_data = self.generator.generate_circular_data(test, velocity, velocity_spread)
            return self.start_time, self.starting_states, self.dataset_data, self.time_arr
        elif(path == 'random'):
            self.time_arr, self.groundtruth_data, self.odometry_data, self.measurement_data, self.dataset_data = self.generator.generate_random_data(test, velocity, velocity_spread)
            return self.start_time, self.starting_states, self.dataset_data, self.time_arr
        else:
            raise Exception("Unsupported robot path specified.")

    def get_dataset_name(self):
        return self.name

    def get_start_time(self):
        return self.start_time

    def get_duration(self):
        return self.duration

    def get_starting_states(self):
        return self.starting_states

    def get_start_moving_times(self):
        return self.start_moving_times

    def get_landmark_map(self):
        return self.landmark_map

    def get_time_arr(self, data_category):
        return self.time_arr[data_category]

    def get_robot_groundtruth(self, gt_time, robot_index):
        gt_time_arr = [ groundtruth['time'] for groundtruth in self.groundtruth_data[robot_index] ]
        gt_index = find_nearest_time_idx(gt_time_arr, gt_time)
        gt = self.groundtruth_data[robot_index][gt_index]
        return gt

    # Used to cycle through all robots in native simulation
    def reset_robot_ctr(self):
        if (self.curr_robot_idx > self.num_robots-1):
            self.curr_robot_idx = 0
            return True
        return False

    # Inputs: request, current_time (from simulation manager)
    # Outputs: valid_respond, current_time, req
    # Calls get_dataline and sets appropriate message state before returning
    # Manages incrementation of current_time to return sim manager
    def respond(self, req, current_time, need_specific_time = False):

        valid_respond, message, req_type, robot_idx = self.get_dataline(req, current_time)

        if (valid_respond):
            req.set_type(req_type)
            req.set_message(message)

        if (self.reset_robot_ctr()): # and req_type == 'measurement'
            current_time += self.delta_t

        return valid_respond, current_time, req

    # Inputs: current time, empty request (passed by respond)
    # Outputs: valid_dataline, message, req_type, robot_idx
    # Selects robot and request type, emplaces message data
    def get_dataline(self, req, current_time):
        message = req.get_message()

        if message['robot_index'] == None:
            robot_idx = self.curr_robot_idx
            # select req_type, robot_idx
            if req.get_type() == None:
                req_type = self.curr_request
                # alternate odometry and measurement requests
                if (self.curr_request == 'odometry'):
                    self.curr_request = 'measurement'
                else:
                    self.curr_request = 'odometry'
                    # go to next robot
                    self.curr_robot_idx += 1
            else:
                req_type = req.get_type()

            message['robot_index'] = robot_idx

        # Past the total time of simulation or no further measurements
        valid_dataline = current_time <= self.end_time or (abs(current_time - self.end_time) < 0.001) # <= w/floats

        # Select message data
        if valid_dataline:
            if req_type == 'odometry':
                odo_time_idx = find_nearest_time_idx(self.time_arr[req_type][robot_idx], current_time)
                message['data'] = self.odometry_data[robot_idx][odo_time_idx]
            # consider case of no measurements
            elif req_type == 'measurement':
                message['data'] = self.find_measurement(current_time, robot_idx)

        gt_time_idx = find_nearest_time_idx(self.time_arr['groundtruth'][robot_idx], current_time)
        message['groundtruth'] = self.groundtruth_data[robot_idx][gt_time_idx]

        message['time'] = current_time

        return valid_dataline, message, req_type, robot_idx

    # Inputs: current time, robot_idx
    # Outputs:  closest landmark measurement for given time and robot, if available
    #           {'time': current_time, 'subject_ID': None, 'measurment_range': None,'bearing': None} if no measurement available
    # NOTE - prefers absolute observations over relative observations
    def find_measurement(self, current_time, robot_idx):

        available_measurements = []
        measurement_indices = []

        # landmark measurements
        for data_idx, measurement in enumerate(self.measurement_data[robot_idx]):
            if (abs(measurement['time'] - current_time) < 0.001 and measurement['subject_ID']  > 5):
                available_measurements.append(measurement['measurment_range'])
                measurement_indices.append(data_idx)

        # Relative measurements
        if (len(available_measurements) == 0):
            for data_idx, measurement in enumerate(self.measurement_data[robot_idx]):
                if (abs(measurement['time'] - current_time) < 0.001 and measurement['subject_ID']  <= 5): # relative observation
                    available_measurements.append(measurement['measurment_range'])
                    measurement_indices.append(data_idx)

        # No measurements -> invalid subject id "-1"
        if (len(available_measurements) == 0):
            return {'time': current_time,'subject_ID': -1, 'measurment_range': 0, 'bearing': 0}

        least_index = np.argmin(available_measurements)
        idx = measurement_indices[least_index]
        return self.measurement_data[robot_idx][idx]
