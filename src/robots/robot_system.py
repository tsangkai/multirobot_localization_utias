from robots.robot_distributive import DistributiveRobot
from robots.robot_centralized import CentralizedRobotSystem
import os
import sys
import numpy as np


#sys.path.insert(0, '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/functions/robots')
sys.path.append(os.path.join(os.path.dirname(__file__), "."))


class RobotSystem:
    """
    general class for robot which can be distrbutive or centralized
    """

    def __init__(self, name, robot_labels, loc_algo, bound_algo=None, distr_sys=True,):
        self.name = name
        self.robot_labels = robot_labels
        self.num_robots = int(len(robot_labels))
        self.distr_sys = distr_sys
        self.time = 0
        if distr_sys:
            self.robot_sys = []
            for i, label in enumerate(self.robot_labels):
                dis_robot = DistributiveRobot(robot_labels, i, loc_algo)
                self.robot_sys.append(dis_robot)
            # create a list of distibutive robot
            print('created a team of ', len(
                self.robot_sys), ' distibutive robots ')

        else:
            self.robot_sys = CentralizedRobotSystem(
                name, robot_labels, loc_algo)
            print('created a centralized system for ',
                  self.num_robots, ' robots ')
        self.gt_orientations = np.zeros(self.num_robots)

    def set_starting_state(self, start_state_arr):
        if self.distr_sys:
            for i, label in enumerate(self.robot_labels):
                self.robot_sys[i].set_starting_state(start_state_arr)
        else:
            self.robot_sys.set_starting_state(start_state_arr)

    def request_data(self, time):
        pass
    '''
    def load_map(self, landmark_map):
        if self.distr_sys:
            for i, label in enumerate(self.robot_labels):
                self.robot_sys[i].load_map(landmark_map)
        else:
            self.robot_sys.load_map(landmark_map)
    '''
    '''
    def set_start_moving_times(self, start_moving_times):
        if self.distr_sys:
            for i, label in enumerate(self.robot_labels):
                self.robot_sys[i].set_start_moving_times(start_moving_times)
        else:
            self.robot_sys.set_start_moving_times(start_moving_times)
    '''

    def load_map(self, landmark_map):
        self.landmark_map = landmark_map

    def set_start_moving_times(self, start_moving_times):
        self.prev_prop_times = start_moving_times

    def gt_update_orientation(self, rbt_idx, update_orientation):
        self.gt_orientations[rbt_idx] = update_orientation

    def localization_update(self, rsp):
        #message format: {'time': float, 'robot_index': int, 'data': {'time': float, 'velocity': float, 'orientation': float}, 'groundtruth': None}
        message = rsp.get_message()
        self.time = message['time']
        robot_index = message['robot_index']
        message_data = message['data']
        rsp_type = rsp.get_type()

        valid_update = True

        if rsp_type == 'odometry':  # propogation update
            update_type = 'propagation'
            v = message_data['velocity']
            a_v = message_data['angular velocity']

            #sigma_odo = np.matrix([[5.075*v, 0], [0, 0.1]]) #with respect to velocity and orientation for utias datasets
            # with respect to velocity and orientation
            sigma_odo = np.matrix([[0.01, 0], [0, 0.1]])
            sensor_covariance = sigma_odo
            delta_t = message_data['delta_t']

            self.gt_update_orientation(
                robot_index, message_data['orientation'])

            if delta_t < 0:
                print('current time: ', message_data['time'])
                print('prev_prop_time: ', self.prev_prop_times[robot_index])
                raise Exception("Error incorrect delta_t!")

            sensor_input = [delta_t, v,
                            message_data['orientation'], self.gt_orientations]
            self.prev_prop_times[robot_index] = message_data['time']

        elif rsp_type == 'measurement':

            # with respect to range and bearing
            sigma_obser = np.matrix([[0.015, 0], [0, 0.01]])
            sensor_covariance = sigma_obser
            obj_id = message_data['subject_ID']
            meas_range = message_data['measurment_range']
            bearing = message_data['bearing']
            # landmark observation
            if message_data['subject_ID'] != None and message_data['subject_ID'] > 5:
                update_type = 'landmark observation'
                landmark_loc = self.landmark_map.get(obj_id)
                if landmark_loc != None:
                    sensor_input = [landmark_loc, meas_range,
                                    bearing, message_data['time'], obj_id]
                    valid_update = True
                else:
                    valid_update = False
            else:
                if obj_id in self.robot_labels:
                    update_type = 'relative observation'
                    obser_index = self.robot_labels.index(obj_id)
                    valid_update = True
                    sensor_input = [obser_index, meas_range,
                                    bearing, message_data['time'], obj_id]
                else:
                    update_type = 'invalid observation'
                    print("No measurements for robot:",
                          self.robot_labels[robot_index], " at time:", self.time)
                    valid_update = False
                    sensor_input = None

        elif rsp_type == 'communication':
            update_type = 'communication'
            obj_id = message_data['subject_ID']
            sender_idx = self.robot_labels.index(obj_id)
            if self.distr_sys:
                [comm_rbt_state, orientaions,
                    comm_rbt_state_variance] = self.robot_sys[sender_idx].get_status()
            else:
                [comm_rbt_state, orientaions,
                    comm_rbt_state_variance] = self.robot_sys.get_status()
            sensor_input = [sender_idx, comm_rbt_state,
                            comm_rbt_state_variance]
            sensor_covariance = np.matrix([[0.01, 0], [0, 0.01]])
        else:

            print('undefined message type!')

        if valid_update:
            sensor_data = [sensor_input, sensor_covariance]

        if valid_update:
            if self.distr_sys:
                [est_states, est_orientaions, est_state_variance,
                    update_type] = self.robot_sys[robot_index].state_update(update_type, sensor_data)
                trace_sigma_s = self.robot_sys[robot_index].get_trace_state_variance(
                )
            else:
                [est_states, est_orientaions, est_state_variance, update_type] = self.robot_sys.state_update(
                    robot_index, update_type, sensor_data)
                trace_sigma_s = self.robot_sys.get_trace_state_variance(
                    robot_index)
        else:
            if self.distr_sys:
                [est_states, est_orientaions,
                    est_state_variance] = self.robot_sys[robot_index].get_status()
                trace_sigma_s = self.robot_sys[robot_index].get_trace_state_variance(
                )
            else:
                [est_states, est_orientaions,
                    est_state_variance] = self.robot_sys.get_status()
                trace_sigma_s = self.robot_sys.get_trace_state_variance(
                    robot_index)

        robot_state = {'est_states': est_states, 'x_pos': est_states[2*robot_index], 'y_pos': est_states[2*robot_index+1],
                       'trace of state variance': trace_sigma_s, 'state variance': est_state_variance, 'update_type': update_type}
        return robot_state
