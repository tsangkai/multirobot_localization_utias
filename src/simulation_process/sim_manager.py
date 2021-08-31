from random import randint
from numpy import linalg as LA
import requests.request_response as request_response
import numpy as np
import numpy.matlib
import math
import os
import sys

#sys.path.insert(0, '/Users/william/Documents/SLAM_Simulation/Simulation-Environment-for-Cooperative-Localization/functions/requests')
sys.path.append(os.path.join(os.path.dirname(__file__), "../requests"))


class SimulationManager():
    def __init__(self, name):
        self.name = name

    def comm_controller(self, rbt_idx, num_robots, rsp):
        receiver_idx = rbt_idx
        #sender_idx = (rbt_idx+1)%(num_robots)
        sender_idx = None
        '''
        if rsp.get_type()=='measurement' and rsp.get_data()['subject_ID'] <= 5:
            sender_id = rsp.get_data()['subject_ID']
            sender_idx = self.robot_labels.index(sender_id)
        '''

        if rbt_idx == 0:
            sender_idx = 1

        if rbt_idx == 1:
            sender_idx = 2

        return receiver_idx, sender_idx

    def simulated_communication(self, receiver_status, sender_status):
        max_range = 5  # in meters
        threshold = 0.02

        [receiver_idx, receiver_gt] = receiver_status
        [sender_idx, sender_gt] = sender_status

        r_x, r_y = receiver_gt['x_pos'], receiver_gt['y_pos']
        s_x, s_y = sender_gt['x_pos'], sender_gt['y_pos']
        dis = math.sqrt((r_x-s_x)**2+(r_y-s_y)**2)

        mean = math.exp(-1.5*dis)
        std = math.exp(-0.5*dis)
        '''
        if numpy.random.normal(mean, std) > threshold:
            return True
        else:
            return False
        '''

        return True

    def allow_operation(self, rsp):

        if rsp.get_type() == 'measurement':
            rbt_idx = rsp.get_robot_index()
            rsp_dasa = rsp.get_data()
            if rsp_dasa['subject_ID'] > 5 and self.robots_cant_observe_lm is not None and self.robot_labels[rbt_idx] in self.robots_cant_observe_lm:
                return False

        return True

    def sim_process_native(self, robot_labels, dm, robot_system, state_recorder, simulated_comm=True, comm=True, simple_plot=False, robots_cant_observe_lm=None):
        #dm: dataset manager
        print('******** Simulation Process Started! ********')
        print('communication: ', comm)
        self.robot_labels = robot_labels
        self.robot_system = robot_system
        self.robots_cant_observe_lm = robots_cant_observe_lm

        start_time = dm.get_start_time()
        duration = dm.get_duration()
        self.time = start_time
        prev_time = 0

        #recording starting states
        starting_st = dm.get_starting_states()
        state_recorder.set_starting_state(starting_st)

        #
        robot_system.set_starting_state(dm.get_starting_states())
        robot_system.set_start_moving_times(dm.get_start_moving_times())
        robot_system.load_map(dm.get_landmark_map())

        state_var = []
        while self.time < start_time+duration:
            #print("Current time:", self.time)
            g_req = request_response.Request_response(None, None)
            prev_time = self.time
            valid_op, self.time, rsp = dm.respond(g_req, self.time)
            #print(rsp.get_message())
            if self.time < prev_time:
                print('Time inconsistant!')
                break

            if valid_op and self.allow_operation(rsp):
                robot_state = robot_system.localization_update(rsp)
                state_var.append(robot_state['state variance'])
                state_recorder.record_state(rsp, robot_state)

            # communication protocall
            comm_rsp = None
            if comm:
                if valid_op and rsp.get_type() == 'measurement':

                    rbt_idx = rsp.get_robot_index()
                    receiver_idx, sender_idx = self.comm_controller(
                        rbt_idx, self.robot_system.num_robots, rsp)

                    if sender_idx != None and receiver_idx != None:
                        receiver_gt = dm.get_robot_groundtruth(
                            self.time, receiver_idx)
                        sender_gt = dm.get_robot_groundtruth(
                            self.time, sender_idx)

                        if simulated_comm and self.simulated_communication([sender_idx, sender_gt], [receiver_idx, receiver_gt]):
                            # simulated communication
                            comm_rsp = request_response.Comm_req_resp(
                                self.time, receiver_idx)
                            message = comm_rsp.get_message()

                            sender_id = robot_labels[sender_idx]
                            message['data'] = {
                                'subject_ID': sender_id}
                            message['groundtruth'] = rsp.get_groundtruth
                            comm_rsp.set_message(
                                message)

            if comm_rsp != None:
                #print('comm', robot_labels[sender_idx], robot_labels[receiver_idx])

                robot_state = robot_system.localization_update(
                    comm_rsp)
                state_var.append(robot_state['state variance'])
                state_recorder.record_state(rsp, robot_state)

        if simple_plot:
            state_recorder.simple_plot(plot_title=self.name)

        print('******** Simulation Process Finished! ********')
        print('End time: ', self.time)

        return self.time

    def sim_process_schedule(self, robot_labels, dm, robot_system, state_recorder, freqs, simple_plot=False):
        #dm: dataset manager
        print('******** Simulation Process Started! ********')
        req_type_list = ['odometry', 'measurement', 'communication']
        self.robot_labels = robot_labels
        self.robot_system = robot_system

        start_time = dm.get_start_time()
        duration = dm.get_duration()
        self.time = start_time
        prev_time = 0

        #recording starting states
        starting_st = dm.get_starting_states()
        state_recorder.set_starting_state(starting_st)

        robot_system.set_starting_state(dm.get_starting_states())
        # due to adding data
        robot_system.set_start_moving_times(
            [start_time]*len(robot_labels))
        robot_system.load_map(dm.get_landmark_map())

        next_possible_time_array = [
            [start_time + 1/freq for freq in f] for f in freqs]
        #print(next_possible_time_array)
        while self.time < start_time+duration:
            min_loc = np.argwhere(next_possible_time_array ==
                                  np.min(next_possible_time_array))
            req_type_idx = min_loc[0][0]
            robot_idx = min_loc[0][1]
            req_type = req_type_list[req_type_idx]
            req_time = next_possible_time_array[req_type_idx][robot_idx]
            if req_type != 'communication':
                g_req = request_response.Request_response(
                    req_time, robot_idx)
                g_req.set_type(req_type)
                prev_time = self.time
                valid, rsp_time, rsp = dm.respond(
                    g_req, req_time, need_specific_time=True)
                if valid == False:
                    print('invalid responese')
                    break
                message = rsp.get_message()
                if rsp.get_type() == "odometry":
                    delta_t = 1/freqs[0][robot_idx]
                    message['data']['delta_t'] = delta_t
                    rsp.set_message(message)

                robot_state = robot_system.localization_update(
                    rsp)
                state_recorder.record_state(rsp, robot_state)

            else:
                comm_rsp = request_response.Comm_req_resp(
                    req_time, robot_idx)
                message = comm_rsp.get_message()
                obser_id = (
                    robot_idx+1) % (robot_system.num_robots)
                message['data'] = obser_id
                message['groundtruth'] = rsp.get_groundtruth
                comm_rsp.set_message(message)

                robot_state = robot_system.localization_update(
                    rsp)
                #state_recorder.record_state(rsp, robot_state)

            self.time = next_possible_time_array[req_type_idx][robot_idx]
            if self.time < prev_time:
                print('current time:', self.time)
                print(next_possible_time_array)
                sys.quit('Time inconsistant!')

            next_possible_time_array[req_type_idx][robot_idx] += 1 / \
                freqs[req_type_idx][robot_idx]

        if simple_plot:
            state_recorder.simple_plot(plot_title=self.name)

        print('******** Simulation Process Finished! ********')
        #print('End time: ', self.time)

        return self.time
