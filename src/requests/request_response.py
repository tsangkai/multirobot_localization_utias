#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  3 20:18:50 2018

@author: william
"""

class Request_response:

    def __init__(self, time, idx):
        #self.name = name
        self.message = dict([('time', time), ('robot_index', idx), ('data', None), ('groundtruth', None)])
        self.r_r_type = None
    def get_message(self):
        return self.message
    def get_robot_index(self):
        return self.message['robot_index']
    def get_data(self):
        return self.message['data']
    def get_groundtruth(self):
        return self.message['groundtruth']
    def get_type(self):
        return self.r_r_type

    def set_time(self, time):
        self.message['time'] = time
    def set_robot_idx(self, idx):
        self.message['robot_index'] = idx
    def set_type(self, r_r_type):
        self.r_r_type = r_r_type
    def set_message(self, message):
        self.message = message


class Prop_req_resp(Request_response):

    def __init__(self, time, idx):
        Request_response.__init__(self, time, idx)
        Request_response.set_type(self, 'odometry')

class Meas_req_resp(Request_response):

    def __init__(self, time, idx):    #r_r_type can be landmark or relative measurement
        Request_response.__init__(self, time, idx)
        Request_response.set_type(self, 'measurement')


class Comm_req_resp(Request_response):

    def __init__(self, time, idx):
        Request_response.__init__(self, time, idx)
        Request_response.set_type(self, 'communication')
