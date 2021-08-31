import numpy as np

class ekf_algo_framework():
    """this is the framework with all possible functions for localization algorithms"""
    def __init__(self, algo_name):
        self.algo_name = algo_name

    def get_name(self):
        return self.algo_name

    def state_variance_init(self, num_robots):
        raise Exception("Failed to define state variance!")

    def calculate_trace_state_variance(self, robot_data):
        raise Exception("Failed to define trace of state variance!")

    def algo_update(self, robot_data, update_type, sensor_data):
        if update_type == 'propagation':
            return self.propagation_update(robot_data, sensor_data)
        elif update_type == 'landmark observation':
            return self.absolute_obser_update(robot_data, sensor_data)
        elif update_type == 'relative observation':
            return self.relative_obser_update(robot_data, sensor_data)
        elif update_type == 'communication':
            return self.communication(robot_data, sensor_data)
        else:
            print("invalid update")

    def propagation_update(self, robot_data, sensor_data):
        [s, orinetations, sigma_s, index] = robot_data
        return [s, orinetations, sigma_s]

    def absolute_obser_update(self, robot_data, sensor_data):
        [s, orinetations, sigma_s, index] = robot_data
        return [s, orinetations, sigma_s]

    def relative_obser_update(self, robot_data, sensor_data):
        [s, orinetations, sigma_s, index] = robot_data
        return [s, orinetations, sigma_s]

    def communication(self, robot_data, sensor_data):
        [s, orinetations, sigma_s, index] = robot_data
        return [s, orinetations, sigma_s]
