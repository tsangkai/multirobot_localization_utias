import numpy as np
import utils as u

NUM_ROBOTS = 5

SENSOR_VAR = 0.01
RANGE_VAR = 0.0215
BEARING_VAR = 0.01


class Landmark():

    def __init__(self, subject_num, barcode_num, xy, stddev):
        self.subject_num = subject_num
        self.barcode_num = barcode_num
        self.xy = xy
        self.stddev = stddev


class RobotSystem():

    def __init__(self, dataset, team_settings):
        self.robots = self.init_robots(dataset)
        self.landmarks = self.init_map(dataset)

        self.dt = team_settings['dt']
        self.offset = team_settings['offset']
        self.duration = team_settings['duration']
        
        self.comm_prob_fail = team_settings['comm_prob_fail']

    def init_robots(self, dataset):
        robots = list()

        for r in range(NUM_ROBOTS):
            subject_num = dataset['Barcodes'][r]['Subject #']
            barcode_num = dataset['Barcodes'][r]['Barcode #']

            robot = self.Robot(subject_num, barcode_num)
            robots.append(robot)

        return robots

    def init_map(self, dataset):
        landmarks = list()

        # find number of landmarks
        num_landmarks = len(dataset['Landmark_Groundtruth'])

        for l in range(num_landmarks):
            subject_num = dataset['Landmark_Groundtruth'][l]['Subject #']
            barcode_num = dataset['Barcodes'][l+NUM_ROBOTS]['Barcode #']

            xy = np.array((dataset['Landmark_Groundtruth'][l]['x [m]'], dataset['Landmark_Groundtruth'][l]['y [m]']))
            stddev = np.array((dataset['Landmark_Groundtruth'][l]['x std-dev [m]'], dataset['Landmark_Groundtruth'][l]['y std-dev [m]']))

            landmark = Landmark(subject_num, barcode_num, xy, stddev)
            landmarks.append(landmark)

        return landmarks


class CentralizedTeam(RobotSystem):

    class Robot():

        def __init__(self, subject_num, barcode_num):
            self.subject_num = subject_num
            self.barcode_num = barcode_num

            self.history = dict()
            self.history['est'] = list()
            self.history['gt'] = list()

    def __init__(self, dataset, team_settings):
        RobotSystem.__init__(self, dataset, team_settings)

        self.xys = np.zeros(shape=(2*NUM_ROBOTS), dtype=float)
        self.thetas = np.zeros(shape=(NUM_ROBOTS), dtype=float)
        self.covs = np.zeros(shape=(2*NUM_ROBOTS, 2*NUM_ROBOTS), dtype=float)
        
        self.covs_i = np.zeros(shape=(2*NUM_ROBOTS, 2*NUM_ROBOTS), dtype=float) # for LS-SCI
        self.covs_d = np.zeros(shape=(2*NUM_ROBOTS, 2*NUM_ROBOTS), dtype=float) # for LS-SCI

        self.init_team(dataset)

    def init_team(self, dataset):

        start_time, end_time = u.get_times(dataset, self.offset, self.duration)

        for r in range(NUM_ROBOTS):

            # get ground truth times
            times = [dataset['Robot' + str(r+1)]['Groundtruth'][k]['Time [s]'] for k in range(len(dataset['Robot' + str(r+1)]['Groundtruth']))]

            # get nearest index
            idx = np.argmin(np.abs(start_time - times))

            # save into history
            time = dataset['Robot' + str(r+1)]['Groundtruth'][idx]['Time [s]']
            x = dataset['Robot' + str(r+1)]['Groundtruth'][idx]['x [m]']
            y = dataset['Robot' + str(r+1)]['Groundtruth'][idx]['y [m]']
            theta = dataset['Robot' + str(r+1)]['Groundtruth'][idx]['orientation [rad]']

            self.xys[2*r:2*r+2] = np.array((x, y))
            self.thetas[r] = theta

            self.robots[r].history['gt'].append({'time': time, 'x': x, 'y': y, 'theta': theta})
            self.robots[r].history['est'].append({'time': time, 'x': x, 'y': y, 'theta': theta, 'cov': np.identity(2)*0.01})

        self.covs = np.identity(2*NUM_ROBOTS)*0.01
        
        self.covs_i = np.identity(2*NUM_ROBOTS)*0.01*0.99
        self.covs_d = np.identity(2*NUM_ROBOTS)*0.01*0.01


class DistributiveTeam(RobotSystem):

    class Robot():

        def __init__(self, subject_num, barcode_num):
            self.subject_num = subject_num
            self.barcode_num = barcode_num

            self.history = dict()
            self.history['est'] = list()
            self.history['gt'] = list()

            self.xys = np.zeros(shape=(2*NUM_ROBOTS), dtype=float)
            self.thetas = np.zeros(shape=(NUM_ROBOTS), dtype=float)
            self.covs_i = np.zeros(shape=(2*NUM_ROBOTS, 2*NUM_ROBOTS), dtype=float) # for GS-SCI
            self.covs_d = np.zeros(shape=(2*NUM_ROBOTS, 2*NUM_ROBOTS), dtype=float) # for GS-SCI
            self.covs = np.zeros(shape=(2*NUM_ROBOTS, 2*NUM_ROBOTS), dtype=float) # for GS-CI

    def __init__(self, dataset, team_settings):
        RobotSystem.__init__(self, dataset, team_settings)

        self.comm_type = team_settings['comm_type']
        self.comm_range = team_settings['comm_range']
        self.comm_rate = team_settings['comm_rate']

        self.init_team(dataset)

    def init_team(self, dataset):

        start_time, end_time = u.get_times(dataset, self.offset, self.duration)

        for r in range(NUM_ROBOTS):

            # get ground truth times
            times = [dataset['Robot' + str(r+1)]['Groundtruth'][k]['Time [s]'] for k in range(len(dataset['Robot' + str(r+1)]['Groundtruth']))]

            # get nearest index
            idx = np.argmin(np.abs(start_time - times))

            # save into history
            time = dataset['Robot' + str(r+1)]['Groundtruth'][idx]['Time [s]']
            x = dataset['Robot' + str(r+1)]['Groundtruth'][idx]['x [m]']
            y = dataset['Robot' + str(r+1)]['Groundtruth'][idx]['y [m]']
            theta = dataset['Robot' + str(r+1)]['Groundtruth'][idx]['orientation [rad]']

            self.robots[r].xys[2*r:2*r+2] = np.array((x, y))
            self.robots[r].thetas[r] = theta
            self.robots[r].covs_i = np.identity(2*NUM_ROBOTS)*0.005
            self.robots[r].covs_d = np.identity(2*NUM_ROBOTS)*0.005
            self.robots[r].covs = np.identity(2*NUM_ROBOTS)*0.01

            self.robots[r].history['gt'].append({'time': time, 'x': x, 'y': y, 'theta': theta})
            self.robots[r].history['est'].append({'time': time, 'x': x, 'y': y, 'theta': theta, 'cov': np.identity(2)*0.01})

