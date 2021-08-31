import numpy as np
from math import pi, sqrt, atan2, hypot, sin, cos, pow, trunc
import matplotlib.pyplot as plt

import IPython


class DataGenerator():
    def __init__(self, landmarks, duration, robot_labels, starting_states, start_time, delta_t,
    velocity_noise = 0, angular_velocity_noise = 0, measurement_range_noise = 0, bearing_noise = 0,  communication_noise = 0,
    robot_fov = 2*pi/3):

        self.duration = duration
        self.robot_labels = robot_labels
        self.num_robots = len(robot_labels)
        self.start_time = start_time
        self.end_time = self.start_time + duration
        self.delta_t = delta_t

        # {robot label: [time, x_pos, y_pos, orientation], ... }
        self.starting_states = starting_states

        # angular width (radians) of robot's view in reference to x_axis
        # ex: robot_fov = 2pi/3 means 60 degrees above and below x-axis are within vision
        self.robot_fov = robot_fov

        self.velocity_noise = velocity_noise
        self.angular_velocity_noise = angular_velocity_noise
        self.measurement_range_noise = measurement_range_noise
        self.bearing_noise = bearing_noise
        self.communication_noise = communication_noise

        # landmarks = {landmark ID: [x,y]}
        self.landmark_map = landmarks
        for landmark_id in landmarks:
            if landmark_id < 5:
                raise Exception("Invalid landmark ID: landmark IDs must be bigger than 5.")

        self.time_arr = {'odometry': [[] for robot in robot_labels], 'measurement': [[] for robot in robot_labels], 'groundtruth': [[] for robot in robot_labels]}
        for data_type in self.time_arr:
            for i in range(self.num_robots):
                arr = np.arange(self.start_time, self.end_time, self.delta_t)
                self.time_arr[data_type][i] = arr

        self.groundtruth_data = [ [] for robot in range(self.num_robots)]
        self.odometry_data = [[] for robot in range(self.num_robots)]
        self.measurement_data = [[] for robot in range(self.num_robots)]

    # Inputs: None
    # Outputs: time_arr, groundtruth_data, self.odometry_data, measurement_data, self.dataset_data
    # Generates simulated data based of robots moving in a straight line until simulation ends
    def generate_straight_line_data(self, test, velocity=0.1, velocity_spread=0):

        move_step = (velocity + np.random.normal(0, velocity_spread)) * self.delta_t

        # Generate ground truth data
        for i, label in enumerate(self.robot_labels, 0):
            # append starting state
            self.groundtruth_data[i].append({'time' : self.starting_states[label][0], 'x_pos': self.starting_states[label][1], 'y_pos': self.starting_states[label][2], 'orientation' : self.starting_states[label][3]})
            for time_idx in range(1, len(self.time_arr['groundtruth'][i])):
                curr_x = self.groundtruth_data[i][time_idx-1]['x_pos']
                curr_y = self.groundtruth_data[i][time_idx-1]['y_pos']
                curr_orientation = self.groundtruth_data[i][time_idx-1]['orientation']

                next_orientation = curr_orientation

                next_x = curr_x
                next_y = curr_y

                next_x = curr_x + move_step
                next_y = curr_y

                self.groundtruth_data[i].append({'time' : self.time_arr['groundtruth'][i][time_idx], 'x_pos': next_x, 'y_pos': next_y, 'orientation': next_orientation})

        self.generate_odometry_data()
        self.generate_measurement_data()

        self.dataset_data = {'odometry': self.odometry_data, 'measurement': self.measurement_data, 'groundtruth': self.groundtruth_data}

        if (test):
            self.verify_generated_data()

        return self.time_arr, self.groundtruth_data, self.odometry_data, self.measurement_data, self.dataset_data

    # Inputs: None
    # Outputs: time_arr, groundtruth_data, self.odometry_data, measurement_data, self.dataset_data
    # Generates simulated of robots moving in a circle
    def generate_circular_data(self, test, velocity=0.1, velocity_spread=0):

        radian_step = pi/self.duration # can be hardcoded to be a certain amount of degrees (radians)
        # if it's a "perfect" circle, then velocity spread will be 0
        distance_step = (velocity + np.random.normal(0, velocity_spread))*self.delta_t

        # Generate ground truth data
        for i, label in enumerate(self.robot_labels, 0):
            # append starting state
            self.groundtruth_data[i].append({'time' : self.starting_states[label][0], 'x_pos': self.starting_states[label][1], 'y_pos': self.starting_states[label][2], 'orientation' : self.starting_states[label][3]})
            for time_idx in range(1, len(self.time_arr['groundtruth'][i])):

                curr_x = self.groundtruth_data[i][time_idx-1]['x_pos']
                curr_y = self.groundtruth_data[i][time_idx-1]['y_pos']
                curr_orientation = self.groundtruth_data[i][time_idx-1]['orientation']

                next_x = curr_x + distance_step*cos(radian_step + curr_orientation)
                next_y = curr_y + distance_step*sin(radian_step + curr_orientation)
                next_orientation = curr_orientation + radian_step

                # Converts all orientations to be between pi and pi
                if (next_orientation > pi or next_orientation < -pi):
                    next_orientation = self.converge_to_angle_range(next_orientation)


                self.groundtruth_data[i].append({'time' : self.time_arr['groundtruth'][i][time_idx], 'x_pos': next_x, 'y_pos': next_y, 'orientation': next_orientation})

        self.generate_odometry_data()
        self.generate_measurement_data()

        self.dataset_data = {'odometry': self.odometry_data, 'measurement': self.measurement_data, 'groundtruth': self.groundtruth_data}

        if (test):
            self.verify_generated_data()

        return self.time_arr, self.groundtruth_data, self.odometry_data, self.measurement_data, self.dataset_data

    # Inputs: test=True (run self.verify_generated_data), veloicty, velocity_spread (normally distributed)
    # Outputs: time_arr, groundtruth_data, self.odometry_data, measurement_data, self.dataset_data
    # Simulates random motion
    def generate_random_data(self, test, velocity, velocity_spread):

        # Generate ground truth data
        for i, label in enumerate(self.robot_labels, 0):
            # append starting state
            self.groundtruth_data[i].append({'time' : self.starting_states[label][0], 'x_pos': self.starting_states[label][1], 'y_pos': self.starting_states[label][2], 'orientation' : self.starting_states[label][3]})
            for time_idx in range(1, len(self.time_arr['groundtruth'][i])):

                curr_x = self.groundtruth_data[i][time_idx-1]['x_pos']
                curr_y = self.groundtruth_data[i][time_idx-1]['y_pos']
                curr_orientation = self.groundtruth_data[i][time_idx-1]['orientation']

                # rotate some random number of radians between pi/3 and -pi/3
                next_orientation = curr_orientation + np.random.uniform(-pi/3, pi/3)
                next_orientation = self.converge_to_angle_range(next_orientation)

                rand_dist = (velocity + np.random.normal(0, velocity_spread))*self.delta_t
                # np.random.normal(0,sqrt(0.25))
                next_x = curr_x + rand_dist*cos(next_orientation)
                next_y = curr_y + rand_dist*sin(next_orientation)

                self.groundtruth_data[i].append({'time' : self.time_arr['groundtruth'][i][time_idx], 'x_pos': next_x, 'y_pos': next_y, 'orientation': next_orientation})

        self.generate_odometry_data()
        self.generate_measurement_data()

        self.dataset_data = {'odometry': self.odometry_data, 'measurement': self.measurement_data, 'groundtruth': self.groundtruth_data}

        if (test):
            self.verify_generated_data()

        return self.time_arr, self.groundtruth_data, self.odometry_data, self.measurement_data, self.dataset_data

    # Inputs: None
    # Outputs: None
    # For given robot groundtruths, use velocity = distance/time to generate odometry data
    def generate_odometry_data(self):
        for i, label in enumerate(self.robot_labels, 0):
            # Initial robot state
            self.odometry_data[i].append({'time': self.start_time, 'velocity' : 0, 'angular velocity' : 0, 'orientation' : self.groundtruth_data[i][0]['orientation'],
             'delta_t' : 0})
            for time_idx in range(1, len(self.time_arr['odometry'][i])):
                loc1 = [self.groundtruth_data[i][time_idx-1]['x_pos'], self.groundtruth_data[i][time_idx-1]['y_pos']]
                loc2 = [self.groundtruth_data[i][time_idx]['x_pos'], self.groundtruth_data[i][time_idx]['y_pos']]
                t1 = self.groundtruth_data[i][time_idx-1]['time']
                t2 = self.groundtruth_data[i][time_idx]['time']

                velocity = self.calc_distance(loc1, loc2)/(t2-t1) + np.random.normal(loc=0.0,scale=self.velocity_noise)

                theta_1 = self.groundtruth_data[i][time_idx-1]['orientation']
                theta_2 = self.groundtruth_data[i][time_idx]['orientation']
                angular_velocity = (theta_2-theta_1)/(t2-t1) + np.random.normal(loc=0.0,scale=self.angular_velocity_noise)

                self.odometry_data[i].append({'time': self.time_arr['odometry'][i][time_idx], 'velocity' : velocity, 'angular velocity' : angular_velocity,
                'orientation' : self.groundtruth_data[i][time_idx]['orientation'], 'delta_t' : (t2-t1)})

    # Inputs: None
    # Outputs: None
    # for given robot groundtruths, robot_fov, and landmark map, generate all potential measurements at each time step
    def generate_measurement_data(self):
         # Generate Measurement Data
        for robot_idx, label in enumerate(self.robot_labels, 0):
            # NOTE - time_arr is used to emplace a one-to-one index-to-time correspondence
            # between measurement_data and time_arr['measurement'] to account for varing number of landmark observations
            time_arr = []
            for time_idx in range(0, len(self.time_arr['measurement'][robot_idx])):

                time = self.time_arr['measurement'][robot_idx][time_idx]
                robot_loc_x = self.groundtruth_data[robot_idx][time_idx]['x_pos']
                robot_loc_y = self.groundtruth_data[robot_idx][time_idx]['y_pos']
                robot_orientation = self.groundtruth_data[robot_idx][time_idx]['orientation']
                robot_loc = [robot_loc_x, robot_loc_y]

                # Relative (other robots) observations
                other_robot_locs  = {label : [self.groundtruth_data[i][time_idx]['x_pos'], self.groundtruth_data[i][time_idx]['y_pos']] for i, label in enumerate(self.robot_labels, 0) if i != robot_idx }
                for robotID, other_robot_loc in other_robot_locs.items():

                    if (self.within_vision(self.robot_fov, robot_loc, other_robot_loc, robot_orientation)):
                        measurement_range = self.calc_distance(robot_loc, other_robot_loc) + np.random.normal(loc=0.0,scale=self.measurement_range_noise)
                        bearing = atan2((other_robot_loc[1]-robot_loc[1]), (other_robot_loc[0]-robot_loc[0])) + np.random.normal(loc=0.0,scale=self.bearing_noise) # global coordinates
                        bearing = bearing - robot_orientation # NOTE - local = global - orientation, all between 0 and 2*pi
                        bearing = self.converge_to_angle_range(bearing)
                        self.measurement_data[robot_idx].append({'time': time, 'subject_ID': robotID, 'measurment_range':measurement_range, 'bearing':bearing})
                        time_arr.append(time) # we expect repeats/skipped times in time array

                # Landmark (absolute) observations
                for landmarkID, landmark_loc in self.landmark_map.items():

                    if (self.within_vision(self.robot_fov, robot_loc, landmark_loc, robot_orientation)):
                        measurement_range = self.calc_distance(robot_loc, landmark_loc) + np.random.normal(loc=0.0,scale=self.measurement_range_noise)
                        bearing = atan2((landmark_loc[1]-robot_loc[1]), (landmark_loc[0]-robot_loc[0])) + np.random.normal(loc=0.0,scale=self.bearing_noise) # global coordinates
                        bearing = bearing - robot_orientation # NOTE - local = global - orientation, all between 0 and 2*pi
                        bearing = self.converge_to_angle_range(bearing)
                        self.measurement_data[robot_idx].append({'time': time, 'subject_ID': landmarkID, 'measurment_range':measurement_range, 'bearing':bearing})
                        time_arr.append(time) # we expect repeats/skipped times in time array

            self.time_arr['measurement'][robot_idx] = time_arr

    # Inputs: None
    # Outputs: Verifies Generated Data
    def verify_generated_data(self):
        print('******Verifying Data******')

        # Verify odometry data & measurement data
        odo_test_arr = [[] for robot in self.robot_labels]
        meas_test_arr = [[] for robot in self.robot_labels]
        for i, robot in enumerate(self.robot_labels):
            groundtruth_list = self.groundtruth_data[i]
            odometry_list = self.odometry_data[i]
            measurement_list = self.measurement_data[i]
            for g_idx in range(1, len(groundtruth_list)):

                t1 = groundtruth_list[g_idx-1]['time']
                x1 = groundtruth_list[g_idx-1]['x_pos']
                y1 = groundtruth_list[g_idx-1]['y_pos']
                o1 = groundtruth_list[g_idx-1]['orientation']

                t2 = groundtruth_list[g_idx]['time']
                x2 = groundtruth_list[g_idx]['x_pos']
                y2 = groundtruth_list[g_idx]['y_pos']
                o2 = groundtruth_list[g_idx]['orientation']

                actual_delta_t = (t2-t1)
                actual_v = self.calc_distance([x1,y1], [x2, y2])/actual_delta_t
                actual_w = (o2-o1)/actual_delta_t
                actual_o = o2

                generated_v = odometry_list[g_idx]['velocity']
                generated_w = odometry_list[g_idx]['angular velocity']
                generated_o = odometry_list[g_idx]['orientation']
                generated_delta_t = odometry_list[g_idx]['delta_t']

                v_diff = actual_v - generated_v # should be randomly distributed between user noise
                w_diff = actual_w - generated_w # should be randomly distributed between user noise
                o_diff = actual_o - generated_o # should be 0
                delta_t_diff = actual_delta_t - generated_delta_t # should be 0

                odo_test_arr[i].append({'v_diff': v_diff, 'w_diff': w_diff, 'o_diff': o_diff, 'delta_t_diff': delta_t_diff, 'time': t2})

                curr_time = t2

                measurements = [measurement for measurement in measurement_list if measurement['time']==curr_time]
                curr_loc = [x2,y2]
                for measurement in measurements:
                    # Verify a landmark measurement
                    if (measurement['subject_ID'] > 5):
                        landmark_loc = self.landmark_map[measurement['subject_ID']]

                        actual_meas = self.calc_distance(curr_loc, landmark_loc)
                        generated_measurement = measurement['measurment_range']

                        actual_bearing = self.converge_to_angle_range(atan2(landmark_loc[1]- y2, landmark_loc[0]-x2) - actual_o) # global - orientation = local, -pi to pi
                        generated_bearing = measurement['bearing']

                        meas_diff = actual_meas - generated_measurement
                        bearing_diff = abs(actual_bearing) - abs(generated_bearing)

                        # double check for edge cases where both are approximately pi (one may be (-), the other may be positive
                        if (abs(bearing_diff) > 1):
                            print(actual_bearing, generated_bearing)

                        meas_test_arr[i].append({'meas_diff': meas_diff, 'bearing_diff': bearing_diff, 'time': curr_time})
                    else:
                        other_robot_loc_x = self.groundtruth_data[self.robot_labels.index(measurement['subject_ID'])][g_idx]['x_pos']
                        other_robot_loc_y = self.groundtruth_data[self.robot_labels.index(measurement['subject_ID'])][g_idx]['y_pos']

                        other_robot_loc = [other_robot_loc_x, other_robot_loc_y]

                        actual_meas = self.calc_distance(curr_loc, other_robot_loc)
                        generated_measurement = measurement['measurment_range']

                        actual_bearing = self.converge_to_angle_range(atan2(other_robot_loc[1]- y2, other_robot_loc[0]-x2) - actual_o) # global - orientation = local, -pi to pi
                        generated_bearing = measurement['bearing']

                        meas_diff = actual_meas - generated_measurement
                        bearing_diff = abs(actual_bearing) - abs(generated_bearing)

                        # double check for edge cases where both are approximately pi (one may be (-), the other may be positive
                        if (abs(bearing_diff) > 1):
                            print(actual_bearing, generated_bearing)

                        meas_test_arr[i].append({'meas_diff': meas_diff, 'bearing_diff': bearing_diff, 'time': curr_time})

        fig, ax_arr = plt.subplots(8)
        plt.xlabel('t [s]')
        # plt.title('Odometry & Measurement Data Verification', position=(0.5,7))
        ax_arr[0].set_xlabel('v_diff [m/s]')
        ax_arr[1].set_xlabel('w_diff [r/s]')
        ax_arr[2].set_xlabel('o_diff [r]')
        ax_arr[3].set_xlabel('delta_t_diff [s]')
        ax_arr[4].set_xlabel('meas_diff [m]')
        ax_arr[5].set_xlabel('bearing_diff [r]')

        ax_arr[6].set_xlabel('t')
        ax_arr[7].set_xlabel('t')

        ax_arr[6].set_ylabel('meas_diff [m]')
        ax_arr[7].set_ylabel('bearing_diff [r]')

        labels = ['robot{}'.format(i) for i, lablel in enumerate(self.robot_labels,1,)]

        for diff_list in odo_test_arr:
            v_diff_arr = [d['v_diff'] for d in diff_list]
            w_diff_arr = [d['w_diff'] for d in diff_list]
            o_diff_arr = [d['o_diff'] for d in diff_list]
            delta_t_diff_arr = [d['delta_t_diff'] for d in diff_list]
            # time_arr = [d['time'] for d in diff_list]

            n_bins = 60

            ax_arr[0].hist(v_diff_arr, n_bins)
            ax_arr[1].hist(w_diff_arr, n_bins)
            ax_arr[2].hist(o_diff_arr, n_bins)
            ax_arr[3].hist(delta_t_diff_arr, n_bins)

        for diff_list in meas_test_arr:
            meas_diff_arr = [d['meas_diff'] for d in diff_list]
            bearing_diff_arr = [d['bearing_diff'] for d in diff_list]
            meas_time_arr = [d['time'] for d in diff_list]

            ax_arr[4].hist(meas_diff_arr, n_bins)
            ax_arr[5].hist(bearing_diff_arr, n_bins)

            ax_arr[6].scatter(meas_time_arr, meas_diff_arr)
            ax_arr[7].scatter(meas_time_arr, bearing_diff_arr)

        fig.legend(labels, prop={'size': 12})
        plt.subplots_adjust(left=.07, bottom=.08, right=.94, top=.89, wspace=.2, hspace=.75)
        plt.show()
        # plt.close('all')

        print('******Verification Complete******')

    # Input: loc=[x,y]
    # Output: boolean if the point [x,y] is within the circular map of radius r centered @ origin
    def within_map(self, loc, r):
        return hypot(loc[0], loc[1]) < r

    # Input: loc1=[x1,y1], loc2=[x2,y2]
    # Output: Euclidean distance between two points
    def calc_distance(self, loc1, loc2):

        x1 = loc1[0]
        y1 = loc1[1]

        x2 = loc2[0]
        y2 = loc2[1]

        distance = sqrt((x1-x2)**2 + (y1-y2)**2)
        return distance

    # Input: robot_loc, landmark_loc, [x,y]
    # Output: boolean, a given point is with the field of view (sector of a circle) of a robot
    def within_vision(self, robot_fov, robot_loc, landmark_loc, robot_orientation=0):
        # between pi and -pi with respect to LOCAL FRAME
        bearing = atan2((landmark_loc[1]-robot_loc[1]),(landmark_loc[0]-robot_loc[0])) - robot_orientation
        if (robot_fov == 2*pi):
            return True
        return abs(bearing) <= robot_fov/2

    # get angle between -pi and pi
    def converge_to_angle_range(self, theta):
        while (theta < -pi):
            theta += 2*pi
        while (theta > pi):
            theta -= 2*pi
        return theta

# NOTE - Data formatting
'''
Groundtruth format
ORIENTATION DEFINED WITH RESPECT TO GLOBAL FRAME OF REFERENCE
{
    'time': 1536012433.141,
    'x_pos': -0.690963,
    'y_pos': -1.685781,
    'orientation': 0.03251611228318508
}

Odometry Format
{
    'time': 1536012462.22,
    'velocity': 0.0,
    'angular velocity': -0.2,
    'orientation': -0.4324032103332002,
    'delta_t': 0.09999990463256836
}


Measurement Data Format
subject ID = Landmark ID/RobotID
measurement_range = distance between robot and landmark
bearing = angle betweeen robot and landmark, defined with respect to ROBOT'S FRAME OF REFERENCE
{
    'time': 1536012482.625,
    'subject_ID': 120,
    'measurment_range': 2.8757099026371695,
    'bearing': 0.29890824572449504
}
'''
