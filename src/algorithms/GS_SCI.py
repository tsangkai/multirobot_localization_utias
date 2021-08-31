import numpy as np
from numpy import matrix
from math import cos, sin, atan2, sqrt
from algorithms.EKF import ekf_algo_framework

def rot_mtx(theta):
    return matrix([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

class GS_SCI(ekf_algo_framework):
    def __init__(self, algo_name):
        self.algo_name = algo_name

    def state_variance_init(self, num_robots):
        sigma_d = 0.04*np.matrix(np.identity(2*num_robots), dtype = float)
        sigma_i = 0.04*np.matrix(np.identity(2*num_robots), dtype = float)
        state_variance = [sigma_d, sigma_i]
        return state_variance

    def calculate_trace_state_variance(self, robot_data):
        [s, orinetations, sigma_s, index] = robot_data
        i = 2*index
        [sigma_d, sigma_i] = sigma_s
        total_sigma = sigma_d + sigma_i
        trace_state_var = np.trace(total_sigma[i:i+2, i:i+2])/2
        return np.sqrt(np.abs(np.trace(total_sigma/2)))


    def propagation_update(self, robot_data, sensor_data):
        [s, orinetations, sigma_s, index] = robot_data
        [measurement_data, sensor_covariance] = sensor_data
        sigma_odo = sensor_covariance

        var_v = 0.1

        num_robots = int(len(s)/2)
        delta_t = measurement_data[0]
        v = measurement_data[1]
        orinetations[index] = measurement_data[2]
        self_theta = orinetations[index]

        Q = sigma_odo
        W = delta_t*rot_mtx(self_theta)

        i = 2*index

        s[i,0] = s[i,0] + cos(self_theta)*v*delta_t   #x
        s[i+1,0] = s[i+1,0] + sin(self_theta)*v*delta_t #y

        [sigma_d, sigma_i] = sigma_s

        Q = sigma_odo
        W = delta_t*rot_mtx(self_theta)*np.matrix([[1,0],[0,v]])

        for j in range(num_robots):
            jj = 2*j

            if j ==index:
                total_sigma = sigma_i[jj:jj+2, jj:jj+2] + sigma_d[jj:jj+2, jj:jj+2]
                sigma_i[jj:jj+2, jj:jj+2] += W*Q*W.getT()
                total_sigma += W*Q*W.getT()
                sigma_d[jj:jj+2, jj:jj+2] = total_sigma - sigma_i[jj:jj+2, jj:jj+2]

            else:

                total_sigma = sigma_i[jj:jj+2, jj:jj+2] + sigma_d[jj:jj+2, jj:jj+2] #repeated
                sigma_i[jj:jj+2, jj:jj+2] += delta_t*delta_t*var_v*np.identity(2)   #unknow other robot's movement
                total_sigma += delta_t*delta_t*var_v*np.identity(2)
                sigma_d[jj:jj+2, jj:jj+2] = total_sigma - sigma_i[jj:jj+2, jj:jj+2]


        sigma_s = [sigma_d, sigma_i]

        return [s, orinetations, sigma_s]



    def absolute_obser_update(self, robot_data, sensor_data):
        [s, orinetations, sigma_s, index] = robot_data
        [measurement_data, sensor_covariance] = sensor_data

        sigma_ob = sensor_covariance
        num_robots = int(len(s)/2)
        self_theta = orinetations[index]

        landmark_loc = measurement_data[0]
        dis = measurement_data[1]
        phi = measurement_data[2]

        i = 2*index
        H_i  = np.matrix(np.zeros((2,2*num_robots)), dtype = float)
        H_i[0, i] = -1
        H_i[1, i+1] = -1
        H = rot_mtx(self_theta).getT()*H_i

        delta_x = landmark_loc[0] - s.item(i,0)
        delta_y = landmark_loc[1] - s.item(i+1,0)
        z_hat = rot_mtx(self_theta).getT()*(np.matrix([delta_x, delta_y]).getT())
        z = matrix([dis*cos(phi), dis*sin(phi)]).getT()

        [sigma_d, sigma_i] = sigma_s
        total_sigma = sigma_i + sigma_d

        sigma_ob[1,1] = sigma_ob[1,1]*dis*dis
        sigma_z = rot_mtx(phi) * sigma_ob * rot_mtx(phi).getT()


        sigma_invention = H * total_sigma * H.getT()  + sigma_z
        kalman_gain = total_sigma*H.getT()*sigma_invention.getI()

        s = s + kalman_gain*(z - z_hat)

        total_sigma = total_sigma - kalman_gain*H*total_sigma
        i_mtx = np.identity(num_robots*2)
        sigma_i = (i_mtx - kalman_gain*H) * sigma_i * (i_mtx.copy() - kalman_gain*H).getT() + kalman_gain * sigma_z * kalman_gain.getT()
        sigma_d = total_sigma - sigma_i

        sigma_s = [sigma_d, sigma_i]

        return [s, orinetations, sigma_s]

    def relative_obser_update(self, robot_data, sensor_data):
        #when robot i observes robot j

        [s, orinetations, sigma_s, index] = robot_data
        [measurement_data, sensor_covariance] = sensor_data
        sigma_ob = sensor_covariance

        num_robots = int(len(s)/2)
        self_theta = orinetations[index]

        i = index * 2
        obser_index = measurement_data[0]
        dis = measurement_data[1]
        phi = measurement_data[2]
        j = obser_index * 2

        z = matrix([dis*cos(phi), dis*sin(phi)]).getT()

        H_ij  = np.zeros((2,2*num_robots))
        H_ij[0, i] = -1
        H_ij[1, i+1] = -1
        H_ij[0, j] = 1
        H_ij[1, j+1] = 1
        H = rot_mtx(self_theta).getT()*H_ij

        #z_hat = H * s
        #sigma_ob[1,1] = sigma_ob[1,1]*meas_range*meas_range

        delta_x = s.item(j,0) - s.item(i,0)
        delta_y = s.item(j+1,0) - s.item(i+1,0)
        z_hat = rot_mtx(self_theta).getT()*(np.matrix([delta_x, delta_y]).getT())

        sigma_ob[1,1] = sigma_ob[1,1]*dis*dis
        sigma_z = rot_mtx(phi)*sigma_ob*rot_mtx(phi).getT()

        [sigma_d, sigma_i] = sigma_s
        total_sigma = sigma_i + sigma_d

        e = 0.83

        p_1 = 1/e * sigma_d[i:i+2, i:i+2] + sigma_i[i:i+2, i:i+2]
        p_2 = 1/(1-e) *sigma_d[j:j+2, j:j+2] + sigma_i[j:j+2, j:j+2]
        kalman_gain = p_1*(p_1+p_2).getI()

        s[i:i+2] = kalman_gain*(s[j:j+2]-s[i:i+2])
        i_mtx = np.identity(num_robots*2)
        total_sigma[i:i+2, i:i+2] = (-kalman_gain) * p_1
        sigma_i[i:i+2, i:i+2]  = (np.identity(2) - kalman_gain) * sigma_i[i:i+2, i:i+2] * (np.identity(2) - kalman_gain).getT() + kalman_gain * sigma_i[j:j+2, j:j+2] * kalman_gain.getT()
        sigma_d[i:i+2, i:i+2]  = total_sigma[i:i+2, i:i+2]  - sigma_i[i:i+2, i:i+2]

        '''
        p_1 = (1/e) * sigma_d + sigma_i
        p_2 = (1/(1-e)) * H * (sigma_i + sigma_d) * H.getT()  + sigma_z

        sigma_invention = H * p_1 * H.getT()  + p_2
        kalman_gain = p_1 *H.getT()*sigma_invention.getI()

        s = s + kalman_gain*(z - z_hat)
        i_mtx = np.identity(num_robots*2)


        total_sigma = (i_mtx.copy()-kalman_gain*H) * p_1
        sigma_i = (i_mtx - kalman_gain*H) * sigma_i * (i_mtx.copy() - kalman_gain*H).getT() + kalman_gain * sigma_z * kalman_gain.getT()
        sigma_d = total_sigma - sigma_i
        '''



        sigma_s = [sigma_d, sigma_i]

        return [s, orinetations, sigma_s]

    def communication(self, robot_data, sensor_data):

        [s, orinetations, sigma_s, index] = robot_data
        [comm_data, comm_variance] = sensor_data
        [comm_robot_s, orinetations, comm_robot_sigma_s]=comm_data
        num_robots = int(len(s)/2)

        e =  0.8 # (iii+1)*0.01

        [sigma_d, sigma_i] = sigma_s
        [comm_robot_sigma_d, comm_robot_sigma_i] = comm_robot_sigma_s

        p_1 = (1/e) * sigma_d + sigma_i
        p_2 = (1/(1-e)) * comm_robot_sigma_d + comm_robot_sigma_i

        kalman_gain = p_1 * (p_1+p_2).getI()
        s = s + kalman_gain * (comm_robot_s - s)

        i_mtx = np.identity(num_robots*2)
        total_sigma = (i_mtx.copy() - kalman_gain) * p_1
        sigma_i = (i_mtx - kalman_gain) * sigma_i * (i_mtx.copy() - kalman_gain).getT() + kalman_gain * comm_robot_sigma_i * kalman_gain.getT()
        sigma_d = total_sigma - sigma_i
        sigma_s = [sigma_d, sigma_i]

        return [s, orinetations, sigma_s]


