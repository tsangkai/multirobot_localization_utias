from numpy import matrix
from numpy import random
from numpy import linalg
from math import cos, sin, atan2, sqrt

import sim_env





class RobotLocalSCI:
	""" Global state with CI algorithm """

	N = sim_env.num_robot

	def __init__(self, index , initial_s):
		self.index = index

		self.s = initial_s.copy()

		# sigma = sigma_d + sigma_i
		self.sigma_d = sim_env.i_mtx_10.copy()*0.005
		self.sigma_i = sim_env.i_mtx_10.copy()*0.005

		self.position = [initial_s[2*index,0], initial_s[2*index+1,0]]
		self.theta = 0


	# access functions
	def get_s(self):
		return self.s.copy()

	def get_sigma(self):
		return (self.sigma_i + self.sigma_d).copy()


	# localization algorithm

	def prop_update(self):
		dt = sim_env.dt

		# select valid motion input
		[v, a_v] = [random.uniform(-sim_env.max_v, sim_env.max_v), random.uniform(-sim_env.max_oemga, sim_env.max_oemga)]
		v_star = v + random.normal(0, sqrt(sim_env.var_u_v))
		pre_update_position = [self.position[0] + cos(self.theta)*v_star*dt, self.position[1] + sin(self.theta)*v_star*dt]


		while(not sim_env.inRange(pre_update_position, sim_env.origin)):

			[v, a_v] = [random.uniform(-sim_env.max_v, sim_env.max_v), random.uniform(-sim_env.max_oemga, sim_env.max_oemga)]
			v_star = v + random.normal(0, sqrt(sim_env.var_u_v))
			pre_update_position = [self.position[0] + cos(self.theta)*v_star*dt, self.position[1] + sin(self.theta)*v_star*dt]


		# real position update
		self.position[0] += cos(self.theta)*v_star*dt
		self.position[1] += sin(self.theta)*v_star*dt

		self.theta += a_v*dt

		# estimation update
		ii = 2*self.index

		self.s[ii,0] = self.s[ii,0] + cos(self.theta)*v*dt
		self.s[ii+1,0] = self.s[ii+1,0] + sin(self.theta)*v*dt

		# covariance update
		for j in range(RobotLocalSCI.N):
			jj = 2*j

			if j==self.index:
				total_sigma = self.sigma_i[jj:jj+2, jj:jj+2] + self.sigma_d[jj:jj+2, jj:jj+2]
				self.sigma_i[jj:jj+2, jj:jj+2] += dt*dt*sim_env.rot_mtx(self.theta)*matrix([[sim_env.var_u_v, 0],[0, sim_env.var_u_theta]])*sim_env.rot_mtx(self.theta).getT()
				total_sigma += dt*dt*sim_env.rot_mtx(self.theta)*matrix([[sim_env.var_u_v, 0],[0, sim_env.var_u_theta]])*sim_env.rot_mtx(self.theta).getT()
				self.sigma_d[jj:jj+2, jj:jj+2] = total_sigma - self.sigma_i[jj:jj+2, jj:jj+2]

			else:
				total_sigma = self.sigma_i[jj:jj+2, jj:jj+2] + self.sigma_d[jj:jj+2, jj:jj+2]
				self.sigma_i[jj:jj+2, jj:jj+2] += dt*dt*sim_env.var_v*sim_env.i_mtx_2.copy()
				total_sigma += dt*dt*sim_env.var_v*sim_env.i_mtx_2.copy()
				self.sigma_d[jj:jj+2, jj:jj+2] = total_sigma - self.sigma_i[jj:jj+2, jj:jj+2]



	def ablt_obsv(self, obs_value, landmark):

		ii = 2*self.index
		total_sigma = self.sigma_i + self.sigma_d


		H_i = matrix([[0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0]], dtype=float)
		H_i[0, ii] = -1
		H_i[1, ii+1] = -1


		H = sim_env.rot_mtx(self.theta).getT()*H_i
		
		dis = obs_value[0]
		phi = obs_value[1]

		hat_z = sim_env.rot_mtx(self.theta).getT() * (landmark.position + H_i*self.s)
		z = matrix([dis*cos(phi), dis*sin(phi)]).getT()

		sigma_z = sim_env.rot_mtx(phi) * matrix([[sim_env.var_dis, 0],[0, dis*dis*sim_env.var_phi]]) * sim_env.rot_mtx(phi).getT() 
		sigma_invention = H * total_sigma * H.getT()  + sigma_z
		kalman_gain = total_sigma * H.getT()*sigma_invention.getI()

		self.s = self.s + kalman_gain*(z - hat_z)

		total_sigma = total_sigma - kalman_gain*H*total_sigma
		self.sigma_i = (sim_env.i_mtx_10.copy() - kalman_gain*H) * self.sigma_i * (sim_env.i_mtx_10.copy() - kalman_gain*H).getT() + kalman_gain * sigma_z * kalman_gain.getT()
		self.sigma_d = total_sigma - self.sigma_i


	def rela_obsv(self, obs_idx, obs_value):
		ii = 2*self.index
		jj = 2*obs_idx

		H_ij = matrix([[0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0]], dtype=float)
		H_ij[0, ii] = -1
		H_ij[1, ii+1] = -1
		H_ij[0, jj] = 1
		H_ij[1, jj+1] = 1


		H = sim_env.rot_mtx(self.theta).getT()*H_ij

		dis = obs_value[0]
		phi = obs_value[1]

		hat_z = H * self.s
		z = matrix([dis*cos(phi), dis*sin(phi)]).getT()

		sigma_z = sim_env.rot_mtx(phi) * matrix([[sim_env.var_dis, 0],[0, dis*dis*sim_env.var_phi]]) * sim_env.rot_mtx(phi).getT() 

		e =  0.83#  (iii+1)*0.01

		p_1 = (1/e) * self.sigma_d + self.sigma_i
		p_2 = (1/(1-e)) * H * (self.sigma_i + self.sigma_d) * H.getT()  + sigma_z
		sigma_invention = H * p_1 * H.getT()  + p_2
		kalman_gain = p_1 *H.getT()*sigma_invention.getI()

		self.s = self.s + kalman_gain*(z - hat_z)
		total_sigma = (sim_env.i_mtx_10.copy()-kalman_gain*H) * p_1
		self.sigma_i = (sim_env.i_mtx_10.copy()-kalman_gain*H) * self.sigma_i * (sim_env.i_mtx_10.copy()-kalman_gain*H).getT() + kalman_gain * sigma_z * kalman_gain.getT()
		self.sigma_d = total_sigma - self.sigma_i



	def comm(self, comm_robot_s, comm_robot_sigma_i, comm_robot_sigma_d):

		e =  0.93#  (iii+1)*0.01


		p_1 = (1/e) * self.sigma_d + self.sigma_i
		p_2 = (1/(1-e)) * comm_robot_sigma_d + comm_robot_sigma_i

		kalman_gain = p_1 * (p_1+p_2).getI()
		self.s = self.s + kalman_gain * (comm_robot_s - self.s) 

		total_sigma = (sim_env.i_mtx_10.copy() - kalman_gain) * p_1
		self.sigma_i = (sim_env.i_mtx_10.copy() - kalman_gain) * self.sigma_i  * (sim_env.i_mtx_10.copy() - kalman_gain).getT() + kalman_gain * comm_robot_sigma_i * kalman_gain.getT()
		self.sigma_d = total_sigma - self.sigma_i




