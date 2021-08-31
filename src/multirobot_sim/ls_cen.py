from numpy import matrix
from numpy import random
from numpy import linalg
from math import cos, sin, atan2, sqrt

from robotteam import RobotTeam

import sim_env






class RobotTeamLocalCen(RobotTeam):
	""" Local state with centralized algorithm """

	def __init__(self , initial_s):

		RobotTeam.__init__(self, initial_s)
		self.th_sigma = self.sigma.copy()

	def get_th_sigma(self):
		return self.th_sigma.copy()

	def prop_update(self):

		for idx in range(5):
			dt = sim_env.dt
			ii = 2*idx

			# select valid motion input
			[v, a_v] = [random.uniform(-sim_env.max_v, sim_env.max_v), random.uniform(-sim_env.max_oemga, sim_env.max_oemga)]
			v_star = v + random.normal(0, sqrt(sim_env.var_u_v))
			pre_update_position = [self.position[ii] + cos(self.theta[idx])*v_star*dt, self.position[ii+1] + sin(self.theta[idx])*v_star*dt]


			while(not sim_env.inRange(pre_update_position, sim_env.origin)):

				[v, a_v] = [random.uniform(-sim_env.max_v, sim_env.max_v), random.uniform(-sim_env.max_oemga, sim_env.max_oemga)]
				v_star = v + random.normal(0, sqrt(sim_env.var_u_v))
				pre_update_position = [self.position[ii] + cos(self.theta[idx])*v_star*dt, self.position[ii+1] + sin(self.theta[idx])*v_star*dt]

			# real position update
			self.position[ii,0] += cos(self.theta[idx])*v_star*dt
			self.position[ii+1,0] += sin(self.theta[idx])*v_star*dt
			self.theta[idx] += a_v*dt

			# estimation update
			self.s[ii,0] += cos(self.theta[idx])*v*dt
			self.s[ii+1,0] += sin(self.theta[idx])*v*dt

			# covariance update
			self.sigma[ii:ii+2, ii:ii+2] += dt*dt*sim_env.rot_mtx(self.theta[idx])*matrix([[sim_env.var_u_v, 0],[0, sim_env.var_u_theta]])*sim_env.rot_mtx(self.theta[idx]).getT()
			self.th_sigma[ii:ii+2, ii:ii+2] += dt*dt*sim_env.var_u_v*matrix([[1, 0],[0, 1]])
			

	def ablt_obsv(self, idx, obs_value, landmark):

		i = 2*idx

		H_i = matrix([[0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0]], dtype=float)
		H_i[0, i] = -1
		H_i[1, i+1] = -1

		H = sim_env.rot_mtx(self.theta[idx]).getT()*H_i
		
		dis = obs_value[0]
		phi = obs_value[1]

		hat_z = sim_env.rot_mtx(self.theta[idx]).getT() * (landmark.position + H_i*self.s)
		z = matrix([dis*cos(phi), dis*sin(phi)]).getT()

		# algorithmic update
		sigma_z = sim_env.rot_mtx(phi) * matrix([[sim_env.var_dis, 0],[0, dis*dis*sim_env.var_phi]]) * sim_env.rot_mtx(phi).getT() 
		sigma_invention = H * self.sigma * H.getT()  + sigma_z
		kalman_gain = self.sigma * H.getT() * sigma_invention.getI()

		self.s = self.s + kalman_gain*(z - hat_z)
		self.sigma = self.sigma - kalman_gain*H*self.sigma


		# analytical update
		max_lambda = max(sim_env.var_dis, sim_env.d_max*sim_env.d_max*sim_env.var_phi)
		sigma_th_z =  max_lambda * sim_env.i_mtx_2.copy() 
		sigma_th_invention = H_i * self.th_sigma * H_i.getT() + sigma_th_z
		kalman_th_gain = self.th_sigma*H_i.getT()*sigma_th_invention.getI()

		self.th_sigma = self.th_sigma - kalman_th_gain*H_i*self.th_sigma		




	def rela_obsv(self, idx, obs_idx, obs_value):
		i = 2*idx
		j = 2*obs_idx


		H_ij = matrix([[0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0]], dtype=float)
		H_ij[0, i] = -1
		H_ij[1, i+1] = -1
		H_ij[0, j] = 1
		H_ij[1, j+1] = 1


		H = sim_env.rot_mtx(self.theta[idx]).getT()*H_ij
		
		dis = obs_value[0]
		phi = obs_value[1]

		hat_z = sim_env.rot_mtx(self.theta[idx]).getT() * H_ij * self.s
		z = matrix([dis*cos(phi), dis*sin(phi)]).getT()

		# algorithmic update
		sigma_z = sim_env.rot_mtx(phi) * matrix([[sim_env.var_dis, 0],[0, dis*dis*sim_env.var_phi]]) * sim_env.rot_mtx(phi).getT() 
		sigma_invention = H * self.sigma * H.getT()  + sigma_z
		kalman_gain = self.sigma * H.getT() * sigma_invention.getI()

		self.s = self.s + kalman_gain*(z - hat_z)
		self.sigma = self.sigma - kalman_gain*H*self.sigma


		# analytical update
		max_lambda = max(sim_env.var_dis, sim_env.d_max*sim_env.d_max*sim_env.var_phi)
		sigma_th_z =  max_lambda * sim_env.i_mtx_2.copy() 
		sigma_th_invention = H_ij * self.th_sigma * H_ij.getT() + sigma_th_z
		kalman_th_gain = self.th_sigma*H_ij.getT()*sigma_th_invention.getI()

		self.th_sigma = self.th_sigma - kalman_th_gain*H_ij*self.th_sigma	





