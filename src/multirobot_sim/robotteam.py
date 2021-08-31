from numpy import matrix
from numpy import random

from math import cos, sin, atan2, sqrt
import sim_env


class RobotTeam():
	""" The class for local state algorithm """

	def __init__(self , initial_s):

		self.s = initial_s.copy()
		self.sigma = sim_env.i_mtx_10.copy()*0.01
		self.position = initial_s.copy()
		self.theta = [0.0,0,0,0,0]


	# access functions
	def get_s(self):
		return self.s.copy()

	def get_sigma(self):
		return self.sigma.copy()

	def error(self):
		err = 0
		for idx in range(5):
			ii = 2*idx
			err += pow(self.s[ii,0] - self.position[ii,0],2) + pow(self.s[ii+1,0] - self.position[ii+1,0],2)
		
		return sqrt(err*0.2)


	#

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


