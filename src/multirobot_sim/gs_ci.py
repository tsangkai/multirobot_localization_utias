from numpy import matrix
from numpy import random
from numpy import linalg
from math import cos, sin, atan2, sqrt

import sim_env





class RobotLocalCI():
	""" Global state with CI algorithm """

	N = sim_env.num_robot

	def __init__(self, index , initial_s):

		self.index = index

		self.s = initial_s.copy()
		self.sigma = sim_env.i_mtx_10.copy()*0.01
		self.th_sigma = sim_env.i_mtx_10.copy()*0.01

		self.position = [initial_s[2*index,0], initial_s[2*index+1,0]]
		self.theta = 0.0


	# access functions
	def get_s(self):
		return self.s.copy()

	def get_sigma(self):
		return self.sigma.copy()


	def get_th_sigma(self):
		return self.th_sigma.copy()

	#

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
		for j in range(RobotLocalCI.N):
			jj = 2*j

			if j==self.index:
				self.sigma[jj:jj+2, jj:jj+2] += dt*dt*sim_env.rot_mtx(self.theta)*matrix([[sim_env.var_u_v, 0],[0, sim_env.var_u_theta]])*sim_env.rot_mtx(self.theta).getT()
				self.th_sigma[jj:jj+2, jj:jj+2] += dt*dt*sim_env.var_u_v*matrix([[1, 0],[0, 1]])

			else:
				self.sigma[jj:jj+2, jj:jj+2] +=  dt*dt*sim_env.var_v*sim_env.i_mtx_2.copy()
				self.th_sigma[jj:jj+2, jj:jj+2] += dt*dt*sim_env.var_v*sim_env.i_mtx_2.copy()



	def ablt_obsv(self, obs_value, landmark):
		i = 2*self.index


		H_i = matrix([[0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0]], dtype=float)
		H_i[0, i] = -1
		H_i[1, i+1] = -1


		H = sim_env.rot_mtx(self.theta).getT()*H_i
		
		dis = obs_value[0]
		phi = obs_value[1]

		hat_z = sim_env.rot_mtx(self.theta).getT() * (landmark.position + H_i*self.s)
		z = matrix([dis*cos(phi), dis*sin(phi)]).getT()

		# algorithmic update
		sigma_z = sim_env.rot_mtx(phi) * matrix([[sim_env.var_dis, 0],[0, dis*dis*sim_env.var_phi]]) * sim_env.rot_mtx(phi).getT() 
		sigma_invention = H * self.sigma * H.getT()  + sigma_z
		kalman_gain = self.sigma*H.getT()*sigma_invention.getI()

		self.s = self.s + kalman_gain*(z - hat_z)
		self.sigma = self.sigma - kalman_gain*H*self.sigma


		# analytical update
		sigma_th_z =  max(sim_env.var_dis, sim_env.d_max*sim_env.d_max*sim_env.var_phi)* sim_env.i_mtx_2.copy() 
		self.th_sigma = (self.th_sigma.getI() + H_i.getT() * sigma_th_z.getI() * H_i).getI()



	def rela_obsv(self, obs_idx, obs_value):
		i = 2*self.index
		j = 2*obs_idx


		H_ij = matrix([[0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0]], dtype=float)
		H_ij[0, i] = -1
		H_ij[1, i+1] = -1
		H_ij[0, j] = 1
		H_ij[1, j+1] = 1


		H = sim_env.rot_mtx(self.theta).getT()*H_ij

		dis = obs_value[0]
		phi = obs_value[1]

		hat_z = H * self.s
		z = matrix([dis*cos(phi), dis*sin(phi)]).getT()

		# algorithmic update
		sigma_z = sim_env.rot_mtx(phi) * matrix([[sim_env.var_dis, 0],[0, dis*dis*sim_env.var_phi]]) * sim_env.rot_mtx(phi).getT() 
		sigma_invention = H * self.sigma * H.getT()  + sigma_z
		kalman_gain = self.sigma*H.getT()*sigma_invention.getI()

		self.s = self.s + kalman_gain*(z - hat_z)
		self.sigma = self.sigma - kalman_gain*H*self.sigma

		# analytic update
		sigma_th_z =  max(sim_env.var_dis, sim_env.d_max*sim_env.d_max*sim_env.var_phi)* sim_env.i_mtx_2.copy() 
		self.th_sigma = (self.th_sigma.getI() + H_ij.getT() * sigma_th_z.getI() * H_ij).getI()



	def comm(self, comm_robot_s, comm_robot_sigma, comm_robot_th_sigma):

		e =  0.8 # (iii+1)*0.01

		# algorithmic update
		sig_inv = e*self.sigma.getI() + (1-e)*comm_robot_sigma.getI()
		self.s = sig_inv.getI() * (e*self.sigma.getI()*self.s + (1-e)*comm_robot_sigma.getI()*comm_robot_s)
		self.sigma = sig_inv.getI()

		# analytic update
		self.th_sigma = ( e*self.th_sigma.getI() + (1-e)*comm_robot_th_sigma.getI() ).getI()		



	def commA(self, comm_robot_s, comm_robot_sigma, comm_robot_th_sigma):

		e =  0.999 # (iii+1)*0.01

		H_i = matrix([
			[0,0,0,0,0,0,1,0,0,0], 
			[0,0,0,0,0,0,0,1,0,0],
			[0,0,0,0,0,0,0,0,1,0],
			[0,0,0,0,0,0,0,0,0,1]], dtype=float)


		# algorithmic update
		sig_inv = e*self.sigma.getI() + (1-e)*H_i.getT()*comm_robot_sigma.getI()*H_i
		self.s = sig_inv.getI() * (e*self.sigma.getI()*self.s + (1-e)*H_i.getT()*comm_robot_sigma.getI()*comm_robot_s)
		self.sigma = sig_inv.getI()

		# analytic update
		self.th_sigma = ( e*self.th_sigma.getI() + (1-e)*H_i.getT()*comm_robot_th_sigma.getI()*H_i ).getI()	


	def commB(self, comm_robot_s, comm_robot_sigma, comm_robot_th_sigma):

		e =  0.999 # (iii+1)*0.01

		H_i = matrix([
			[0,0,1,0,0,0,0,0,0,0], 
			[0,0,0,1,0,0,0,0,0,0],
			[0,0,0,0,1,0,0,0,0,0], 
			[0,0,0,0,0,1,0,0,0,0],			
			[0,0,0,0,0,0,1,0,0,0], 
			[0,0,0,0,0,0,0,1,0,0],
			[0,0,0,0,0,0,0,0,1,0],
			[0,0,0,0,0,0,0,0,0,1]], dtype=float)


		# algorithmic update
		sig_inv = e*self.sigma.getI() + (1-e)*H_i.getT()*comm_robot_sigma.getI()*H_i
		self.s = sig_inv.getI() * (e*self.sigma.getI()*self.s + (1-e)*H_i.getT()*comm_robot_sigma.getI()*comm_robot_s)
		self.sigma = sig_inv.getI()

		# analytic update
		self.th_sigma = ( e*self.th_sigma.getI() + (1-e)*H_i.getT()*comm_robot_th_sigma.getI()*H_i ).getI()	
