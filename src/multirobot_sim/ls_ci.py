from numpy import matrix
from numpy import random
from numpy import linalg
from math import cos, sin, atan2, sqrt

from robotteam import RobotTeam

import sim_env



class RobotTeamLocalCI(RobotTeam):
	""" Local state with CI algorithm """

	def __init__(self , initial_s):
		RobotTeam.__init__(self, initial_s)



	def ablt_obsv(self, idx, obs_value, landmark):

		ii = 2*idx

		H = sim_env.rot_mtx(self.theta[idx]).getT()*matrix([[-1,0],[0,-1]], dtype=float)

		local_s = self.s[ii:ii+2]	
		local_sigma = self.sigma[ii:ii+2,ii:ii+2]

		dis = obs_value[0]
		phi = obs_value[1]

		hat_z = sim_env.rot_mtx(self.theta[idx]).getT() * (landmark.position + H*local_s)
		z = matrix([dis*cos(phi), dis*sin(phi)]).getT()

		sigma_z = sim_env.rot_mtx(phi) * matrix([[sim_env.var_dis, 0],[0, dis*dis*sim_env.var_phi]]) * sim_env.rot_mtx(phi).getT() 
		sigma_invention = H * local_sigma * H.getT()  + sigma_z
		kalman_gain = local_sigma * H.getT() * sigma_invention.getI()


		self.s[ii:ii+2]	= local_s + kalman_gain*(z - hat_z)

		self.sigma[ii:ii+2,ii:ii+2] = local_sigma - kalman_gain*H*local_sigma





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


		z = matrix([[dis*cos(phi)],[dis*sin(phi)]])

		
		hat_j = z + self.s[i:i+2]

		sigma_j_star = self.sigma[i:i+2,i:i+2] + sim_env.rot_mtx(phi) * matrix([[sim_env.var_dis, 0],[0, dis*dis*sim_env.var_phi]]) * sim_env.rot_mtx(phi).getT() 
		sigma_j = self.sigma[j:j+2,j:j+2]

		e = 0.83

		sigma_j_next_inv = e*sigma_j.getI() + (1-e)*sigma_j_star.getI()


		self.s[j:j+2] = sigma_j_next_inv.getI()*(e*sigma_j.getI()*self.s[j:j+2]  + (1-e)*sigma_j_star.getI()*hat_j)
		self.sigma[j:j+2,j:j+2] = sigma_j_next_inv.getI()


