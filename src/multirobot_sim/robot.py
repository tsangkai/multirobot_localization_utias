from numpy import matrix
from numpy import random

from math import cos, sin, atan2, sqrt
import sim_env


class Robot():
	""" The class for global state algorithm """

	N = sim_env.num_robot

	def __init__(self, index , initial_s):

		self.index = index

		self.s = initial_s.copy()
		self.sigma = sim_env.i_mtx_10.copy()*0.01

		self.position = [initial_s[2*index,0], initial_s[2*index+1,0]]
		self.theta = 0.0


	# access functions
	def get_s(self):
		return self.s.copy()

	def get_sigma(self):
		return self.sigma.copy()

	def error(self):
		ii = 2*self.index
		
		return sqrt( pow(self.s[ii]-self.position[0], 2) + pow(self.s[ii+1]-self.position[1], 2)) 






