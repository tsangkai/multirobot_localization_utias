from numpy import matrix
from numpy import linalg as LA
import numpy as np


from gs_ci import RobotLocalCI
import sim_env

import math






a_1 = matrix([[1, 0],
	[0, 1]], dtype=float)

a_2 = matrix([[1, 20],
	[20, 1000]], dtype=float)

a_1_inv = a_1.getI()
a_2_inv = a_2.getI()


print(a_1.getI())
print(a_2.I)
print(a_1_inv-a_2_inv)