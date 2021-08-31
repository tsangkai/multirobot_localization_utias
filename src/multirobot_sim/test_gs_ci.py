from numpy import matrix
from numpy import linalg as LA
import numpy as np


from gs_ci import RobotLocalCI
import sim_env

import math


# import parameters

N = sim_env.num_robot 
M = sim_env.num_landmark 

itr = sim_env.iteration 
time = sim_env.time_end



# recording container

#sigma_tr_arr = [0] * time
#sigma_th_tr_arr = [0] * time
#error_arr = [0] * time

file_tr = open('gs_ci/tr.txt', 'w')
file_th_tr = open('gs_ci/th_tr.txt', 'w')
file_error = open('gs_ci/error.txt', 'w')



for i in range(itr):

    print(i)

    initial = matrix([1, 1, 1, 2, 2, 1, -1, -1, 1, 3], dtype=float).T



    robots = [None] * N
    for n in range(N):
        robots[n] = RobotLocalCI(n, initial.copy())


    landmarks = [None] * M
    for m in range(M):
        landmarks[m] = sim_env.Landmark(m, matrix([0.01, 0.02], dtype=float).getT())


    for t in range(time):
 
        # motion propagation 
        for mp_iteration in range(sim_env.num_of_m_p):
            robots[0].prop_update()
            robots[1].prop_update()
            robots[2].prop_update()
            robots[3].prop_update()
            robots[4].prop_update()

        #robot 0
        [dis, phi] = sim_env.relative_measurement(robots[0].position, robots[0].theta, landmarks[0].position)
        robots[0].ablt_obsv([dis, phi], landmarks[0])


        # robot 2
        #[dis, phi] = sim_env.relative_measurement(robots[2].position, robots[2].theta, landmarks[0].position)
        #robots[2].ablt_obsv([dis, phi], landmarks[0])

        [dis, phi] = sim_env.relative_measurement(robots[2].position, robots[2].theta, robots[0].position)
        robots[2].rela_obsv(0, [dis, phi])

        [dis, phi] = sim_env.relative_measurement(robots[2].position, robots[2].theta, robots[1].position)
        robots[2].rela_obsv(1, [dis, phi])


        # observation - robot 3
        [dis, phi] = sim_env.relative_measurement(robots[3].position, robots[3].theta, landmarks[0].position)
        robots[3].ablt_obsv([dis, phi], landmarks[0])

        [dis, phi] = sim_env.relative_measurement(robots[3].position, robots[3].theta, robots[4].position)
        robots[3].rela_obsv(4, [dis, phi])


        # communication
        #robots[2].commA(robots[3].s[6:10], robots[3].sigma[6:10,6:10], robots[3].th_sigma[6:10,6:10])
        #robots[0].commB(robots[2].s[2:10], robots[2].sigma[2:10,2:10], robots[2].th_sigma[2:10,2:10])
        robots[2].comm(robots[3].s, robots[3].sigma, robots[3].th_sigma)
        robots[0].comm(robots[2].s, robots[2].sigma, robots[2].th_sigma)

        #print(robots[0].sigma)
        #print(robots[2].sigma)
        #raw_input()


        # real error
        s = 0
        for j in range(5):
            s += pow(robots[0].s[2*j,0] - robots[j].position[0],2) + pow(robots[0].s[2*j+1,0] - robots[j].position[1],2)
        s = math.sqrt(s*0.2)

        file_error.write(str(s)+' ')
        #error_arr[t] = error_arr[t] + s*(1/float(itr))

        # covariance error
        #sigma_tr_arr[t] = sigma_tr_arr[t] + math.sqrt(0.2*robots[0].get_sigma().trace()[0,0] )*(1/float(itr))
        tr = math.sqrt(0.2*robots[0].get_sigma().trace()[0,0])
        file_tr.write(str(tr) + ' ')


        #sigma_th_tr_arr[t] = sigma_th_tr_arr[t] + math.sqrt(0.2*robots[0].get_th_sigma().trace()[0,0]  )*(1/float(itr))
        th_tr = math.sqrt(0.2*robots[0].get_th_sigma().trace()[0,0])
        file_th_tr.write(str(th_tr) + ' ')

    file_error.write('\n')
    file_tr.write('\n')
    file_th_tr.write('\n')


file_error.close()
file_tr.close()
file_th_tr.close()


#file = open('output_gs_ci.txt', 'w')
#for t in range(time):
#   file.write( str(sigma_tr_arr[t]) + ' ' + str(sigma_th_tr_arr[t]) + ' ' + str(error_arr[t]) + '\n')
#file.close()


