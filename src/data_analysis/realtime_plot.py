from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.font_manager import FontProperties
from matplotlib import gridspec
import numpy as np
import time
import sys
"""
author: Kyle
"""
# update point: function handling the logic of animating the dot, denoting the current
# position of the robot
# 2 modes: est for estimate, gt for groundtruth
# not used in the implementation


def update_point(i, fig, scat, robot_loc_time_unit, dataset_labels, robotNum, mode):
    label = robotNum
    x = robot_loc_time_unit[label][mode + '_x'][i]
    y = robot_loc_time_unit[label][mode + '_y'][i]
    loc = [x, y]
    scat.set_offsets(loc)
    return scat,
# update path: function handling the logic of animating a path trail left by the robot
# 2 modes: est for estimate, gt for groundtruth
# not used in the implementation


def update_path(i, fig, l, robot_loc_time_unit, dataset_labels, robotNum, mode):
    label = robotNum
    x = robot_loc_time_unit[label][mode + '_x']
    y = robot_loc_time_unit[label][mode + '_y']
    l.set_data(x[:i], y[:i])
    return l,
# get the list of x, y locations of each robot and return those as a list
# 2 modes: est for estimate, gt for groundtruth


def get_robot_locations(num_robots, robot_loc_time_unit, mode):
    arr = []
    # robots in dataset are labeled from 1 to num_robots
    for i in range(1, num_robots + 1):
        # declare a mapping of x, y coordinate lists
        m = {'x': robot_loc_time_unit[i][mode + '_x'],
             'y': robot_loc_time_unit[i][mode + '_y']}
        arr.append(m)
    return arr
# initialize initial positions of each robot for dots


def initialize_robots(num_robots, locations, colors):
    arr = []
    for i in range(0, num_robots):
        scat = plt.scatter(locations[i]['x'], locations[i]['y'], c=colors[i])
        arr.append(scat)
    return arr
# initialize starting point of the robot path trial
# 2 modes: est for estimate, gt for groundtruth


def initialize_path(num_robots, ax, locations, colors, mode):
    s = ""
    mode_str = ""
    # define path marker, based on mode
    # define label representation, based on mode
    if mode == 'est':
        s = ':'
        mode_str = 'estimate'
    else:
        s = '-'
        mode_str = 'groundtruth'
    arr = []
    for i in range(0, num_robots):
        line, = ax.plot(locations[i]['x'], locations[i]['y'],
                        colors[i]+s, label='Robot ' + str(i+1) + ' ' + mode_str)
        arr.append(line)
    return arr
# initialize graphs to be incrementally constructed based on animation over time


def initialize_graphs(num_robots, ax_err, ax_var, loc_err, trace_sigma, time_func, colors):
    arr_err = []
    arr_var = []
    for i in range(0, num_robots):
        time_arr = time_func(i + 1)
        #line, = ax_err.plot(time_arr, loc_err[i+1], colors[i]+'-', label='Robot ' + str(i+1))
        #line2, = ax_var.plot(time_arr, trace_sigma[i+1], colors[i]+'-', label='Robot ' + str(i+1))
        line, = ax_err.plot(time_arr, loc_err[i+1], colors[i]+'-')
        line2, = ax_var.plot(time_arr, trace_sigma[i+1], colors[i]+'-')
        arr_err.append(line)
        arr_var.append(line2)
    return [arr_err, arr_var]
# get the time array for each robot


def get_robot_times(num_robots, robot_loc_time_unit):
    arr = []
    for i in range(1, num_robots + 1):
        arr.append(robot_loc_time_unit[i]['time'])
    return arr
# return the time array of minimum time duration


def min_robot_times(times):
    res = []
    m = 2**63 - 1
    for arr in times:
        if m > len(arr):
            res = list(arr)
            m = len(arr)
    return res
# update function for aggregate animation


def update(i, num_robots, fig, l_est, l_gt, scat_est, scat_gt, location_est, location_gt, times, time_func, min_time_arr, arr_err, arr_var, loc_err, trace_sigma):
    # aggregate of both scatterplot point and path trails
    res = []
    for txt in fig.texts:
        txt.set_visible(False)
    tmp = fig.text(0.2, 0.2, "Time: " +
                   str(round(times[0][i], 1)), fontsize=16)
    for robotNum in range(0, num_robots):
        # factor out common variables
        est_x = location_est[robotNum]['x']
        est_y = location_est[robotNum]['y']
        gt_x = location_gt[robotNum]['x']
        gt_y = location_gt[robotNum]['y']
        # scatter
        s1 = scat_est[robotNum]
        s2 = scat_gt[robotNum]
        s1.set_offsets([est_x[i], est_y[i]])
        s2.set_offsets([gt_x[i], gt_y[i]])
        res.append(s1,)
        res.append(s2,)
        # path
        l1 = l_est[robotNum]
        l2 = l_gt[robotNum]
        l1.set_data(est_x[:i], est_y[:i])
        l2.set_data(gt_x[:i], gt_y[:i])
        res.append(l1,)
        res.append(l2,)
        # graph
        time_arr = time_func(robotNum + 1)
        ind = find_index_time(time_arr, min_time_arr[i])
        l3 = arr_err[robotNum]
        l4 = arr_var[robotNum]
        l3.set_data(time_arr[:ind], loc_err[robotNum+1][:ind])
        l4.set_data(time_arr[:ind], trace_sigma[robotNum+1][:ind])
    #return res
# to prevent subscript out of bounds, determine minimum length timestamp array to be used
# for animation


def min_length(times):
    m = 2**63 - 1  # 64-bit max int upper bound
    for arr in times:
        m = min(m, len(arr))
    return m
# return the index of the array that is less than or equal to t, the current time


def find_index_time(arr, t):
    ind = 0
    for i in arr:
        if i >= t:
            return ind
        ind += 1
    return ind
# precondition: dataset should have sufficient information for N robots
# postcondition: dataset is not manipulated; the function simply mutates the data for
# simpler data processing internally
# This function handles the plot animation graph functionalities


def animate_plot(dataset_labels, data_recorder, analyzer, lm=None):

    time_func = data_recorder.get_time_arr
    loc_err = data_recorder.get_loc_err_arr()
    trace_sigma = data_recorder.get_trace_sigma_s_arr()

    # obtain data
    robot_loc_time_unit = analyzer.robot_location_at_unit_time_interval(
        data_recorder)
    # determine number of robots under consideration for this animation
    num_robots = int(len(dataset_labels))
    # initialize graphs: error, demo, state variance

    fig = plt.figure(figsize=(12, 9), tight_layout=True)
    fig.subplots_adjust(top=0.88)
    gs = gridspec.GridSpec(3, 2)
    fig.suptitle('CoLo Robot Demo Simulation', fontsize=20)
    ax_err = plt.subplot(gs[0, 0])
    ax_var = plt.subplot(gs[0, 1])
    ax = plt.subplot(gs[1:, :])

    '''
    fig = plt.figure()
    gs = gridspec.GridSpec(3, 2)
    ax_err = plt.subplot(gs[0, 0])
    ax_var = plt.subplot(gs[0, 1])
    ax = plt.subplot(gs[1:, :])

    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    '''
    # configure graph
    # set bounds
    #fig.set_size_inches((18,18))
    #ax = plt.axes(xlim=(-6, 6), ylim=(-6, 8))
    #ax_err = plt.axes(xlim=(0, 100), ylim=(0, 0.4))
    # assign labels
    plt.axis('equal')
    ax.set_title('CoLo Demo', fontsize=16)
    ax.set_xlabel('x-axis[m]', fontsize=16)
    ax.set_ylabel('y-axis[m]', fontsize=16)
    ax_err.set_title('Location error', fontsize=16)
    ax_err.set_xlabel('Time[s]', fontsize=16)
    ax_err.set_ylabel('RMS[m]', fontsize=16)
    ax_var.set_title('Trace of state variance', fontsize=16)
    ax_var.set_xlabel('Time [s]', fontsize=16)
    ax_var.set_ylabel('Sigma_s [m^2]', fontsize=16)

    # obtain robot locations from dataset
    robot_locations_est = get_robot_locations(
        num_robots, robot_loc_time_unit, 'est')
    robot_locations_gt = get_robot_locations(
        num_robots, robot_loc_time_unit, 'gt')
    # hardcoded for now, to be passed as parameter in test_simulation call
    colors = ['b', 'g', 'r', 'c', 'k']
    # initialize graph initial positions
    init_points_est = initialize_robots(
        num_robots, robot_locations_est, colors)
    init_points_gt = initialize_robots(num_robots, robot_locations_gt, colors)
    init_path_est = initialize_path(
        num_robots, ax, robot_locations_est, colors, 'est')
    init_path_gt = initialize_path(
        num_robots, ax, robot_locations_gt, colors, 'gt')
    ig = initialize_graphs(num_robots, ax_err, ax_var,
                           loc_err, trace_sigma, time_func, colors)
    arr_err = ig[0]
    arr_var = ig[1]

    if lm != None:
        lm_x = []
        lm_y = []
        for landmark_id, [x, y] in lm.items():
            lm_x.append(x)
            lm_y.append(y)
        ax.scatter(lm_x, lm_y, s=120, marker=6, label='Landmark')

    # obtain robot timestamps
    times = get_robot_times(num_robots, robot_loc_time_unit)
    min_time_arr = min_robot_times(times)
    # initialize animation, passing in frame update function
    # robot animation
    ani = animation.FuncAnimation(fig, update, fargs=(num_robots, fig, init_path_est, init_path_gt, init_points_est, init_points_gt, robot_locations_est,
                                                      robot_locations_gt, times, time_func, min_time_arr, arr_err, arr_var, loc_err, trace_sigma), frames=min_length(times), interval=50, blit=False)
    # Show legend
    fontP = FontProperties()
    fontP.set_size('x-large')
    #ax.legend(prop=fontP, bbox_to_anchor=(1.0, 0.8), loc=9, ncol=1)
    ax.legend(prop=fontP, loc=1)
    ax_var.legend(prop=fontP, bbox_to_anchor=(1.1, 0.8), loc=9, ncol=1)
    # TODO - fix gif formatting
    # (optional) save as GIF, comment the following line if you don't want this
    # https://imagemagick.org/ used for install
    # ani.save('animated_plot.gif', writer="imagemagick", fps=60) # sort by date modified in Colo folder to quickly fine, don't forget to delete colo_demo frames
    # print("******Saved gif in Colo folder!******")
    plt.show()
    plt.pause(0.001)
