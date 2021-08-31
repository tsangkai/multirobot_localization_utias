import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Animator():
    """docstring for animate_plot"""

    def __init__(self, dataset_labels, data_recorder, analyzer):
        self.name = "Animator"
        self.dataset_labels = dataset_labels
        self.robot_loc_time_unit = analyzer.robot_location_at_unit_time_interval(
            state_recorder)

    def get_robot_locations(self, robot_loc_time_unit, mode):
        arr = []
        # robots in dataset are labeled from 1 to num_robots
        for robot_label in self.dataset_labels:
            # declare a mapping of x, y coordinate lists
            m = {'x': robot_loc_time_unit[i][mode + '_x'],
                 'y': robot_loc_time_unit[i][mode + '_y']}
            arr.append(m)
        return arr

    def update(self, ):
        pass

    def animate_plot(dataset_labels, data_recorder, analyzer, lm=None):
        self.robot_locations_gt = get_robot_locations(
            num_robots, self.robot_loc_time_unit, 'gt')
        self.robot_locations_est = get_robot_locations(
            num_robots, self.robot_loc_time_unit, 'est')
        ani = animation.FuncAnimation(fig, update, fargs=(num_robots, fig, init_path_est, init_path_gt, init_points_est, init_points_gt, robot_locations_est,
                                                          robot_locations_gt, times, time_func, min_time_arr, arr_err, arr_var, loc_err, trace_sigma), frames=min_length(times), interval=50, blit=False)
