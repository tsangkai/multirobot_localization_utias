from matplotlib import pyplot as plt
import IPython

# [time_stamps, loc_err_per_run, data_recorder.name]
# [time_stamps, trace_per_run, data_recorder.name]
def generate_comparison_plot(arr_loc_err, arr_trace, only_trace):
    fig = plt.figure()
    fig1 = fig.add_subplot(211)
    fig2 = fig.add_subplot(212)
    for loc_err in arr_loc_err:
        fig1.plot(loc_err[0], loc_err[1], label = loc_err[2])
    
    for trace in arr_trace:
        if (only_trace != None) and trace[2] in only_trace:
            fig2.plot(trace[0], trace[1], '--' ,label = trace[2])
        else:
            fig2.plot(trace[0], trace[1], label = trace[2])

    fig1.set_title('Location error', fontsize = 24)
    fig1.set_xlabel('Time[s]', fontsize = 22)
    fig1.set_ylabel('RMS[m]', fontsize = 22) 
    fig1.set_ylim(0, 1)
    fig1.legend(loc=1, fontsize = 16)
    fig1.tick_params(labelsize=18)

    fig2.set_title('Trace of state covariance', fontsize = 24)
    fig2.set_xlabel('Time [s]', fontsize = 22)
    fig2.set_ylabel('Sigma_s [m^2]', fontsize = 22)
    fig2.set_ylim(0, 0.1)
    fig2.legend(loc=1, fontsize = 20)
    fig2.tick_params(labelsize=18)

    # plt.subplots_adjust(hspace=.6)
    plt.show()
    return

def robot1_algo_comparison_plot(arr_data_recorder, only_trace=None):
    print("************Algorithm Comparison***************")
    arr_loc_err = []
    arr_trace = []

    for data_recorder in arr_data_recorder:
        r1_label = data_recorder.get_dataset_labels()[0]

        loc_err_per_run = data_recorder.loc_err_arr[r1_label]
        trace_per_run = data_recorder.trace_sigma_s_arr[r1_label]
        time_stamps = data_recorder.get_time_arr(r1_label)

        if only_trace == None or data_recorder.name not in only_trace:
            arr_loc_err.append([time_stamps, loc_err_per_run, data_recorder.name])
        arr_trace.append([time_stamps, trace_per_run, data_recorder.name] )
    print('Plotting Comparison Graphs')
    generate_comparison_plot(arr_loc_err, arr_trace, only_trace)
    return arr_loc_err, arr_trace

