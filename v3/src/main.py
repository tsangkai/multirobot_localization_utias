import argparse
import time
import utils as u


def parse_arguments():
    '''
    Parses command line arguments.

    Arguments
    ---------
    -p, --path : str
        Parent path to the UTIAS datasets.
    -d, --datasets : int
        UTIAS datasets to process.
    -a, --algorithms : str
        Localization algorithms to run on each dataset.
    --comm_type : str
        The type of communication for distributive localization algorithms.
    --comm_range : float
        Range of communication. Only useful if the communication type is "range".
    --comm_rate : float
        Rate of communication (seconds). Set to 0 to default to the time precision.
    --comm_prob_fail : float
        Probability of communication failure for each algorithm, between 0 and 1.
    --dt : float
        The time precision of each iteration (seconds).
    --offset : float
        The offset of the execution start time in the dataset (seconds).
    --duration : float
        The execution time starting from the offset. Set to zero to use the entire dataset.

    '''

    parser = argparse.ArgumentParser(description='UTIAS dataset and localization algorithm arguments.')

    parser.add_argument('-p', '--path', nargs='?', type=str,
                        dest='path',
                        default='../data/',
                        help='Parent path to the UTIAS datasets.')

    parser.add_argument('-d', '--datasets', nargs='+', type=int,
                        dest='datasets',
                        default=[9],
                        choices=[1, 2, 3, 4, 5, 6, 7, 8, 9],
                        help='UTIAS datasets to process.')

    parser.add_argument('-a', '--algorithms', nargs='+', type=str,
                        dest='algorithms',
                        default=['gsci'],
                        choices=['lscen', 'lsci', 'lsbda', 'gssci', 'gsci', 'lssci'],
                        help='Localization algorithms to run on each dataset.')

    parser.add_argument('--comm_type', nargs='?', type=str,
                        dest='comm_type',
                        default='all',
                        choices=['all', 'observation', 'range', 'observation_range'],
                        help='The type of communication for distributive localization algorithms.')

    parser.add_argument('--comm_range', nargs='?', type=float,
                        dest='comm_range',
                        default=10,
                        help='Range of communication (meters). Only useful if the communication type is "range".')
    
    parser.add_argument('--comm_rate', nargs='?', type=float,
                        dest='comm_rate',
                        default=1,
                        help='Rate of communication (seconds). Set to 0 to default to the time precision.')
    
    parser.add_argument('--comm_prob_fail', nargs='?', type=float,
                        dest='comm_prob_fail',
                        default=0,
                        help='Probability of communication failure for each algorithm, between 0 and 1.')

    parser.add_argument('--dt', nargs='?', type=float,
                        dest='dt',
                        default=0.25,
                        help='The time precision of each iteration (seconds).')

    parser.add_argument('--offset', nargs='?', type=float,
                        dest='offset',
                        default=60,
                        help='The offset of the execution start time in the dataset (seconds).')

    parser.add_argument('--duration', nargs='?', type=float,
                        dest='duration',
                        default=240,
                        help='The execution time starting from the offset. Set to zero to use the entire dataset.')

    args = parser.parse_args()

    return args


def main(args):
    '''
    Main function that analyzes then plots every specified algorithm for every specified dataset.
    '''

    # extract args
    path, datasets, algorithms, comm_type, comm_range, comm_rate, comm_prob_fail, dt, offset, duration = \
        args.path, args.datasets, args.algorithms, args.comm_type, args.comm_range, args.comm_rate, args.comm_prob_fail, args.dt, args.offset, args.duration

    print('Loaded Settings:')
    print('\tUTIAS MRCLAM Path: ' + str(path))
    print('\tDataset(s): ' + str(datasets))
    print('\tAlgorithm(s): ' + str(algorithms))
    print('\tCommunication Type: ' + str(comm_type))
    print('\tCommunication Range: ' + str(comm_range) + ' meters')
    print('\tCommunication Rate: ' + str(comm_rate) + ' seconds')
    print('\tCommunication Failure Probability: ' + str(comm_prob_fail))
    print('\tTime Precision: ' + str(dt) + ' seconds / timestep')
    print('\tTime Offset: ' + str(offset) + ' seconds')
    print('\tTime Duration: ' + str(duration) + ' seconds')
    print('')

    # start
    exec_start = time.time()

    # load datasets
    UTIAS = u.load_datasets(path, datasets)

    # execute algorithms on datasets
    results = u.execute(UTIAS, algorithms, comm_type, comm_range, comm_rate, comm_prob_fail, dt, offset, duration)

    # plot the results
    u.analyze(results, [comm_type, comm_range, comm_rate, comm_prob_fail, dt, offset, duration], save=True)

    # end
    exec_end = time.time()
    hours, minutes, seconds = u.exec_time(exec_start, exec_end)
    print("Total execution time: {:0>2}:{:0>2}:{:05.2f}".format(int(hours), int(minutes), seconds))


if __name__ == '__main__':

    args = parse_arguments()

    main(args)

