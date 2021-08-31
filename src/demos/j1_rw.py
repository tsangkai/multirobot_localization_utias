from Cooperative_Localization_Journal1 import all_algorithms_comp

'''
# for all colo datasets,  may need to change scale on comparison graph: loc[0,0.5], trace[0,0.05]

datasets_path = "/Users/shengkangchen/Documents/CoLo/CoLo-D/CoLo-Datasets/official_dataset"
robot_labels = [1,2,3]
duration = 180 # duration for the simulation in sec

for i in [1, 2, 3, 4]:
    dataset_label = str(i)
    dataset_path = datasets_path + dataset_label + "/"
    graph_name = 'colo_d'+dataset_label
    print(dataset_path)
    all_algorithms_comp(dataset_path, robot_labels, duration, graph_name, robots_cant_observe_lm = [2])

'''

# for all UTIAS datasets, may need to change scale on comparison graph: loc[0,1.5], trace[0,0.2]
#datasets_path = "/Users/shengkangchen/Documents/CoLo/CoLo-D/UTIAS-dataset/MRCLAM_Dataset"
datasets_path = "/home/william/CoLo/CoLo-D/UTIAS-dataset/MRCLAM_Dataset" # for desktop Ubuntu

robot_labels = [1,2,3,4,5]
duration = 600 # duration for the simulation in sec

for i in range(1, 9):
    dataset_label = str(i)
    dataset_path = datasets_path + dataset_label + "/"
    graph_name = 'UTIAS_d'+dataset_label
    print(dataset_path)
    all_algorithms_comp(dataset_path, robot_labels, duration, graph_name, robots_cant_observe_lm = [4, 5])

