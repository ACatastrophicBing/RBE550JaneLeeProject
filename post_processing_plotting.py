import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import math
import json
import os
import argparse
import pandas as pd
import csv
from itertools import chain

# # parser = argparse.ArgumentParser()
# # parser.add_argument("file_name")
# # parser.add_argument("--episodes", default=0)
# # parser.add_argument("--map_path", default=".\RevisedData\map_0.csv")

# episodes = 9
# prefix = '.\RevisedData\'
# suffix = '_Global_Init_5humans'
# trials= ['PRM', 'RRT', 'RRTStar', 'informed_RRT_star', 'astar', 'dijkstra']
# file_names = [(prefix+trial_name+suffix) for trial_name in trials]
# PRM_trial_name = prefix+'PRM'+suffix
# RRT_trial_name = prefix+'RRT'+suffix
# RRTS_trial_name = prefix+'RRTStar'+suffix
# IRRTS_trial_name = prefix+'informed_RRT_star'+suffix
# AStar_trial_name = prefix+'astar'+suffix
# Dijkstra_trial_name = prefix+'dijkstra'+suffix

# # args = parser.parse_args()

# # print("\n. . . Live Plotting the robot path and simulation ", args.file_name, " . . .")
# # if int(episodes) == 0:
# #     print(". . . Plotting only one episode")
# # else:
# #     print(". . . Plotting multiple episodes on one map")
# size = int(episodes) + 1
# success = np.full([size], 1)
# processing_time = np.zeros([size], dtype=float)
# pp_time = np.zeros([size], dtype=float)
# simulation_world_time = np.zeros([size], dtype=int)
# distance_travelled = np.zeros([size], dtype=float)
# robot_smoothness = np.zeros([size], dtype=float)
# path_smoothness = np.zeros([size], dtype=float)
# path_length = np.zeros([size], dtype=float)
# r_paths = [[]] * size
# g_paths = [[]] * size
# print(". . . Loading Map Data")
# map = []
# with open('.\RevisedData\map_1.csv', 'r') as file:
#     # Create a CSV reader object
#     csv_reader = csv.reader(file)

#     # Get the first row from the reader
#     first_row = next(csv_reader)

#     # Print the first row columns
#     for column in first_row:
#         row = column.strip('[').strip(']').strip('\n')
#         map.append(np.fromstring(row, sep=' '))


# map = np.flip(np.asarray(map), axis=1)

# for i in range(size):
#     if args.file_name is not None:
#         print(". . . Loading Model Data")
#         data_path = args.file_name + "_" + str(i) + ".csv"
#         print(". . . Loading File ", data_path)
#         data = pd.read_csv(data_path)
#     else:
#         print("Invalid file name")
#         break

#     simulation_world_time[i] = data['Simulation_World_Time'][0]
#     if simulation_world_time[i] == 0 or simulation_world_time[i] == 400:
#         success[i] = 0
#     robot_smoothness[i] = data['Path_Smoothness'][0]
#     processing_time[i] = data['Tot_Processing_Time'][0]
#     pp_time[i] = data['Path_Planning_Time'][0]
#     distance_travelled[i] = data['Robot_Distance_Travelled'][0]
#     r_path = []
#     g_path = []
#     pp_smooth = 0
#     pp_length = 0
#     prev_path = [0,0]
#     for j in range(1, len(np.asarray(data['Path_Taken']))):
#         # print("Path Taken : " + data['Path_Taken'][j])
#         # print("Path Generated : " + data['Path_Generated'][j])
#         r_path.append(np.fromstring(data['Path_Taken'][j].strip('[]'), sep=',') * 20)  # Note the 20 is the map definition / wrld size
#         r_paths[i].append(r_path[-1])
#         # print(data['Path_Generated'][j])
#         gtemppath = []
#         try:
#             for loc in data['Path_Generated'][j].strip('[[').strip(']]').split(']\r\n ['):
#                 gtemppath.append(np.fromstring(loc, sep=' ') * 20)  # Note the 20 is the map definition / world size
#             # print(gtemppath, prev_path, np.array_equal(gtemppath, prev_path), j)
#             if not np.array_equal(gtemppath, prev_path):
#                 prev_path = gtemppath.copy()
#                 g_path.append(np.asarray(gtemppath))
#                 g_paths[i].append(g_path[-1])

#                 for k in range(1, len(g_path[j-1])):
#                     pp_length = pp_length + math.dist(g_path[j-1][k],g_path[j-1][k-1])
#                     if k > 2:
#                         p1 = np.asarray(g_path[j-1][k-1]) - np.asarray(g_path[j-1][k - 2])  # Yes this could be quicker but also, no
#                         p2 = np.asarray(g_path[j-1][k]) - np.asarray(g_path[j-1][k - 1])
#                         pp_smooth = pp_smooth + abs(math.atan2(p2[1]-p1[1], p2[0]-p1[0]))
#         except:
#             continue
#     path_length[i] = pp_length
#     path_smoothness[i] = pp_smooth

# # print(distance_travelled)
# std_rp_length = np.std(distance_travelled)
# std_pp_smooth = np.std(path_smoothness)
# std_rp_smooth = np.std(robot_smoothness)
# mean_rp_length = np.mean(distance_travelled)
# mean_pp_smooth = np.mean(path_smoothness)
# mean_rp_smooth = np.mean(robot_smoothness)

episodes = 9
size = int(episodes) +1
prefix = '.\\RevisedData\\'
suffix = '_Global_Init_5humans'
trials= ['PRM', 'RRT', 'RRTStar', 'informed_RRT_star'] #'astar', 'dijkstra'
file_names = [(prefix+trial_name+suffix) for trial_name in trials]

# Dictionary to store metrics for each trial
trial_metrics = {trial: {} for trial in trials}

print(". . . Loading Map Data")
map_data = []
with open('.\RevisedData\map_1.csv', 'r') as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        if row and row != '...':
            map_data.append(np.fromstring(row[0], sep=' '))
map_data = np.flip(np.asarray(map_data), axis=1)

for trial_name in trials:
    trial_path = prefix + trial_name + suffix
    success = np.full([episodes + 1], 1)
    processing_time = np.zeros([episodes + 1], dtype=float)
    pp_time = np.zeros([episodes + 1], dtype=float)
    simulation_world_time = np.zeros([episodes + 1], dtype=int)
    distance_travelled = np.zeros([episodes + 1], dtype=float)
    robot_smoothness = np.zeros([episodes + 1], dtype=float)
    path_smoothness = np.zeros([episodes + 1], dtype=float)
    path_length = np.zeros([episodes + 1], dtype=float)
    r_paths = [[]] * (episodes + 1)
    g_paths = [[]] * (episodes + 1)

    for i in range(episodes + 1):
        data_path = trial_path + "_" + str(i) + ".csv"
        print(". . . Loading File ", data_path)
        try:
            data = pd.read_csv(data_path)
            simulation_world_time[i] = data['Simulation_World_Time'][0]
            if simulation_world_time[i] == 0 or simulation_world_time[i] == 400:
                success[i] = 0
            robot_smoothness[i] = data['Path_Smoothness'][0]
            processing_time[i] = data['Tot_Processing_Time'][0]
            pp_time[i] = data['Path_Planning_Time'][0]
            distance_travelled[i] = data['Robot_Distance_Travelled'][0]
            r_path = []
            g_path = []
            pp_smooth = 0
            pp_length = 0
            prev_path = [0,0]
            for j in range(1, len(np.asarray(data['Path_Taken']))):
                r_path.append(np.fromstring(data['Path_Taken'][j].strip('[]'), sep=',') * 20)
                r_paths[i].append(r_path[-1])
                gtemppath = []
                try:
                    for loc in data['Path_Generated'][j].strip('[[').strip(']]').split(']\r\n ['):
                        gtemppath.append(np.fromstring(loc, sep=' ') * 20)
                    if not np.array_equal(gtemppath, prev_path):
                        prev_path = gtemppath.copy()
                        g_path.append(np.asarray(gtemppath))
                        g_paths[i].append(g_path[-1])

                        for k in range(1, len(g_path[j-1])):
                            pp_length += math.dist(g_path[j-1][k], g_path[j-1][k-1])
                            if k > 2:
                                p1 = np.asarray(g_path[j-1][k-1]) - np.asarray(g_path[j-1][k - 2])
                                p2 = np.asarray(g_path[j-1][k]) - np.asarray(g_path[j-1][k - 1])
                                pp_smooth += abs(math.atan2(p2[1]-p1[1], p2[0]-p1[0]))
                except:
                    continue
            path_length[i] = pp_length
            path_smoothness[i] = pp_smooth
        except FileNotFoundError:
            print("File not found:", data_path)
            continue

    trial_metrics[trial_name]['success'] = success.tolist()
    trial_metrics[trial_name]['processing_time'] = processing_time.tolist()
    trial_metrics[trial_name]['pp_time'] = pp_time.tolist()
    trial_metrics[trial_name]['simulation_world_time'] = simulation_world_time.tolist()
    trial_metrics[trial_name]['distance_travelled'] = distance_travelled.tolist()
    trial_metrics[trial_name]['robot_smoothness'] = robot_smoothness.tolist()
    trial_metrics[trial_name]['path_smoothness'] = path_smoothness.tolist()
    trial_metrics[trial_name]['path_length'] = path_length.tolist()
    trial_metrics[trial_name]['r_paths'] = r_paths
    trial_metrics[trial_name]['g_paths'] = g_paths

# Calculating standard deviations and means
std_rp_length = {trial: np.std(trial_metrics[trial]['distance_travelled']) for trial in trials}
std_pp_smooth = {trial: np.std(trial_metrics[trial]['path_smoothness']) for trial in trials}
std_rp_smooth = {trial: np.std(trial_metrics[trial]['robot_smoothness']) for trial in trials}
mean_rp_length = {trial: np.mean(trial_metrics[trial]['distance_travelled']) for trial in trials}
mean_pp_smooth = {trial: np.mean(trial_metrics[trial]['path_smoothness']) for trial in trials}
mean_rp_smooth = {trial: np.mean(trial_metrics[trial]['robot_smoothness']) for trial in trials}

# import matplotlib.pyplot as plt

# Metrics to plot
metrics = ['success', 'processing_time', 'pp_time', 'simulation_world_time', 
           'distance_travelled', 'robot_smoothness', 'path_smoothness', 'path_length']

# Plot each metric
for metric in metrics:
    plt.figure(figsize=(10, 6))
    plt.title(f'{metric.capitalize()} Across Trials')
    plt.xlabel('Trials')
    plt.ylabel(metric.capitalize())

    # Plot data for each trial
    for trial_name in trials:
        trial_data = trial_metrics[trial_name][metric]
        plt.plot(range(len(trial_data)), trial_data, label=trial_name)

    # Plot mean and standard deviation if available
    if f'mean_{metric}' in globals() and f'std_{metric}' in globals():
        mean_data = [globals()[f'mean_{metric}']] * len(trials)
        std_data = [globals()[f'std_{metric}']] * len(trials)
        plt.errorbar(range(len(trials)), mean_data, yerr=std_data, fmt='o', color='black', label='Mean ± Std')

    plt.legend()
    plt.tight_layout()
    plt.show()



# fig, axs = plt.subplots(3,2)
# axs0 = axs[0,0]
# axs1 = axs[0,1]
# axs2 = axs[1,0]
# axs3 = axs[1,1]
# axs4 = axs[2,0]

# robot_travelled_plt = axs0.errorbar(1, mean_rp_length, std_rp_length, linestyle='None', marker='o', color='b')
# success_plt = axs4.errorbar(2, np.mean(success), np.std(success), linestyle='None', marker='o', color='g')
# robot_smooth_plt = axs3.errorbar(2, mean_rp_smooth, std_rp_smooth, linestyle='None', marker='^', color='r')
# robot_path_smoothness = axs3.errorbar(3, mean_pp_smooth, std_pp_smooth, linestyle='None', marker='^', color='g')

# arr = 1 - map
# img = 255 * np.dstack((arr, arr, arr))
# img_rotated = np.rot90(img, k=1)  # Rotate 90 degrees counterclockwise
# axs1.imshow(img_rotated)

# r_paths = np.asarray(r_paths)
# plt_r_path = []
# plt_g_paths = []
# for i in range(size):
#     plt_r_path.append(axs1.plot(r_paths[i, :, 0], r_paths[i, :, 1], color='r', alpha=0.01))
#     for j in range(len(g_paths[i])):
#         plt_g_paths.append(axs1.plot(g_paths[i][j][:, 0], g_paths[i][j][:, 1], color='b', alpha=0.01))

# processing_time_plt = axs2.plot(np.linspace(0, size, size), processing_time)   # Milliseconds
# path_processint_time_ply = axs2.plot(np.linspace(0, size, size), pp_time)      # Milliseconds
# world_time_plt = axs2.plot(np.linspace(0, size, size), simulation_world_time)  # Milliseconds

# plt.show()

# plt.close()