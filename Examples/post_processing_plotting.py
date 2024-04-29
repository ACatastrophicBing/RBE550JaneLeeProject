import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import math
import json
import os
import argparse
import pandas as pd
from itertools import chain

parser = argparse.ArgumentParser()
parser.add_argument("--file_name")
parser.add_argument("--episodes", default=0)


args = parser.parse_args()

print("\n. . . Live Plotting the robot path and simulation ", args.data_file, " . . .")
if int(args.episodes) == 0:
    print(". . . Plotting only one episode")
else:
    print(". . . Plotting multiple episodes on one map")

success = np.full([(args.episodes+1)], 1)
processing_time = np.array([(args.episodes+1)], dtype=float) / 1000000
pp_time = np.array([(args.episodes+1)], dtype=float) / 1000000
simulation_world_time = np.array([(args.episodes+1)], dtype=int)
distance_travelled = np.array([(args.episodes+1)], dtype=float)
robot_smoothness = np.array([(args.episodes+1)], dtype=float)
path_smoothness = np.array([(args.episodes+1)], dtype=float)
path_length = np.array([(args.episodes+1)], dtype=float)
r_paths = [] * (args.episodes+1)
g_paths = [] * (args.episodes+1)
for i in range(args.episodes + 1):
    if args.data_file is not None:
        print(". . . Loading Model Data")
        data_path = args.data_path + "_" + i + ".csv"
        print(". . . Loading File ", data_path)
        data = pd.read_csv(data_path)
    else:
        print("Invalid file name")
        break

    simulation_world_time[i] = data['Simulation_World_Time'][0]
    if simulation_world_time[i] == 0 or simulation_world_time[i] == 400:
        success[i] = 1
    robot_smoothness[i] = data['Path_Smoothness'][0]
    processing_time[i] = data['Tot_Processing_Time'][0]
    pp_time[i] = data['Path_Planning_Time'][0]
    distance_travelled[i] = data['Robot_Distance_Travelled'][0]
    r_path = []
    g_path = []
    pp_smooth = 0
    pp_length = 0
    for j in range(1, len(np.asarray(data['Path_Taken']))):
        r_path.append(np.fromstring(data['Path_Taken'][j].strip('[]'), sep=','))
        g_path.append(np.fromstring(data['Path_Generated'][j].strip('[]'), sep=','))
        r_paths[i].append(r_path[-1])
        g_paths[i].append(g_path[-1])
        pp_length = pp_length + math.dist(g_path[i],g_path[i+1])
        if j < len(np.asarray(data['Path_Taken']))-1 :
            p1 = np.asarray(g_path[j]) - np.asarray(g_path[j - 1])  # Yes this could be quicker but also, no
            p2 = np.asarray(g_path[j + 1]) - np.asarray(g_path[j])
            pp_smooth = pp_smooth + abs(math.atan2(p2[1]-p1[1], p2[0]-p1[0]))
    path_length[i] = pp_length
    path_smoothness[i] = pp_smooth

std_rp_length = np.std(distance_travelled, axis=1)
std_pp_smooth = np.std(path_smoothness, axis=1)
std_rp_smooth = np.std(robot_smoothness, axis=1)
mean_rp_length = np.mean(distance_travelled, axis=1)
mean_pp_smooth = np.mean(path_smoothness, axis=1)
mean_rp_smooth = np.mean(robot_smoothness, axis=1)

fig, axs = plt.subplots(2,2)
axs0 = axs[0]
axs1 = axs[1]
axs2 = axs[2]
axs3 = axs[3]

robot_travelled_plt = axs0.errorbar(np.linspace(0,args.episodes+1, args.episodes+1), mean_rp_length, std_rp_length, linestyle='None', marker='^')
robot_smooth_plt = axs0.errorbar(np.linspace(0,args.episodes+1, args.episodes+1), mean_rp_smooth, std_rp_smooth, linestyle='None', marker='^')
robot_path_smoothness = axs0.errorbar(np.linspace(0,args.episodes+1, args.episodes+1), mean_pp_smooth, std_pp_smooth, linestyle='None', marker='^')

r_paths = np.asarray(r_paths)
g_paths = np.asarray(g_paths)
plt_r_path = []
plt_g_paths = []
for i in range(args.episode + 1):
    plt_r_path.append(axs1.plot(r_paths[i, :, 0], r_paths[i, :, 1], color='r', alpha=0.25))
    for j in range(len(g_paths[i])):
        plt_g_paths.append(axs1.plot(g_paths[i, j, :, 0], g_paths[i, j, :, 1], color='b', alpha=0.25))

processing_time_plt = axs2.plt(np.linspace(0, args.episodes+1, args.episodes+1), processing_time)   # Milliseconds
path_processint_time_ply = axs2.plt(np.linspace(0, args.episodes+1, args.episodes+1), pp_time)      # Milliseconds
world_time_plt = axs2.plt(np.linspace(0, args.episodes+1, args.episodes+1), simulation_world_time)  # Milliseconds

