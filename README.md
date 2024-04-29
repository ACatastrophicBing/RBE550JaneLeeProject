# RobotSim373

A simple robot simulator based on Box2D and visualization with matplotlib.

## Install

This Project was written for Python 3.8.18
You need the following packages: Box2D, matplotlib, numpy, pandas, scipy, networkx, skimage

## Run
in your conda environment in this folder, run the following : 
python .\Examples\main.py Data_Recording_Folder --path_planner PRM

Where Data_Recording_Folder is the folder you wish to save your data into
The following flags can also be called when running this line : 
--path_planner path_planning_algo : Defaults to PRM, but you can change it to any algorithm we have included in path_planner in the Simulator.py file
--visualize : Defaults to no visualization, but calling this will visualize the planned path whenever the path planner is called, you need to implement you own visualization code when an algorithm is added
--dont_init_with_global_knowledge : Pretty self explanatory, calling this will initialize the robot map as being completely unknown and empty (WIP For Frontier finding algos)
--use_global_knowledge : The robot will use the global map instead of the robot local map
--update_on_robot_movement : The robot will update its path when the robot moves
--update_on_map_update : The robot will update its path when the global map updates, this does not mean the robot is using global knowledge
--iterations : The number of times you want to run this specific experiment, this is for data collection purposes, and when collecting data, DO NOT VISUALIZE THE SIM OR PATHS GENERATED

## Run post_processing_plotting.py (Data Plotting)
Run this file from the terminal following a similar formatting : 
python post_processing_plotting.py Data_Recording_Folder/NameOfFileButStopBeforeThe_BeforeTheNumber --episodes num_episodes_you_want_plotted

This will plot file 0 or all files with the name up to episode num_episodes