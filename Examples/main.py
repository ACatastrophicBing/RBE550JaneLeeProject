#!/usr/bin/env python
# coding: utf-8

import numpy as np
from Simulator import Simulator
import sys
import os
path = os.getcwd()
sys.path.insert(1,path)
print(path)
from RobotSim373 import *
import argparse
import csv
import time
import math

sim = None
map = None
robot_map = None
path_planner = None
update_when_robot_moves = None
update_when_map_updates = None
visualize = None

class TrajectoryController:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.reached_destination = False

    def update_target(self, robot):
        if self.reached_destination:
            return None, None  # done

        target_x, target_y = self.waypoints[self.current_waypoint_index]
        dx = target_x - robot['center'].x
        dy = target_y - robot['center'].y
        distance = np.sqrt(dx ** 2 + dy ** 2)

        if distance < 2:  # update to next waypoint
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.reached_destination = True
                return None, None
            else:
                target_x, target_y = self.waypoints[self.current_waypoint_index]

        return target_x, target_y

def act(t, robot):

    # TODO : Whenever you are trying to run a specific algorithm, follow a similar format to this :
    # NOTE: THIS IS NOT USING TRAJECTORY PLANNER AND IS NOT IN ANY WAY FINISHED;
    # YOU NEED TO MODIFY THE FOLLOWING CODE TO WORK WITH WHAT ALGORITHM YOU WANT TO RUN PROPERLY
    # IF YOU ARE ONLY RUNNING THE PATH PLANNER ON STARTUP, CHANGE NONE OF THESE VARIABLES AND @CAN TO GET THE MAP
    # TO INITIALIZE BEFORE RUNNING ACT
    global controller, tick_time_run, path_planning_time, robot_path, map_processing_time

    
    ms = time.process_time_ns()
    map.update(t)
    map_processing_time = map_processing_time + time.process_time_ns() - ms


    s = time.process_time_ns()
    if sim.map.robot_flag and update_when_robot_moves and not update_when_map_updates:
        print("[ACT] Updating Map From Robot Position Update")
        controller = TrajectoryController(sim.map.path_plan(path_planner))
    if sim.map.map_flag and update_when_map_updates and not update_when_robot_moves:
        print("[ACT] Updating Path From Map Update")
        controller = TrajectoryController(sim.map.path_plan(path_planner))
    if t == 0: # Trajectory controller NEEDS to generate a path for robot to move, I don't know why, but that's just how it is
        print("[ACT] Initializing Starting Path Plan")
        controller = TrajectoryController(sim.map.path_plan(path_planner))
    path_planning_time = path_planning_time + time.process_time_ns() - s

    if controller.waypoints is not None:
        target = controller.update_target(robot)
        path_generated.append(controller.waypoints)
    else:
        print("[ACT] No Path Found")
        if not update_when_robot_moves and not update_when_robot_moves:
            print("[ACT] Environment MUST be restarted, because a new trajectory will NOT be made")
            return True
        target = [robot['center'].x, robot['center'].y]

    if target == (None, None):
        print("[ACT] Robot has reached the final destination")
        robot['left'].F = 0
        robot['right'].F = 0
        tick_time_run = t
        return True

    target_x, target_y = target

    base_speed = 0.4
    base_turn_speed = 0.2
    angle_tolerance = 5
    Kp_distance = 0.05
    Kp_angle = 0.05
    robot_to_target = 1  # Distance threshold to be considered "close" to the target


    # Direction, distance, and location
    dx = target_x - robot['center'].x
    dy = target_y - robot['center'].y
    robot_path.append([robot['center'].x, robot['center'].y])

    distance_to_target = np.sqrt(dx**2 + dy**2)
    angle_to_target = np.degrees(np.arctan2(dy, dx))

    # Robot's current orientation
    robot_angle = robot['center'].angle % 360

    # Calculate the shortest angle to the target
    angle_diff = (angle_to_target - robot_angle + 180) % 360 - 180

    # Adjust speed
    speed = min(base_speed, Kp_distance * distance_to_target)
    if distance_to_target > robot_to_target:
        turn_speed = min(base_turn_speed, Kp_angle * abs(angle_diff))

        if abs(angle_diff) > angle_tolerance:  # Need to turn
            turn_direction = np.sign(angle_diff)
            robot['left'].F = -turn_direction * turn_speed
            robot['right'].F = turn_direction * turn_speed
        else:  # Move forward
            print('speed', speed)
            robot['left'].F = speed
            robot['right'].F = speed

    # Stop when you are close by
    if distance_to_target < 1:
        robot['left'].F = 0
        robot['right'].F = 0

    for human in sim.humans:
        if human.read_distance() < 2:
            # Turn
            human.F = 0
            human.τ = 0.01
        else:
            human.F = 0.3

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("recording_path")
    parser.add_argument("--visualize", action='store_true', default=False)
    parser.add_argument("--path_planner", default='PRM')
    parser.add_argument("--dont_init_with_global_knowledge", action='store_false', default=True)
    parser.add_argument("--use_global_knowledge", action='store_true', default=False)
    parser.add_argument("--update_on_robot_movement", action='store_true', default=False)
    parser.add_argument("--update_on_map_update", action='store_true', default=False)
    parser.add_argument("--iterations", default=1)
    # parser.add_argument("--any_more_things_you_may_need",default = False)

    args = parser.parse_args()
    path_planner = args.path_planner
    update_when_robot_moves = args.update_on_robot_movement
    update_when_map_updates = args.update_on_map_update
    visualize = args.visualize

    wrld_size = [50,50]
    # NOTE : THIS IS IMPORTANT : THIS IS THE PLACE WHERE YOU CHANGE WHERE TO PUT THE GOAL
    goal = [25, 25]
    start = [2, 10.5]

    num_robots = 1
    num_humans = 5
    num_obstacles = 20
    global tick_time_run, path_planning_time, map_processing_time, robot_path, path_generated

    if args.path_planner == "informed_RRT_star":
        dilation = 0.1
    else:
        dilation = 0.5

    time_allocated = 400

    for i in range(int(args.iterations)):
        env = Environment(wrld_size[0], wrld_size[1])

        robot_path = []
        path_generated = []
        path_planning_time, map_processing_time, tick_time_run = 0, 0, 0

        sim = Simulator(env, start, goal, c_space_dilation=dilation, rand_obstacles=20, wrld_size=wrld_size,
                        num_humans=num_humans, global_map_init=args.dont_init_with_global_knowledge,
                        use_global_knowledge=args.use_global_knowledge, visualize=visualize, map_selector=0)
        map = sim.map
        controller = None

        ts = time.process_time_ns()
        run_sim(env, lambda t, robot: act(t, robot), total_time=time_allocated, dt_display=5, disp=True)
        end = time.process_time_ns()
        total_time = end - ts
        failed_straightup = False

        if tick_time_run == 0:
            print("[MAIN] Robot FAILED to reach goal")
            tick_time_run = time_allocated
            failed_straightup = True
        else:
            print("[MAIN] The time taken to run and reach the goal successfully was ", tick_time_run, " seconds of simulated environment time")

        print("[MAIN] Processing Time Taken : ", total_time)
        print("[MAIN] Path Planning Processing Taken : ", path_planning_time)
        print("[MAIN] Time Taken Spent Updating The Map : ", map_processing_time)

        recording_path = args.recording_path + "/"
        data_file_name = args.path_planner
        if args.dont_init_with_global_knowledge:
            data_file_name = data_file_name + "_Global_Init"
        if args.use_global_knowledge:
            data_file_name = data_file_name + "_Global_Knowledge"
        if args.update_on_robot_movement:
            data_file_name = data_file_name + "_UpdateOnRobotMovement"
        if args.update_on_map_update:
            data_file_name = data_file_name + "_UpdateOnMapUpdate"

        dist_travelled = 0
        angle_sum = 0
        for k in range(len(robot_path) - 1):
            dist_travelled = dist_travelled + math.dist(robot_path[k],robot_path[k+1])
            if k > 0:
                p1 = np.asarray(robot_path[k]) - np.asarray(robot_path[k-1]) # Yes this could be quicker but also, no
                p2 = np.asarray(robot_path[k+1]) - np.asarray(robot_path[k])
                angle_sum = angle_sum + abs(math.atan2(p2[1]-p1[1], p2[0]-p1[0]))
        if failed_straightup:
            smoothness = 0
        else:
            smoothness = angle_sum / len(robot_path)

        data_file_name = data_file_name + "_" + str(i) + ".csv"
        print("[MAIN] Saving ", data_file_name)
        with open(recording_path + data_file_name, 'w+') as output:
            writer = csv.writer(output, delimiter=',')
            writer.writerow(['Tot_Processing_Time', 'Simulation_World_Time', 'Path_Planning_Time',
                             'Robot_Distance_Travelled', 'Path_Smoothness', 'Path_Taken', 'Path_Generated'])
            writer.writerow([total_time, tick_time_run, path_planning_time, dist_travelled, smoothness, 0, 0])
            for j in range(0, len(robot_path)):
                writer.writerow([0, 0, 0, 0, 0, robot_path[j], path_generated[j]])


# NOTE : If you are looking for a function call or data, do print(dir(data)) and for type do print(type(data))