#!/usr/bin/env python
# coding: utf-8

import numpy as np
from Simulator import Simulator, Map
import sys
import os
path = os.getcwd()
sys.path.insert(1,path)
print(path)
from RobotSim373 import *



sim = None
map = None
robot_map = None

def act(t, robot):
    #EVERYTHING IS IN DEGREES
    target_x = 15
    target_y = 15
    base_speed = 0.6
    base_turn_speed = 0.2
    angle_tolerance = 5
    Kp_distance = 0.02
    Kp_angle = 0.05
    robot_to_target = 2  # Distance threshold to be considered "close" to the target


    # Direction and distance
    dx = target_x - robot['center'].x
    dy = target_y - robot['center'].y

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
            # print("Robot turning")
            robot['left'].F = -turn_direction * turn_speed
            robot['right'].F = turn_direction * turn_speed


        else:  # Move forward
            robot['left'].F = speed
            robot['right'].F = speed

    # Stop when you are close by
    if distance_to_target < 3:
        robot['left'].F = 0
        robot['right'].F = 0


    map.update(t)

    for human in sim.humans:
        if human.read_distance() < 2:
            # Turn
            human.F = 0
            human.τ = 0.01
        else:
            human.F = 0.3


def robot_controller( robot):
    return [0,0]

if __name__ == "__main__":
    wrld_size = [50,50]
    env = Environment(wrld_size[0], wrld_size[1])
    num_robots = 1
    num_humans = 5
    num_obstacles = 20
    sim = Simulator(env, rand_obstacles=20, map_selector=0, wrld_size=wrld_size, num_humans=num_humans, global_map_init=True, use_global_knowledge=False)
    map = sim.map
    run_sim(env, act, figure_width=6, total_time=10, dt_display=10)
    map.path_plan("PRM")

# NOTE : If you are looking for a function call or data, do print(dir(data)) and for type do print(type(data))