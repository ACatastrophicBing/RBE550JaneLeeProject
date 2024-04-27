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
import argparse

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

        if distance < 3:  # update to next waypoint
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
    map.update(t)

    global controller
    if sim.map.robot_flag and update_when_robot_moves and not update_when_map_updates:
        print("[ACT] Updating Map From Robot Position Update")
        controller = TrajectoryController(sim.map.path_plan(path_planner))
    if sim.map.map_flag and update_when_map_updates and not update_when_robot_moves:
        print("[ACT] Updating Path From Map Update")
        controller = TrajectoryController(sim.map.path_plan(path_planner))
    if t == 0: # Trajectory controller NEEDS to generate a path for robot to move, I don't know why, but that's just how it is
        print("[ACT] Initializing Starting Path Plan")
        controller = TrajectoryController(sim.map.path_plan(path_planner))

    if controller.waypoints is not None:
        target = controller.update_target(robot)
    else:
        print("[ACT] No Path Found")
        if not update_when_robot_moves and not update_when_robot_moves:
            print("[ACT] Environment MUST be restarted, because a new trajectory will NOT be made")
            return
        target = [robot['center'].x, robot['center'].y]

    if target == (None, None):
        print("[ACT] Robot has reached the final destination")
        robot['left'].F = 0
        robot['right'].F = 0
        return

    target_x, target_y = target


    base_speed = 0.6
    base_turn_speed = 0.2
    angle_tolerance = 5
    Kp_distance = 0.02
    Kp_angle = 0.05
    robot_to_target = 1  # Distance threshold to be considered "close" to the target


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
            robot['left'].F = -turn_direction * turn_speed
            robot['right'].F = turn_direction * turn_speed
        else:  # Move forward
            robot['left'].F = speed
            robot['right'].F = speed

    # Stop when you are close by
    if distance_to_target < 3:
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
    # parser.add_argument("--any_more_things_you_may_need",default = False)

    args = parser.parse_args()
    path_planner = args.path_planner
    update_when_robot_moves = args.update_on_robot_movement
    update_when_map_updates = args.update_on_map_update
    visualize = args.visualize

    wrld_size = [50,50]
    # NOTE : THIS IS IMPORTANT : THIS IS THE PLACE WHERE YOU CHANGE WHERE TO PUT THE GOAL
    goal = [10,20]

    env = Environment(wrld_size[0], wrld_size[1])
    num_robots = 1
    num_humans = 5
    num_obstacles = 20
    print(visualize)
    sim = Simulator(env, goal, rand_obstacles=20, wrld_size=wrld_size, num_humans=num_humans, global_map_init=args.dont_init_with_global_knowledge, use_global_knowledge=args.use_global_knowledge, visualize=visualize)
    map = sim.map
    controller = None
    run_sim(env, lambda t, robot: act(t, robot), total_time=400, dt_display=50)
    map.path_plan("PRM")

# NOTE : If you are looking for a function call or data, do print(dir(data)) and for type do print(type(data))