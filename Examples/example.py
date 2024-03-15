#!/usr/bin/env python
# coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
path = os.getcwd()
sys.path.insert(1,path)
print(path)
from RobotSim373 import *
import PRM as PRM
import math

class Map:
    def __init__(self, env, robots, boxes,definition=[10,10], lidar_range=0,wrld_size=[24,24]):
        self.env = env               # The environment we are in
        self.robots = robots         # How many actors we have in the environment that are mapping
        self.definition = definition # How detailed of a map we want
        self.lidar_range = lidar_range
        self.wrld_size = wrld_size
        self.map = [[0 for width in range(definition[0])] for height in range(definition[1])]
        self.boxes = [box.body for box in boxes]

    def update(self):
        # for robot in self.robots:
        #     continue
        for box in self.boxes:
            for vert in range(len(box.vertices)):
                self.generate_obstacle_lines(self.world_to_map(box.vertices[vert]), self.world_to_map(box.vertices[vert+1]))
                self.fill_inside_box(self.world_to_map(box.position))

    def world_to_map(self,p1):
        return [p1[0]/self.wrld_size[0] * self.definition[0], p1[1]/self.wrld_size[1] * self.definition[1]]

    def fill_inside_box(self,p):
        x, y = p
        if self.map[x][y] != 0:  # If the point is not inside the box,
            return
        self.map[x][y] = 1  # Fill the point
        # Check and fill neighboring points
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            new_x, new_y = x + dx, y + dy
            if 0 <= new_x < self.definition[0] and 0 <= new_y < self.definition[1]:
                self.fill_inside_box([new_x, new_y])

    def generate_obstacle_lines(self,p1,p2):
        line = ski.draw.line(p1[0], p1[1], p2[0], p2[1])
        for p in line:
            self.map[p[0]][p[1]] = 1
class Simulator:
    def __init__(self, env, num_robots, rand_obstacles=0, map_selector=None,wrld_size=[24,24]):
        self.env = env
        self.robots = []
        self.obstacles = []
        self.map = Map(env, self.robots, boxes=self.obstacles, definition=[100,100], lidar_range=20,wrld_size=wrld_size)

        if map_selector is not None:
            # TODO : Create maps to allow robots to properly explore environment
            placeholder = 0
        else:
            for i in range(rand_obstacles):
                self.obstacles.append(Box(self.env, x=Simulator.randbetween(8, 22), y=Simulator.randbetween(8, 22),
                    width=Simulator.randbetween(.2, 1.2), height=2, angle=Simulator.randbetween(0, 360), color='purple'))

        # for rob in range(num_robots): # TODO : Modify the environment to handle multiple robots
        #     self.robots.append(Robot(env))
        #     self.build(self.robots[rob])
        self.robots = Robot(env)
        self.build(self.robots)

        print(self.robots)

    def build(self, robot):
        # TODO : Generate random positions of robots that don't overlap with other robots


        box1 = Box(robot, x=2, y=10, name="right")
        box2 = Box(robot, x=2, y=11, name="left")

        connect(box1, box2, "weld")

        disk1 = Disk(robot, x=2, y=10.5, radius=1, name="center")

        connect(disk1, box1, "distance")
        connect(disk1, box2, "distance")

    def randbetween(low, high):
        return np.random.rand() * (high - low) + low



sim = None

def act(t, robot):
    # TODO  Modify the robotsim.py to simulate multiple robots at once
    # for robot in robots:
    distance = robot['center'].read_distance() # Disk is reading the distance ahead of it

    if distance > 10:
        robot['left'].F = 0.4
        robot['right'].F = 0.4
    else:
        robot['left'].F = 0.1
        robot['right'].F = 0.1

    # robot['left'].F, robot['right'].F = robot_controller(robot)

    robot.message = distance

def robot_controller( robot):
    return [0,0]

if __name__ == "__main__":
    wrld_size = [24,24]
    env = Environment(wrld_size[0], wrld_size[1])
    num_robots = 1
    sim = Simulator(env, 1, rand_obstacles=20,wrld_size=wrld_size)

    run_sim(env, act, figure_width=6, total_time=13, dt_display=1)
