#!/usr/bin/env python
# coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import os
path = os.getcwd()
sys.path.insert(1,path)
print(path)
from RobotSim373 import *
from PRM import PRM
import math
import skimage as ski


class Map:
    def __init__(self, env, robots, boxes,definition=[100,100], lidar_range=0,wrld_size=[24,24]):
        self.env = env               # The environment we are in
        self.robots = robots         # How many actors we have in the environment that are mapping
        self.definition = definition # How detailed of a map we want
        self.lidar_range = lidar_range
        self.wrld_size = wrld_size
        self.map = np.full((definition[0], definition[1]),1)
        self.boxes = [box.body for box in boxes]
        self.PRM = PRM(1-self.map)

    def update(self):
        # for robot in self.robots:
        #     continue
        for box in self.boxes:
            angle = box.angle
            position = self.world_to_map(box.position)
            vertices = len(box.fixtures[0].shape.vertices)
            for vert in range(vertices):
                self.generate_obstacle_lines(position,
                             self.world_to_map(self.rotate(box.fixtures[0].shape.vertices[vert%vertices], angle)),
                             self.world_to_map(self.rotate(box.fixtures[0].shape.vertices[(vert+1)%vertices], angle)))
            self.fill_inside_box(position)

    def world_to_map(self,p1):
        return (math.floor(p1[0]/self.wrld_size[0] * self.definition[0]), math.floor(p1[1]/self.wrld_size[1] * self.definition[1]))

    def rotate(self, vect, angle):
        (x, y) = vect
        newx = x * math.cos(angle) - y * math.sin(angle)
        newy = x * math.sin(angle) + y * math.cos(angle)
        return [newx, newy]

    def fill_inside_box(self,p):
        x, y = p
        if self.map[x][y] != 1:  # If the point is not inside the box,
            return
        self.map[x][y] = 0  # Fill the point
        # Check and fill neighboring points
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            new_x, new_y = x + dx, y + dy
            if 0 <= new_x < self.definition[0] and 0 <= new_y < self.definition[1]:
                self.fill_inside_box([new_x, new_y])

    def generate_obstacle_lines(self,position, p1,p2):
        line = ski.draw.line(p1[0] + position[0], p1[1] + position[1], p2[0] + position[0], p2[1] + position[1])
        for i in range(len(line[:][0])):
            self.map[line[0][i]][line[1][i]] = 0

    def visualize(self):
        pos = self.robots['center'].position
        pos = self.world_to_map([pos.x,pos.y])
        self.PRM.sample(n_pts=1000, sampling_method="random")
        self.PRM.search(pos, self.world_to_map([20,10]))
        self.PRM.draw_map()

    def map_to_world(self, p1):
        return (p1[0] / self.definition[0] * self.wrld_size[0],
                p1[1] / self.definition[1] * self.wrld_size[1])

        
    def calculate_path(self, start_pos, end_pos):
        start = self.world_to_map(start_pos)
        end = self.world_to_map(end_pos)
        path = self.PRM.search(start, end)
        if path is not None:
            # Convert path back to world coordinates
            return [self.map_to_world(p) for p in path]
        return []

class Simulator:
    def __init__(self, env, num_robots, rand_obstacles=0, map_selector=None,wrld_size=[24,24]):
        self.env = env
        self.robots = []
        self.obstacles = []

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

        # print(type(self.robots))
        # print(dir(self.robots))

        self.map = Map(env, self.robots, boxes=self.obstacles, definition=[1000,1000], lidar_range=20,wrld_size=wrld_size)

    def build(self, robot):
        # TODO : Generate random positions of robots that don't overlap with other robots
        # ADDED FRICITION HERE
        dampling = 0.5

        box1 = Box(robot, x=2, y=10, name="right", linearDamping=dampling )
        box2 = Box(robot, x=2, y=11, name="left",linearDamping=dampling)

        connect(box1, box2, "weld")

        disk1 = Disk(robot, x=2, y=10.5, radius=1, name="center")

        connect(disk1, box1, "distance")
        connect(disk1, box2, "distance")
        connect(disk1, box1, "weld")
        connect(disk1, box2, "weld")

    def randbetween(low, high):
        return np.random.rand() * (high - low) + low

class TrajectoryController:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.reached_destination = False

    def update_target(self, robot):
        if self.reached_destination:
            return None, None  #done

        target_x, target_y = self.waypoints[self.current_waypoint_index]
        dx = target_x - robot['center'].x
        dy = target_y - robot['center'].y
        distance = np.sqrt(dx**2 + dy**2)

        if distance < 2:  # update to next one 
            print("DONEEE")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.reached_destination = True
                return None, None
            else:
                target_x, target_y = self.waypoints[self.current_waypoint_index]

        return target_x, target_y


def act(t, robot, controller):
    target = controller.update_target(robot)
    if target is (None,None):
        print("Robot has reached the final destination")
        robot['left'].F = 0
        robot['right'].F = 0
        return  #

    target_x, target_y = target
    print(f"Target Coordinates: {target_x}, {target_y}")

    base_speed = 0.6  
    base_turn_speed = 0.2  
    angle_tolerance = 5  
    Kp_distance = 0.02 
    Kp_angle = 0.05  
    robot_to_target = 1  #do not make it too close not tuned well 

    # Direction and distance
    print("target",target)
    dx = target_x - robot['center'].x
    dy = target_y - robot['center'].y
    print("Robot X", robot['center'].x)  
    print("Robot Y", robot['center'].y)  

    distance_to_target = np.sqrt(dx**2 + dy**2)
    angle_to_target = np.degrees(np.arctan2(dy, dx))
    
    # Robot's current orientation
    robot_angle = robot['center'].angle % 360
    print("Robot Angle", robot_angle)  

    # Calculate the shortest angle to the target
    angle_diff = (angle_to_target - robot_angle + 180) % 360 - 180
    print("Robot Angle Difference", angle_diff)  

    # Adjust speed
    speed = min(base_speed, Kp_distance * distance_to_target)
    if distance_to_target > robot_to_target:
        turn_speed = min(base_turn_speed, Kp_angle * abs(angle_diff))
        if abs(angle_diff) > angle_tolerance:  # Need to turn
            turn_direction = np.sign(angle_diff)
            print("Robot turning") 
            robot['left'].F = -turn_direction * turn_speed
            robot['right'].F = turn_direction * turn_speed
            print("speed left", -turn_direction * turn_speed) 
            print("speed right", turn_direction * turn_speed) 
        else:  # Move forward
            print("Robot going straight")  
            print("speed", speed)  
            robot['left'].F = speed
            robot['right'].F = speed

    # Stop when close to target
    if distance_to_target < 2: 
        print("Robot at the target")  
        print("speed", speed)  
        robot['left'].F = 0
        robot['right'].F = 0



if __name__ == "__main__":
    wrld_size = [24,24]
    env = Environment(wrld_size[0], wrld_size[1])
    num_robots = 1
    sim = Simulator(env, 1, rand_obstacles=4, wrld_size=wrld_size)
    map = sim.map

    map.update() 

    pos = map.robots['center'].position
    start_pos = map.world_to_map([pos.x,pos.y])
    end_pos = (20, 20)  # target position

    map.PRM.sample(n_pts=200, sampling_method="random")  
    map.PRM.search(start_pos, map.world_to_map(end_pos))  

    if map.PRM.path:
        # Filter  'start' and 'goal' 
        waypoints = [map.map_to_world(map.PRM.samples[idx]) for idx in map.PRM.path if isinstance(idx, int)]
        controller = TrajectoryController(waypoints)
        run_sim(env, lambda t, robot: act(t, robot, controller), figure_width=6, total_time=300, dt_display=10)
        # map.visualize()
    else:
        print("No path found between the start and end positions.")


# NOTE : If you are looking for a function call or data, do print(dir(data)) and for type do print(type(data))