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
from skimage.morphology import isotropic_dilation
from scipy import spatial
import networkx as nx


class Map:
    def __init__(self, env, robot, boxes, humans=[],definition=[100,100], wrld_size=[24,24], lidar_range=5.0,
                 map_update_rate = 100, global_map_init = True, c_space_dilation = 1.0, human_radius = 0.5,
                 use_global_knowledge = False, max_obj_size=12):
        self.env = env               # The environment we are in
        self.robot = robot           # How many actors we have in the environment that are mapping
        self.humans = humans         # How many random movement agents we have in the environment
        self.human_size = human_radius
        self.definition = definition # How detailed of a map we want
        self.definition_conversion = [(definition[0]-1)/wrld_size[0], (definition[1]-1)/wrld_size[1]]
        self.lidar_range = int(lidar_range * self.definition_conversion[0])
        self.wrld_size = wrld_size
        self.lidar_range_mod = max_obj_size / 2 * self.definition_conversion[0]
        self.boxes = [box.body for box in boxes]
        self.cell_map = []
        self.human_size_map = []
        self.global_map_init = global_map_init
        self.c_space_dilation = c_space_dilation
        self.use_global_knowledge = use_global_knowledge

        self.robot_position = self.world_to_map(self.robot['center'].position)
        self.create_cell_maps()
        self.robot_flag = False # If the robot updates its position in world, flag this
        self.map_flag = False   # If the map has any changes compared to what it previously was, flag this
        self.cell_map_update = 0
        self.map_update_rate = map_update_rate # How often we check the map to update things, map will also update if robot moves

        self.box_angles = np.empty([len(self.boxes)], dtype=float)
        self.box_positions = np.empty([len(self.boxes),2], dtype=int)
        self.box_vertices = np.empty([len(self.boxes),4,2], dtype=float)
        self.points = np.zeros([len(self.boxes) * 4, 2], dtype=int)
        for box in range(len(self.boxes)):
        #     self.box_angles.append(box.angle)
        #     self.box_positions.append(self.world_to_map(box.position))
            self.box_vertices[box] = np.asarray(self.boxes[box].fixtures[0].shape.vertices)

        self.human_positions = []
        for human in self.humans:
            self.human_positions.append(self.world_to_map(human.position))

        self.robot_map = np.full((definition[0], definition[1]), 1, dtype=bool)

        self.map = np.zeros((definition[0], definition[1]), dtype=bool) # 0 is empty 1 is blocked

        self.frontier_map = np.full(self.map.shape, 1, dtype=bool) # Is this efficient? No, but this'll be needed later

        self.brute_force_init()

        if global_map_init:
            print("[MAP] Initializing Map Knowing Global Snapshot")
            # This cannot actually be done here since the environment hasn't been generated yet, so it is instead handled in the update function ONCE
        else:
            print("[MAP] Initializing Map NOT Knowing Global Snapshot")
            self.robot_map = np.full(self.map.shape, 1, dtype=bool)

        self.last_map_update = 0

        self.robot_cspace = np.zeros(self.map.shape, dtype=bool)

    def brute_force_init(self):
        for i in range(len(self.boxes)):
            verts = len(self.box_vertices[i])
            self.box_angles[i] = self.boxes[i].angle
            self.box_positions[i] = self.world_to_map(self.boxes[i].position)
            for vert in range(verts):
                self.generate_obstacle_lines(self.box_positions[i],
                                             self.world_to_map(self.rotate(self.box_vertices[i][vert % verts],
                                                                           self.box_angles[i])),
                                             self.world_to_map(self.rotate(self.box_vertices[i][(vert + 1) % verts],
                                                                           self.box_angles[i])))

    def update(self, tick):
        # for robot in self.robots:
        #     continue
        self.map_flag = False

        if int(tick * 1000) - self.last_map_update > self.map_update_rate and self.use_global_knowledge:
            # print("Updating Global Map")
            self.update_global_map()
            self.last_map_update = tick
            self.map_flag = True

        perceived_position = self.world_to_map(self.robot['center'].position)
        if perceived_position != self.robot_position:
            self.robot_flag = True
            self.robot_position = perceived_position

        if self.global_map_init and tick > 1:
            print("Initializing the Robot map based on global current belief")
            self.robot_map = np.copy(self.map)
            self.global_map_init = False

        if int(tick * 1000) - self.last_map_update > self.map_update_rate and self.robot_flag and not self.use_global_knowledge and not self.global_map_init:
            self.robot_flag = False
            if self.use_global_knowledge:
                self.robot_map = self.map
            else:
                for i in range(len(self.boxes)):
                    for vert in range(len(self.box_vertices[i])):
                        self.points[i * 4 + vert] = (np.asarray(self.box_positions[i])
                         + np.asarray(self.world_to_map(self.rotate(self.box_vertices[i][vert], self.box_angles[i]))))

                kdtree = spatial.KDTree(self.points)
                near_index = kdtree.query_ball_point(self.robot_position, r=self.lidar_range + self.lidar_range_mod)
                near = [self.points[p] for p in near_index]
                for point in near:
                    # print("Adding ", point, " to the map")
                    vert_loc = np.where(self.points == point)
                    box, vert = int(math.floor(vert_loc[0][0] / len(self.boxes))), vert_loc[0][0] % len(self.boxes)
                    p1 = self.points[box * 4 + (vert + 1)%4].astype(int)
                    if vert == 0:
                        p2 = self.points[box * 4 + 3].astype(int)
                    else:
                        p2 = self.points[box * 4 + vert - 1].astype(int)
                    line1 = ski.draw.line(self.points[vert_loc][0], self.points[vert_loc][1], p1[0], p1[1])
                    for i in range(len(line1[:][0])):
                        if math.sqrt(max(0,(self.robot_position[0] - line1[0][i])**2 + (self.robot_position[1] - line1[1][i]))):
                            self.robot_map[line1[0][i]][line1[1][i]] = self.map[line1[0][i]][line1[1][i]]
                    line2 = ski.draw.line(self.points[vert_loc][0], self.points[vert_loc][1], p2[0], p2[1])
                    for i in range(len(line2[:][0])):
                        if math.sqrt(max(0,(self.robot_position[0] - line2[0][i])**2 + (self.robot_position[1] - line2[1][i]))):
                            self.robot_map[line2[0][i]][line2[1][i]] = self.map[line2[0][i]][line2[1][i]]

                for i in range(len(self.humans)):  # These dudes move every time step so you have to update the global map
                    if self.human_positions[i] != self.world_to_map(self.humans[i].position):
                        for pos in self.human_size_map:
                            x = pos[0] + self.human_positions[i][0]
                            y = pos[1] + self.human_positions[i][1]
                            if 0 <= x < self.definition[0] and 0 <= y < self.definition[1] and math.sqrt(max(0,(self.robot_position[0] - x)**2 + (self.robot_position[1] - y))):
                                self.robot_map[x][y] = 0

                        self.human_positions[i] = self.world_to_map(self.humans[i].position)
                        for pos in self.human_size_map:
                            x = pos[0] + self.human_positions[i][0]
                            y = pos[1] + self.human_positions[i][1]
                            if 0 <= x < self.definition[0] and 0 <= y < self.definition[1]:
                                self.robot_map[x][y] = 1


                if self.cell_map_update%10 == 0:
                    # print("Removing local objecct drift")
                    for pos in self.cell_map:
                            x = pos[0] + self.robot_position[0]
                            y = pos[1] + self.robot_position[1]
                            if 0 <= x < self.definition[0] and 0 <= y < self.definition[1]:
                                self.robot_map[x][y] = self.map[x][y]

                self.cell_map_update = self.cell_map_update + 1


    def update_global_map(self):
        for i in range(len(self.boxes)):
            if (self.box_angles[i] != self.boxes[i].angle or
                    not np.array_equal(self.box_positions[i], self.world_to_map(self.boxes[i].position))):
                verts = len(self.box_vertices[i])
                for vert in range(verts):
                    self.remove_obstacle_lines(self.box_positions[i],
                                self.world_to_map(self.rotate(self.box_vertices[i][vert%verts][:], self.box_angles[i])),
                                self.world_to_map(self.rotate(self.box_vertices[i][(vert+1)%verts], self.box_angles[i])))
                self.box_angles[i] = self.boxes[i].angle
                self.box_positions[i] = self.world_to_map(self.boxes[i].position)
                for vert in range(verts):
                    # print("Adding a line for box ", self.box_positions[i], " from ",
                    #       self.world_to_map(self.rotate(self.box_vertices[i][vert % verts],self.box_angles[i])), " to ",
                    #       self.world_to_map(self.rotate(self.box_vertices[i][(vert + 1) % verts],self.box_angles[i])))
                    self.generate_obstacle_lines(self.box_positions[i],
                                self.world_to_map(self.rotate(self.box_vertices[i][vert % verts],self.box_angles[i])),
                                self.world_to_map(self.rotate(self.box_vertices[i][(vert + 1) % verts],self.box_angles[i])))

        for i in range(len(self.humans)): # These dudes move every time step so you have to update the global map
            if self.human_positions[i] != self.world_to_map(self.humans[i].position):
                for pos in self.human_size_map:
                    x = pos[0] + self.human_positions[i][0]
                    y = pos[1] + self.human_positions[i][1]
                    if 0 <= x < self.definition[0] and 0 <= y < self.definition[1]:
                        self.map[x][y] = 0
                self.human_positions[i] = self.world_to_map(self.humans[i].position)
                for pos in self.human_size_map:
                    x = pos[0] + self.human_positions[i][0]
                    y = pos[1] + self.human_positions[i][1]
                    if 0 <= x < self.definition[0] and 0 <= y < self.definition[1]:
                        self.map[x][y] = 1




    def world_to_map(self,p1):
        return (math.floor(p1[0] * self.definition_conversion[0]), math.floor(p1[1] * self.definition_conversion[1]))

    def rotate(self, vect, angle):
        (x, y) = vect
        newx = x * math.cos(angle) - y * math.sin(angle)
        newy = x * math.sin(angle) + y * math.cos(angle)
        return [newx, newy]

    def flood_fill(self,p):
        x, y = p
        if self.map[x][y] != 0:  # If the point is not inside the box,
            return
        self.map[x][y] = 1  # Fill the point
        # Check and fill neighboring points
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            new_x, new_y = x + dx, y + dy
            if 0 <= new_x < self.definition[0] and 0 <= new_y < self.definition[1]:
                self.flood_fill([new_x, new_y])

    def generate_obstacle_lines(self, position, p1,p2):
        line = ski.draw.line(p1[0] + position[0], p1[1] + position[1], p2[0] + position[0], p2[1] + position[1])
        for i in range(len(line[:][0])):
            self.map[line[0][i]][line[1][i]] = 1

    def remove_obstacle_lines(self, position, p1,p2):
        line = ski.draw.line(p1[0] + position[0], p1[1] + position[1], p2[0] + position[0], p2[1] + position[1])
        for i in range(len(line[:][0])):
            self.map[line[0][i]][line[1][i]] = 0

    def path_plan(self, algorithm):
        if self.use_global_knowledge:
           self.robot_cspace =  np.asanyarray( isotropic_dilation(self.map,radius=int(self.c_space_dilation * self.definition_conversion[0]))
)   
        else:
            self.robot_cspace = isotropic_dilation(self.robot_map,
                                                   radius=int(self.c_space_dilation * self.definition_conversion[0]))
        # self.robot_cspace = self.robot_map
        path = None

        if algorithm == "PRM":
            print(type(self.robot_cspace))
            print(self.robot_cspace)
            self.PRM = PRM(self.robot_cspace)
            self.PRM.sample(n_pts=2000, sampling_method="random")
            if len(self.PRM.samples) == 0:
                print("No samples generated.")
                return None
            # Start and goal
            start = self.robot_position
            goal = self.world_to_map([32, 37]) 

            self.PRM.search(start, goal)
            if self.PRM.path:
                    print('PRM PATH',self.PRM.path)
                    node_path = self.PRM.path  
                    clean_node_path =  path = [x for x in node_path if isinstance(x, int)]
                    print('clean node', clean_node_path)
                    path = [None] * len(clean_node_path)

                    for i in range(max(len(clean_node_path), 2)):
                        if i < len(clean_node_path):
                            sample = self.PRM.samples[clean_node_path[i]]
                            path[i] = (int(sample[0]), int(sample[1]))
                    print('first waypoint',path)

                    # divide bythe conversion 
                    if isinstance(self.definition_conversion, list):
                    
                            conversion_factor = self.definition_conversion[0]
                    else:
                            conversion_factor = self.definition_conversion


                    for i in range(len(path)):
                        path[i] = tuple(node / conversion_factor for node in path[i])


                    print('REAL waypoint',path)
                    return path 
            else:
                    print("No path found")
                    return None


        if algorithm == "AD*" and self.robot_flag:
            return path
        




    def create_cell_maps(self):
        """
        Creates a cell map that stores all the cells within a given radius around 0,0 to update our map.

        Args:
        radius (int): The radius to consider.

        Returns:
        cell_map : A list of all the cell map which we can loop over
        """
        radius = self.lidar_range

        # Populate the cell map
        for x in range(-radius, radius):
            for y in range(-radius, radius):
                distance = math.sqrt((x ** 2) + (y ** 2))
                if distance <= radius:
                    self.cell_map.append([x,y])


        for x in range(-int(self.human_size * self.definition_conversion[0]), int(self.human_size * self.definition_conversion[0])):
            for y in range(-int(self.human_size * self.definition_conversion[1]),int(self.human_size * self.definition_conversion[1])):
                distance = math.sqrt((x ** 2) + (y ** 2))
                if distance <= radius:
                    self.human_size_map.append([x, y])


class Simulator:
    def __init__(self, env, rand_obstacles=0, map_selector=None,wrld_size=[24,24], num_humans=0,
                 lidar_range=5.0, map_update_rate = 100, global_map_init = True, c_space_dilation = 1.0,
                 human_radius = 0.5, use_global_knowledge = False, definition = [1000,1000], human_friction=0.7):
        self.env = env
        self.robots = []
        self.obstacles = []
        self.humans = []
        for i in range(num_humans):
            self.humans.append(Disk(self.env, x=Simulator.randbetween(8, 22), y=Simulator.randbetween(8, 22),
                    radius=0.5, angle=Simulator.randbetween(0, 360), color='red', linearDamping=human_friction))

        if map_selector is not None:
            # TODO : Create maps to allow robots to properly explore environment

            placeholder = 0
            if map_selector == 1: # Labyrinth
                obstacle_color = 'purple'
                self.obstacles.append(Wall(self.env, x=20.5, y=25.5, width=1, height=7, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=28.5, y=25.5, width=1, height=7, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=37.5, y=49, width=25, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=49.5, y=24, width=1, height=48, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=18.5, y=38.5, width=-13, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=37.5, y=25, width=-1, height=28, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=32.5, y=25, width=-1, height=16, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=26.5, y=33.5, width=-5, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=42.5, y=6.5, width=3, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=42.5, y=25.5, width=-1, height=37, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=22, y=43.5, width=-14, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=15.5, y=38.5, width=9, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=10.5, y=32.5, width=1, height=13, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=10.5, y=17, width=1, height=12, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=24.5, y=11.5, width=11, height=11, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=26.5, y=28.5, width=5, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=33, y=16.5, width=2, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=22, y=16.5, width=14, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=15.5, y=25, width=1, height=16, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=19.5, y=33.5, width=9, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=21.5, y=6.5, width=33, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=5.5, y=25.5, width=1, height=37, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=15, y=43.5, width=18, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=25, y=1, width=48, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=0.5, y=24, width=1, height=48, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=9.5, y=49, width=19, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=24.5, y=39.5, width=1, height=21, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=24.5, y=21.5, width=9, height=1, angle=0, color=obstacle_color))
        else:
            obstacle_color = 'purple'
            self.obstacles.append(Wall(self.env, x=20.5, y=25.5, width=1, height=7, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=28.5, y=25.5, width=1, height=7, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=37.5, y=49, width=25, height=2, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=49.5, y=24, width=1, height=48, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=18.5, y=38.5, width=13, height=1, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=37.5, y=25, width=1, height=28, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=32.5, y=25, width=1, height=16, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=26.5, y=33.5, width=5, height=1, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=42.5, y=6.5, width=3, height=1, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=42.5, y=25.5, width=1, height=37, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=22, y=43.5, width=14, height=1, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=15.5, y=38.5, width=9, height=1, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=10.5, y=32.5, width=1, height=13, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=10.5, y=17, width=1, height=12, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=24.5, y=11.5, width=11, height=11, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=26.5, y=28.5, width=5, height=1, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=33, y=16.5, width=2, height=1, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=22, y=16.5, width=14, height=1, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=15.5, y=25, width=1, height=16, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=19.5, y=33.5, width=9, height=1, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=21.5, y=6.5, width=33, height=1, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=5.5, y=25.5, width=1, height=37, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=15, y=43.5, width=18, height=1, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=25, y=1, width=48, height=2, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=0.5, y=24, width=1, height=48, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=9.5, y=49, width=19, height=2, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=24.5, y=39.5, width=1, height=21, angle=0, color=obstacle_color))
            self.obstacles.append(Wall(self.env, x=24.5, y=21.5, width=9, height=1, angle=0, color=obstacle_color))
            # for i in range(rand_obstacles):
            #     self.obstacles.append(Box(self.env, x=Simulator.randbetween(8, 22), y=Simulator.randbetween(8, 22),
            #         width=Simulator.randbetween(.2, 1.2), height=2, angle=Simulator.randbetween(0, 360), color='purple'))

        self.robots = Robot(env)
        self.build(self.robots)

        # print(type(self.robots))
        # print(dir(self.robots))

        self.map = Map(env, self.robots, boxes=self.obstacles, definition=definition, lidar_range=lidar_range,
                       wrld_size=wrld_size,humans=self.humans, map_update_rate=map_update_rate,
                       global_map_init=global_map_init, c_space_dilation=c_space_dilation, human_radius=human_radius,
                       use_global_knowledge=use_global_knowledge)

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
            return None, None  # done

        target_x, target_y = self.waypoints[self.current_waypoint_index]
        dx = target_x - robot['center'].x
        dy = target_y - robot['center'].y
        distance = np.sqrt(dx**2 + dy**2)

        if distance < 3:  # update to next waypoint
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.reached_destination = True
                return None, None
            else:
                target_x, target_y = self.waypoints[self.current_waypoint_index]

        return target_x, target_y



def act(t, robot, controller):
    target = controller.update_target(robot)
    if target == (None, None):
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

    # shortest angle to the target
    angle_diff = (angle_to_target - robot_angle + 180) % 360 - 180
    print("Robot Angle Difference", angle_diff)  

    # speed stuff
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
    if distance_to_target < 3: 
        print("Robot at the target")  
        print("speed", speed)  
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

if __name__ == "__main__":
    wrld_size = [50,50]
    env = Environment(wrld_size[0], wrld_size[1])
    num_robots = 1
    num_humans = 2
    num_obstacles = 20
    sim = Simulator(env, rand_obstacles=20,wrld_size=wrld_size, num_humans=num_humans, global_map_init=True ,use_global_knowledge=True)
    map = sim.map
    waypoints = map.path_plan("PRM")
    # waypoints = [(3, 2), (10, 2), (15, 15), (20, 20)]

    if waypoints:
        controller = TrajectoryController(waypoints)
        run_sim(env, lambda t, robot: act(t, robot, controller), total_time=400, dt_display=10)
    else:
        print("No valid waypoints generated, simulation will not start.", waypoints)
# NOTE : If you are looking for a function call or data, do print(dir(data)) and for type do print(type(data))