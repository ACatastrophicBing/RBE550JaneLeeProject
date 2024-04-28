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
from RRT import RRT
class Map:
    def __init__(self, env, robot, goal, boxes=[], humans=[],definition=[100,100], wrld_size=[50,50], lidar_range=5.0,
                 map_update_rate = 100, global_map_init = True, c_space_dilation = 1.0, human_radius = 0.5,
                 use_global_knowledge = False, max_obj_size=12, visualize = False):
        self.env = env               # The environment we are in
        self.robot = robot           # How many actors we have in the environment that are mapping
        self.humans = humans         # How many random movement agents we have in the environment
        self.human_size = human_radius
        self.definition = definition # How detailed of a map we want
        self.definition_conversion = np.asarray([(definition[0]-1)/wrld_size[0], (definition[1]-1)/wrld_size[1]])
        self.lidar_range = int(lidar_range * self.definition_conversion[0])
        self.wrld_size = wrld_size
        self.lidar_range_mod = max_obj_size / 2 * self.definition_conversion[0]
        self.boxes = [box.body for box in boxes]
        self.cell_map = []
        self.human_size_map = []
        self.global_map_init = global_map_init
        self.c_space_dilation = c_space_dilation
        self.use_global_knowledge = use_global_knowledge
        self.goal = self.world_to_map(goal)

        self.robot_position = self.world_to_map(self.robot['center'].position)
        self.create_cell_maps()
        self.robot_flag = True # If the robot updates its position in world, flag this
        self.map_flag = True   # If the map has any changes compared to what it previously was, flag this
        self.cell_map_update = 0
        self.map_update_rate = map_update_rate # How often we check the map to update things, map will also update if robot moves

        self.box_angles = np.empty([len(self.boxes)], dtype=float)
        self.box_positions = np.empty([len(self.boxes),2], dtype=int)
        self.box_vertices = np.empty([len(self.boxes),4,2], dtype=float)
        self.points = np.zeros([len(self.boxes) * 4, 2], dtype=int)
        self.visualize = visualize
        for box in range(len(self.boxes)):
            if self.boxes[box].fixtures[0].shape.type == 2:
                self.box_vertices[box] = np.asarray(self.boxes[box].fixtures[0].shape.vertices)
            if self.boxes[box].fixtures[0].shape.type == 0:
                self.box_vertices[box] = self.boxes[box].fixtures[0].shape.radius * self.definition_conversion[0]

        self.human_positions = []
        for human in self.humans:
            self.human_positions.append(self.world_to_map(human.position))

        self.map = np.zeros((definition[0], definition[1]), dtype=bool) # 0 is empty 1 is blocked

        self.frontier_map = np.full(self.map.shape, 1, dtype=bool) # Is this efficient? No, but this'll be needed later

        self.brute_force_init()

        if global_map_init:
            print("[MAP] Initializing Map Knowing Global Snapshot")
            # This cannot actually be done here since the environment hasn't been generated yet, so it is instead handled in the update function ONCE
        else:
            print("[MAP] Initializing Map NOT Knowing Global Snapshot")
            self.robot_map = np.full(self.map.shape, 0, dtype=bool)

        self.last_map_update = -1000

        self.robot_cspace = np.zeros(self.map.shape, dtype=bool)

    def brute_force_init(self):
        for i in range(len(self.boxes)):
            verts = len(self.box_vertices[i])
            self.box_angles[i] = self.boxes[i].angle
            self.box_positions[i] = self.world_to_map(self.boxes[i].position)
            if self.boxes[i].fixtures[0].shape.type == 2:
                for vert in range(verts):
                    self.generate_obstacle_lines(self.box_positions[i],
                                                 self.world_to_map(self.rotate(self.box_vertices[i][vert % verts],
                                                                               self.box_angles[i])),
                                                 self.world_to_map(self.rotate(self.box_vertices[i][(vert + 1) % verts],
                                                                               self.box_angles[i])))
            if self.boxes[i].fixtures[0].shape.type == 0:
                # print(self.box_positions[i])
                self.generate_circle_radius(self.box_positions[i], self.box_vertices[i][0][0], True)


    def update(self, tick):
        # for robot in self.robots:
        #     continue

        if int(tick * 1000) - self.last_map_update > self.map_update_rate and self.use_global_knowledge:
            # print("Updating Global Map")
            self.update_global_map()
            self.last_map_update = tick
            self.map_flag = True
        else:
            self.map_flag = False

        perceived_position = self.world_to_map(self.robot['center'].position)
        if perceived_position != self.robot_position:
            self.robot_flag = True
            self.robot_position = perceived_position

        if self.global_map_init and tick == 0:
            print("Initializing the Robot map based on global current belief")
            self.robot_map = np.copy(self.map)
            self.global_map_init = False
            self.last_map_update = tick

        if int(tick * 1000) - self.last_map_update > self.map_update_rate and self.robot_flag and not self.use_global_knowledge and not self.global_map_init:
            self.last_map_update = tick
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
                        if math.sqrt((self.robot_position[0] - line1[0][i])**2 + (self.robot_position[1] - line1[1][i])**2):
                            self.robot_map[line1[0][i]][line1[1][i]] = self.map[line1[0][i]][line1[1][i]]
                    line2 = ski.draw.line(self.points[vert_loc][0], self.points[vert_loc][1], p2[0], p2[1])
                    for i in range(len(line2[:][0])):
                        if math.sqrt((self.robot_position[0] - line2[0][i])**2 + (self.robot_position[1] - line2[1][i])**2):
                            self.robot_map[line2[0][i]][line2[1][i]] = self.map[line2[0][i]][line2[1][i]]

                for i in range(len(self.humans)):  # These dudes move every time step so you have to update the global map
                    if self.human_positions[i] != self.world_to_map(self.humans[i].position):
                        for pos in self.human_size_map:
                            x = pos[0] + self.human_positions[i][0]
                            y = pos[1] + self.human_positions[i][1]
                            if 0 <= x < self.definition[0] and 0 <= y < self.definition[1] and math.sqrt((self.robot_position[0] - x)**2 + (self.robot_position[1] - y)**2):
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

                if self.boxes[i].fixtures[0].shape.type == 2:
                    for vert in range(verts):
                        self.remove_obstacle_lines(self.box_positions[i],
                                    self.world_to_map(self.rotate(self.box_vertices[i][vert%verts][:], self.box_angles[i])),
                                    self.world_to_map(self.rotate(self.box_vertices[i][(vert+1)%verts], self.box_angles[i])))
                if self.boxes[i].fixtures[0].shape.type == 0:
                    self.generate_circle_radius(self.box_positions[i], self.box_vertices[i][0][0], False)

                self.box_angles[i] = self.boxes[i].angle
                self.box_positions[i] = self.world_to_map(self.boxes[i].position)

                if self.boxes[i].fixtures[0].shape.type == 2:
                    for vert in range(verts):
                        # print("Adding a line for box ", self.box_positions[i], " from ",
                        #       self.world_to_map(self.rotate(self.box_vertices[i][vert % verts],self.box_angles[i])), " to ",
                        #       self.world_to_map(self.rotate(self.box_vertices[i][(vert + 1) % verts],self.box_angles[i])))
                        self.generate_obstacle_lines(self.box_positions[i],
                                    self.world_to_map(self.rotate(self.box_vertices[i][vert % verts],self.box_angles[i])),
                                    self.world_to_map(self.rotate(self.box_vertices[i][(vert + 1) % verts],self.box_angles[i])))

                if self.boxes[i].fixtures[0].shape.type == 0:
                    self.generate_circle_radius(self.box_positions[i], self.box_vertices[i][0][0], True)

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

    def generate_circle_radius(self, position, radius, add_to_map):
        n = math.ceil(2 * radius * math.pi) + 1
        for i in range(n):
            # print(position)
            # print(math.ceil(math.cos(2 * math.pi / n * (i + 1)) * radius) + position[0], math.ceil(math.sin(2 * math.pi / n * (i + 1)) * radius) + position[0])
            self.map[math.ceil(math.cos(2 * math.pi / n * (i + 1)) * radius) + position[0], math.ceil(math.sin(2 * math.pi / n * (i + 1)) * radius) + position[1]] = add_to_map



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


    def path_plan(self, algorithm):
        '''
        Takes in a string denoting what algorithm the robot is running.

        return : A path that is either a np array or a list of positions to go to, the return is in World coordinates,
        not map coordinates.
        '''
        if self.use_global_knowledge:
            self.robot_cspace = np.asarray(isotropic_dilation(self.map,
                                                   radius=int(self.c_space_dilation * self.definition_conversion[0])))
        else:
            self.robot_cspace = np.asarray(isotropic_dilation(self.robot_map,
                                                   radius=int(self.c_space_dilation * self.definition_conversion[0])))

        path = None

        if algorithm == "PRM":
            self.PRM = PRM(self.robot_cspace) # NOTE : You don't need to use self.Whatever,
            # I'm just using it because it was useful to do when debugging all the other code.
            self.PRM.sample(n_pts=500, sampling_method="random")
            self.PRM.search(self.robot_position, self.goal)
            if self.PRM.path:
                node_path = self.PRM.path
                path =  np.zeros([len(node_path), 2]) # Removes the START node
                path[0] = np.divide(np.asarray(self.robot_position, dtype=float), self.definition_conversion)
                for i in range(1, len(node_path)):
                    if isinstance(node_path[i], int):
                        path[i] = np.divide(np.asarray(self.PRM.samples[node_path[i]], dtype=float), self.definition_conversion)
                    else:
                        path[i] = np.divide(np.asarray(self.goal, dtype=float), self.definition_conversion)
                if self.visualize:
                    print("[MAP] Visualizing Map")
                    self.PRM.draw_map()
        elif algorithm == "RRT" or algorithm == "RRT*":
                rrt = RRT(self.robot_cspace, self.robot_position, self.goal)
                if algorithm == "RRT":
                    print('Starting RRT')
                    rrt.RRT(n_pts=5000)  
                else:
                    print('Starting RRT*')
                    rrt.RRT_star(n_pts=5000, neighbor_size=20)  
                if rrt.found:
                    # print('Path found')
                    path = []
                    current = rrt.goal
                    while current.parent is not None:
                        path.append((current.row, current.col))
                        current = current.parent
                    path.append((current.row, current.col))  # Add the start point
                    path.reverse() # Reverse 
                    # print('path',path)
                    # convert map 
                    rrt_height = 1000 
                    sim_width, sim_height = 50, 50


                    # This is chanching the map to world
                    def transform_coordinates(rrt_point, rrt_height, sim_width, sim_height):
                        # invert y 
                        sim_x = rrt_point[1] * sim_width / rrt_height
                        sim_y = (rrt_height - rrt_point[0]) * sim_height / rrt_height
                        return sim_x, sim_y

                    transformed_path = [transform_coordinates(point, rrt_height, sim_width, sim_height) for point in path]
                    # print('Transformed path:', transformed_path)
                    return transformed_path
                else:
                    print("No path found using", algorithm)
                    return None
        if algorithm == "AD*" and self.robot_flag:
            path = None
        return path


class Simulator:
    def __init__(self, env, goal, rand_obstacles=0, map_selector=None,wrld_size=[50,50], num_humans=0,
                 lidar_range=5.0, map_update_rate = 100, global_map_init = True, c_space_dilation = 1.0,
                 human_radius = 0.5, use_global_knowledge = False, definition = [1000,1000], human_friction=0.7, visualize=False):
        self.env = env
        self.robots = []
        self.obstacles = []
        self.humans = []
        self.goal = goal
        for i in range(num_humans):
            self.humans.append(Disk(self.env, x=Simulator.randbetween(8, 22), y=Simulator.randbetween(8, 22),
                    radius=0.5, angle=Simulator.randbetween(0, 360), color='red', linearDamping=human_friction))

        if map_selector is not None:
            # TODO : Create maps to allow robots to properly explore environment
            obstacle_color = 'purple'
            if map_selector == 0: # Warehouse
                self.obstacles.append(Wall(self.env, x=13.0, y=4.0, width=20, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=13.0, y=10.0, width=20, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=13.0, y=16.0, width=20, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=13.0, y=22.0, width=20, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=37.0, y=6.0, width=20, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=37.0, y=12.0, width=20, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=37.0, y=18.0, width=20, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=37.0, y=24.0, width=20, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=30.0, y=34.5, width=2, height=11, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=35.0, y=33.0, width=2, height=6, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=37.0, y=46.5, width=26, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=15.5, y=46.5, width=5, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=12.5, y=48.0, width=1, height=4, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=3.5, y=48.0, width=1, height=4, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=5.5, y=49.5, width=3, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=7.5, y=48.0, width=1, height=4, angle=0, color=obstacle_color))
                self.obstacles.append(Disk(self.env,x=1.0,y=26.0,angle=0,radius=2.0,density=1000,color=obstacle_color))
                self.obstacles.append(Disk(self.env,x=6.0,y=26.0,angle=0,radius=2.0,density=1000,color=obstacle_color))
                self.obstacles.append(Disk(self.env,x=3.0,y=28.0,angle=0,radius=2.0,density=1000,color=obstacle_color))
                self.obstacles.append(Disk(self.env,x=9.0,y=28.0,angle=0,radius=2.0,density=1000,color=obstacle_color))
                self.obstacles.append(Disk(self.env,x=6.0,y=31.0,angle=0,radius=2.0,density=1000,color=obstacle_color))
                self.obstacles.append(Disk(self.env,x=37.5,y=27.5,angle=0,radius=1.5,density=1000,color=obstacle_color))
                self.obstacles.append(Disk(self.env,x=39.5,y=26.5,angle=0,radius=1.5,density=1000,color=obstacle_color))
                self.obstacles.append(Disk(self.env,x=40.5,y=28.5,angle=0,radius=1.5,density=1000,color=obstacle_color))
                self.obstacles.append(Disk(self.env,x=39.0,y=37.0,angle=0,radius=2.0,density=1000,color=obstacle_color))
                self.obstacles.append(Disk(self.env,x=43.0,y=34.0,angle=0,radius=2.0,density=1000,color=obstacle_color))
                self.obstacles.append(Disk(self.env,x=43.0,y=38.0,angle=0,radius=2.0,density=1000,color=obstacle_color))
            if map_selector == 1: # Labyrinth
                self.obstacles.append(Wall(self.env, x=20.5, y=25.5, width=1, height=7, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=28.5, y=25.5, width=1, height=7, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=37.5, y=49, width=25, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=49.5, y=24, width=1, height=48, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=18.5, y=38.5, width=13, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=37.5, y=25, width=1, height=28, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=32.5, y=25, width=1, height=16, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=26.5, y=33.5, width=5, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=41.5, y=6.5, width=3, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=42.5, y=25.5, width=1, height=37, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=22, y=43.5, width=14, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=22, y=38.5, width=22, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=10.5, y=32.5, width=1, height=13, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=10.5, y=17, width=1, height=12, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=24, y=11.5, width=26, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=26.5, y=28.5, width=5, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=32, y=16.5, width=2, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=22, y=16.5, width=14, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=15.5, y=25, width=1, height=16, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=24, y=33.5, width=18, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=21, y=6.5, width=32, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=5.5, y=25.5, width=1, height=37, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=24, y=43.5, width=36, height=1, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=25, y=1, width=48, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=0.5, y=24, width=1, height=48, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=9.5, y=49, width=19, height=2, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=24.5, y=39.5, width=1, height=21, angle=0, color=obstacle_color))
                self.obstacles.append(Wall(self.env, x=24.5, y=21.5, width=9, height=1, angle=0, color=obstacle_color))
        else:
            for _ in range(rand_obstacles):
                self.obstacles.append(Box(self.env, x=Simulator.randbetween(8, 42), y=Simulator.randbetween(8, 42), width=Simulator.randbetween(.2, 2.5), height=2, angle=Simulator.randbetween(0, 360), color='purple'))

        self.robots = Robot(env)
        self.build(self.robots)

        # print(type(self.robots))
        # print(dir(self.robots))

        self.map = Map(env, self.robots, goal, boxes=self.obstacles, definition=definition, lidar_range=lidar_range,
                       wrld_size=wrld_size,humans=self.humans, map_update_rate=map_update_rate,
                       global_map_init=global_map_init, c_space_dilation=c_space_dilation, human_radius=human_radius,
                       use_global_knowledge=use_global_knowledge, visualize=visualize)

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

