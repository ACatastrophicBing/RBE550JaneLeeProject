import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
import heapq
import numpy as np
import networkx as nx

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, is_dy_obs):
        self.row = row             # coordinate
        self.col = col             # coordinate
        self.is_obs = is_obs       # obstacle?
        self.is_dy_obs = is_dy_obs # dynamic obstacle?
        self.tag = "NEW"           # tag ("NEW", "OPEN", "CLOSED")
        self.h = math.inf          # cost to goal (NOT heuristic)
        self.k = math.inf          # best h
        self.parent = None         # parent node
        self.g = math.inf
        self.rhs = math.inf
        self.visited = False

class AnytimeDynAStar:
    def __init__(self, grid, dynamic_grid, start, goal):
        # Maps
        self.grid = grid                  # the pre-known grid map
        self.dynamic_grid = dynamic_grid  # the actual grid map (with dynamic obstacles)

        G_grid = nx.from_numpy_array(grid)
        G_dynGrid = nx.from_numpy_array(dynamic_grid)

        # Create a new grid to store nodes
        size_row = len(grid)
        size_col = len(grid[0])
        self.grid_node = [[None for i in range(size_col)] for j in range(size_row)]
        for row in range(size_row):
            for col in range(size_col):
                self.grid_node[row][col] = self.instantiate_node((row, col))

        # The start node
        self.start = self.grid_node[start[0]][start[1]]
        self.start.g = math.inf
        self.start.rhs = math.inf
        # The goal node
        self.goal = self.grid_node[goal[0]][goal[1]]
        self.goal.g = math.inf
        self.goal.rhs = 0
        
        self.epsilon_0 = 0.5
        self.epsilon = self.epsilon_0 # Determine a value
        self.epsilon_delta = 0.1

        self.openList = []
        self.closedList = []
        self.incosisList = []

        self.insertOpen(self.goal, self.calculateKey(self.goal))
        heapq.heapify(self.openList)

        self.path = []

    #  Helper Functions   

    def instantiate_node(self, point):
        row, col = point
        node = Node(row, col, self.grid[row][col], self.dynamic_grid[row][col])
        return node
    
    def get_neighbors(self, node):
        row = node.row
        col = node.col
        neighbors = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                if row + i < 0 or row + i >= len(self.grid) or \
                   col + j < 0 or col + j >= len(self.grid[0]):
                    continue
                if i == 0 and j == 0:
                    continue
                neighbors.append(self.grid_node[row + i][col + j])
        return neighbors
    
    def cost(self, node1, node2):
        if node1.is_obs or node2.is_obs:
            return math.inf
        a = node1.row - node2.row
        b = node1.col - node2.col
        return (a**2 + b**2) ** (1/2)
    
    def calc_g(self, node):
        node.g = self.cost(node, self.start)
        return node.g
    
    def calc_h(self, node):
        node.h = self.cost(node, self.goal)
        return node.h
    
    def calc_rhs(self, node):
        if node == self.goal:
            return 0
        
        min_rhs = math.inf
        for successor in self.get_neighbors(node):
            if not successor.is_obs:
                rhs_candidate = self.calc_g(successor) + self.cost(node, successor)
                min_rhs = min(min_rhs, rhs_candidate)
        
        node.rhs = min_rhs
        return node.rhs

    def insertOpen(self, node, keys):
        k1, k2 = keys
        heapq.heappush(self.openList, (k1, k2, node))

    def updateKey(self, node, k1, k2):
        self.removeOpen(node)
        self.insertOpen(node, (k1, k2))

    def removeOpen(self, node):
        self.openList = [(k1, k2, n) for k1, k2, n in self.openList if n != node]
        heapq.heapify(self.openList)

    def removeInconsis(self, node):
        self.incosisList = [(k1, k2, n) for k1, k2, n in self.incosisList if n != node]
        heapq.heapify(self.incosisList)

    def topNode(self): # Return min priority node
        if self.openList:
            node = self.openList[0][2]
            return node
        else:
            return None
    
    def topKey(self): # Return smallest key pair
        if self.openList:
            keys = (self.openList[0][0], self.openList[0][1])
            return keys
        else:
            return (math.inf, math.inf)
        
    def popNode(self): # Pop node of min priority
        if self.openList:
            return heapq.heappop(self.openList)[2]
        else:
            return None
        
    def compareKeys(self, keyPair1, keyPair2):
        if keyPair1[0] < keyPair2[0]:
            return -1
        elif keyPair1[0] > keyPair2[0]:
            return 1
        else:
            if keyPair1[1] < keyPair2[1]:
                return -1
            elif keyPair1[1] > keyPair2[1]:
                return 1
            else:
                return 0
    
    def calculateKey(self, node):
        self.calc_g(node)
        self.calc_h(node)
        self.calc_rhs(node)

        if node.g > node.rhs:
            keys = ((node.rhs + self.epsilon * node.h), node.rhs)
        else:
            keys = ((node.g + node.h), node.g)
        return keys
    
    def extract_path(self):
        state = self.start
        path = []
        path.append(state)

        while True:
            cost_list = {}
            for neighbor in self.get_neighbors(state):
                if not neighbor.is_obs:
                    cost_list[neighbor] = self.calc_g(neighbor)
            print("costlist : ", cost_list)
            path.append(min(cost_list, key=cost_list.get))
            if state == self.goal:
                break

        return path
    
    def updateState(self, node):
        if not node.visited:
            node.g = math.inf
        if node != self.goal:
            node.rhs = self.calc_rhs(node)
        if node in self.openList:
            self.removeOpen(node)
        if (self.calc_g(node) != self.calc_rhs(node)):
            if node not in self.closedList:
                self.insertOpen(node, self.calculateKey(node))
            else:
                self.incosisList.append(node)

    def computeShortestPath(self):
        while (self.compareKeys(self.topKey(), self.calculateKey(self.start)) == -1) or (self.calc_rhs(self.start) != self.calc_g(self.start)):
            node = self.popNode()
            if node.g > node.rhs:
                node.g = node.rhs
                self.closedList.append(node)
                neighbors = self.get_neighbors(node)
                for neighbor in neighbors:
                    if not neighbor.is_obs:
                        self.updateState(neighbor)
            else:
                node.g = math.inf
                neighbors = self.get_neighbors(node)
                for neighbor in neighbors:
                    if not neighbor.is_obs:
                        self.updateState(neighbor)
                self.updateState(node)

    def onFlag(self, dynamic_grid):
        self.dynamic_grid = dynamic_grid
        tol = 5
        changed_list = np.argwhere(np.logical_xor(self.grid, self.dynamic_grid))
        quant_cost_change = len(changed_list)
        node_changed_list = [self.grid_node[node_pos[0]][node_pos[1]] for node_pos in changed_list]
        for node in node_changed_list:
            cost = self.cost(self.robot_position, node) # Where to store this?
            self.updateState(node)
        if quant_cost_change > tol:
            self.epsilon += self.epsilon_delta
            self.computeShortestPath() 
            return self.extract_path()
        elif self.epsilon > 1:
                self.epsilon += -1*self.epsilon_delta
        for node in self.incosisList:
            self.removeInconsis(node)
            self.insertOpen(node, self.calculateKey(node))
        for tuple in self.openList:
            _, _, node = tuple
            k1, k2 = self.calculateKey(node)
            self.updateKey(node,k1,k2)
        self.closedList.clear()
        self.computeShortestPath()
        self.grid = dynamic_grid
        return self.extract_path()

    # def run(self):
    #     self.computeShortestPath()
    #     # Publish current sub-optimal solution
    #     while True:
    #         # Determine if visible cost change
    #         # cost_change, edge_list = self.check_maps(robot_map_in_FOV, global_map)
    #         if cost_change:
    #             ## List directed edges (u,v) with changed edge costs and update traversal costs
    #             for node1, node2 in edge_list:
    #                 cost = self.cost(node1, node2) # Where to store this?
    #                 self.updateState(node1)
    #         # if value_cost_change > tol:
    #             # self.epsilon += self.epsilon_delta
    #             # OR replan
    #         elif self.epsilon > 1:
    #             self.epsilon += -1*self.epsilon_delta
    #         for node in self.incosisList:
    #             self.incosisList.remove(node)
    #             self.insertOpen(node, self.calculateKey(node))
    #         for node, _ in self.openList:
    #             k1, k2 = self.calculateKey(node)
    #             self.updateKey(node,k1,k2)
    #         self.closedList.clear()
    #         self.computeShortestPath()
    #         # Publish current suboptimal solution
    #         if self.epsilon == 1:
    #             pass # Eliminate this
    #             # wait for changes in edge costs