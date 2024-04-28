import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
import heapq
import numpy as np

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

class DStarLite:
    def __init__(self, grid, dynamic_grid, start, goal):
        # Maps
        self.grid = grid                  # the pre-known grid map
        self.dynamic_grid = dynamic_grid  # the actual grid map (with dynamic obstacles)
        # Create a new grid to store nodes
        size_row = len(grid)
        size_col = len(grid[0])
        self.grid_node = [[None for i in range(size_col)] for j in range(size_row)]
        for row in range(size_row):
            for col in range(size_col):
                self.grid_node[row][col] = self.instantiate_node((row, col))

        # The start node
        self.start = self.grid_node[start[0]][start[1]]
        # The goal node
        self.goal = self.grid_node[goal[0]][goal[1]]
        
        self.open = set()
        self.queue = []
        self.km = 0
        self.goal.rhs = 0
        self.insert(self.goal, self.calculateKey(self.goal))
        heapq.heapify(self.queue)

        self.path = []


    #  Helper Functions   

    def instantiate_node(self, point):
        row, col = point
        node = Node(row, col, not self.grid[row][col], not self.dynamic_grid[row][col])
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
        node.g = self.cost(node, self.goal)
        return node.g
    
    def calc_h(self, node):
        node.h = self.cost(node, self.start)
        return node.h
    
    def calc_rhs(self, node):
        if node == self.goal:
            return 0
        
        min_rhs = math.inf
        for successor in self.get_neighbors(node):
            if not successor.is_obs:
                rhs_candidate = self.calc_g(successor) + self.cost(node, successor) + self.calc_h(node)
                min_rhs = min(min_rhs, rhs_candidate)
        
        node.rhs = min_rhs
        return node.rhs

    def insert(self, node, keys):
        k1, k2 = keys
        heapq.heappush(self.queue, (k1, k2, node))

    def updateKey(self, node, k1, k2):
        self.remove(node)
        self.insert(node, k1, k2)

    def remove(self, node):
        self.queue = [(k1, k2, n) for k1, k2, n in self.queue if n != node]
        heapq.heapify(self.queue)

    def topNode(self): # Return min priority node
        if self.queue:
            node = self.queue[0][2]
            return node
        else:
            return None
    
    def topKey(self): # Return smallest key pair
        if self.queue:
            keys = (self.queue[0][0:1])
            return keys
        else:
            return (math.inf, math.inf)
        
    def popNode(self): # Pop node of min priority
        if self.queue:
            return heapq.heappop(self.queue)[2]
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
        
    # D* Lite Functions
    
    def calculateKey(self, node):
        self.calc_g(node, self.start)
        self.calc_h(node, self.goal)
        self.calc_rhs(node)
        keys = (min(node.g, (node.rhs + node.h + self.km)), min(node.g, node.rhs))
        return keys
    
    def updateVertex(self, node):
        if node != self.goal:
            node.rhs = self.calc_rhs(node)
        if node in self.open:
            self.delete(node)
        if (self.calc_g(node) != self.calc_rhs(node)):
            self.insert(node, self.calculateKey(node))

    def computeShortestPath(self):
        while (self.compareKeys(self.topKey(), self.calculateKey(self.start)) == -1) or (self.calc_rhs(self.start) != self.calc_g(self.start)):
            kOld = self.topKey()
            node = self.popNode()
            nodeKey = self.calculateKey(node)
            if self.compareKeys(kOld, nodeKey) == -1:
                self.insert(node, nodeKey)
            elif self.calc_g(node) > self.calc_rhs(node):
                node.g = node.rhs
                neighbors = self.get_neighbors(node)
                for neighbor in neighbors:
                    if not neighbor.is_obs:
                        self.updateVertex(neighbor)
            else:
                node.g = math.inf
                self.updateVertex(node)
                neighbors = self.get_neighbors(node)
                for neighbor in neighbors:
                    if not neighbor.is_obs:
                        self.updateVertex(neighbor)

    def extract_path(self): # Same as from ADStar
        state = self.start
        path = []
        path.append(state)

        while True:
            cost_list = {}
            for neighbor in self.get_neighbors(state):
                if not neighbor.is_obs:
                    cost_list[neighbor] = self.calc_g(neighbor)
            path.append(min(cost_list, key=cost_list.get))
            if state == self.goal:
                break

        return path
    
    def firstPass(self):
        state = self.start
        self.computeShortestPath()
        if state != self.goal and self.start.g != math.inf:
            neighbors = self.get_neighbors(state)
            candidate_state = neighbors[0]
            for neighbor in neighbors:
                if (self.cost(state,neighbor) + neighbor.g) < self.cost(state,candidate_state) + candidate_state.g:
                    candidate_state = neighbor
            return candidate_state
    
    def onFlag(self, dynamic_grid): # on robot move
        self.dynamic_grid = dynamic_grid
        changed_list = np.argwhere(np.logical_xor(self.grid, self.dynamic_grid))
        self.grid = dynamic_grid
        quant_cost_change = len(changed_list)
        if quant_cost_change > 0:
            km = km + self.calc_h(state)
            state = self.robot_position #Reversal from psuedocode
            for node in changed_list:
            # cost = self.cost(self.robot_position, node) # Where to store this? Instead:
                node.g = self.calc_g(node)
                node.h = self.calc_h(node)
                node.rhs = self.calc_rhs(node)
                self.updateVertex(node)
            self.computeShortestPath()
        if state != self.goal and self.start.g != math.inf:
            neighbors = self.get_neighbors(state)
            candidate_state = neighbors[0]
            for neighbor in neighbors:
                if (self.cost(state,neighbor) + neighbor.g) < self.cost(state,candidate_state) + candidate_state.g:
                    candidate_state = neighbor
            return candidate_state

    # def run(self):
    #     state = self.start
    #     # __init__
    #     self.computeShortestPath()
    #     while state != self.goal and self.start.g != math.inf:
    #         neighbors = self.get_neighbors(state)
    #         candidate_state = neighbors[0]
    #         for neighbor in neighbors:
    #             if (self.cost(state,neighbor) + neighbor.g) < self.cost(state,candidate_state) + candidate_state.g:
    #                 candidate_state = neighbor
    #         state = candidate_state
    #         # Move robot to <state>
    #         # sim.move_robot(state)
    #         # Determine if visible cost change
    #         # cost_change, edge_list = self.check_maps(robot_map_in_FOV, global_map)
    #         if cost_change:
    #             km = km + self.calc_h(state)
    #             self.start = state # Reversal from psuedocode
    #             ## List directed edges (u,v) with changed edge costs and update traversal costs
    #             for node1, node2 in edge_list:
    #                 cost = self.cost(node1, node2) # Where to store this?
    #                 self.updateVertex(node1)
    #             # OR:
    #             # for node in edgeList:
    #             #     node.g = self.calc_g(node)
    #             #     node.h = self.calc_h(node)
    #             #     node.rhs = self.calc_rhs(node)
    #             self.computeShortestPath()

