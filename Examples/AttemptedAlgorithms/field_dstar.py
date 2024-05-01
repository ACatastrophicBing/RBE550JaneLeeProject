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
# D*Lite
        self.g = math.inf
        self.rhs = math.inf

class FieldDStar: # Remember that cells are a center with 8 edge points
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
        self.start.g = math.inf
        self.start.rhs = math.inf
        # The goal node
        self.goal = self.grid_node[goal[0]][goal[1]]
        self.goal.g = math.inf
        self.goal.rhs = 0
        
        self.queue = []
        self.insert(self.goal, self.key(self.goal))
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
    # This is now a interpolation cost, with knowledge of cell dimensions
    
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
        
    # Field D* Functions

    def computeCost(self, node, nodeA, nodeB):
        if nodeA.row != node.row and nodeA.col != node.col:
            node1 = nodeB
            node2 = nodeA
        else:
            node1 = nodeA
            node2 = nodeB
        # c is traversal cost of cell with corners node, node1, and node2
        c = math.sqrt((node2.row - node.row)**2 + (node2.col - node.col)**2)
        # b is traversal cost of cell with corners node, node1, but not node2
        b = abs(node1.col - node.col)
        if min(c,b) == math.inf:
            cost = math.inf
        elif node1.g <= node2.g:
            cost = min(c,b) + node1.g
        else:
            f = node1.g - node2.g
            if f <= b:
                if c <= f:
                    cost = c * math.sqrt(2) + node2.g
                else:
                    y = min(f/math.sqrt(c**2-f**2),1)
                    cost = c * math.sqrt(1 + y**2) + f * (1 - y) + node2.g
            else:
                if c <= b:
                    cost = c * math.sqrt(2) + node2.g
                else:
                    x = 1 - min(b/math.sqrt(c**2-b**2),1)
                    cost = c * math.sqrt(1 + (1 - x)**2) + b * x + node2.g
        return cost
        # I do not fully understand this function, all g cost calculation may have changed
    
    def key(self, node):
        self.calc_g(node, self.start)
        self.calc_h(node, self.goal)
        self.calc_rhs(node)
        keys = (min(node.g, (node.rhs + node.h)), min(node.g, node.rhs))
        return keys
    
    def updateNode(self, node):
        if not node.visited:
            node.g = math.inf
        if node != self.goal:
            connbrs = []
            row = node.row
            col = node.col
            try:
                connbrs.append((self.grid_node[row][col+1],self.grid_node[row-1][col+1]))
            except:
                pass
            try:
                connbrs.append((self.grid_node[row-1][col+1],self.grid_node[row-1][col]))
            except:
                pass
            try:
                connbrs.append((self.grid_node[row-1][col],self.grid_node[row-1][col-1]))
            except:
                pass
            try:
                connbrs.append((self.grid_node[row-1][col-1],self.grid_node[row][col-1]))
            except:
                pass
            try:
                connbrs.append((self.grid_node[row][col-1],self.grid_node[row+1][col-1]))
            except:
                pass
            try:
                connbrs.append((self.grid_node[row+1][col-1],self.grid_node[row+1][col]))
            except:
                pass
            try:
                connbrs.append((self.grid_node[row+1][col],self.grid_node[row+1][col+1]))
            except:
                pass
            try:
                connbrs.append((self.grid_node[row+1][col+1],self.grid_node[row][col+1]))
            except:
                pass
            candidate_node_a, candidate_node_b = connbrs[0]
            candidate_cost = self.computeCost(node,candidate_node_a,candidate_node_b)
            for item in connbrs:
                candidate_node_a, candidate_node_b = item
                if self.computeCost(node,candidate_node_a,candidate_node_b) < candidate_cost:
                    candidate_cost = self.computeCost(node,candidate_node_a,candidate_node_b)
        if node in self.open:
            self.remove(node)
        if node.g != node.rhs:
            self.insert(node,self.key(node))

    def computeShortestPath(self):
        while (self.compareKeys(self.topKey(), self.calculateKey(self.start)) == -1) or (self.calc_rhs(self.start) != self.calc_g(self.start)):
            node = self.popNode()
            if node.g < node.rhs:
                node.g = node.rhs
                neighbors = self.get_neighbors(node)
                for neighbor in neighbors:
                    self.updateNode(neighbor)
            else:
                node.g = math.inf
                neighbors = self.get_neighbors(node)
                for neighbor in neighbors:
                    self.updateNode(neighbor)
                self.updateNode(node)

    def onFlag(self):
        self.computeShortestPath()
        changed_list = np.argwhere(np.logical_xor(self.grid, self.dynamic_grid))
        quant_cost_change = len(changed_list)
        if quant_cost_change > 0:
            for node in changed_list:
                corners = self.getCorners(node) # How do we plan to do this?
                for corner in corners:
                    self.updateNode(corner)