# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy.spatial import distance
from scipy.spatial import KDTree
import random


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost
        


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = np.rot90(map_array)            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(map_array.shape[1] - 1 - start[1], start[0])
        self.goal = Node(map_array.shape[1] - 1 - goal[1], goal[0])

        # self.start = Node(start[0], start[1]) # start node
        # self.goal = Node(goal[0], goal[1])    # goal node
   # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        self.sampling_method = ""          # sampling method added to show the name of the algorithm
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    #done same as PRM
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        # Same as PRM implementation but with nodes instead of points
        # Create a line between p1 and p2 
        distance = np.sqrt((node1.row - node2.row)**2 + (node1.col - node2.col)**2)
        return distance

    #Done same as PRM
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        # Taken from my RPM algorith simmilar to dis 
        # Create a line between p1 and p2 
        # get the the nodes and make them p1 and p2 
        p1 = np.array([node1.row, node1.col])
        p2 = np.array([node2.row, node2.col])

        # Create a line between p1 and p2
        line = np.linspace(p1, p2, num=100, endpoint=True)

        # rest is same as RPM 
        for point in line:
            x, y = int(point[0]), int(point[1])
            # I had to add this to check if the given point is within the map otherwise it will give out of index error
            if x >= 0 and y >= 0 and x < self.size_row and y < self.size_col:
                if self.map_array[x, y] == 1:  # Check obstacles
                    return True  # Collision detected
        return False  # No collision detected

    # DONE 
    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        if random.random() < goal_bias:
            return self.goal
        else:
            return Node(random.randint(0, self.size_row-1), random.randint(0, self.size_col-1))



    # DONE
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###

        node_near = self.vertices[0]
        # Get the distance of the nearest node
        dist = self.dis(point, node_near)
        # Loop through all the nodes
        for node in self.vertices:
            # If the distance is less than the current distance
            if self.dis(point, node) < dist:
                node_near = node
                # Update the distance
                dist = self.dis(point, node)
        return node_near


    # Done 
    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        #neighbors list 
        neighbors = []
        # Loop through all the nodes
        for node in self.vertices:
            # If the distance is less than the neighbor size
            if self.dis(new_node, node) < neighbor_size:
                # Add the node to list created 
                neighbors.append(node)
        return neighbors



    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        # Loop through all the neighbors (given as a list of nodes)
        for neighbor in neighbors:
            # Rewire the new node if connecting to a new neighbor node will give least cost. and check for the collision
            if not self.check_collision(new_node, neighbor) and ((new_node.cost + self.dis(new_node, neighbor)) < neighbor.cost):
                # Update the parent of the neighbor
                neighbor.parent = new_node
                # Update the cost of the neighbor
                neighbor.cost = new_node.cost + self.dis(new_node, neighbor)



    # Edited to print the name of the algorithm 
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        arr = 1 - self.map_array
        img = 255 * np.dstack((arr, arr, arr))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        plt.title(f"RRT Pathfinding using {self.sampling_method} ")


        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=2000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()
        self.sampling_method = "RRT"
        self.goal.cost = 0.0
        self.vertices = [self.start]

        ### YOUR CODE HERE ###
        for _ in range(n_pts):
            # get a newpoint
            rand_node = self.get_new_point(0.1)
            # get newpoint's nearest node
            nearest_node = self.get_nearest_node(rand_node)

            step_size = 8 
            # get the angle of the new point and the nearest node
            theta = np.arctan2(rand_node.row - nearest_node.row, rand_node.col - nearest_node.col)
            # get the new point
            new_row = nearest_node.row + np.sin(theta) * step_size
            new_col = nearest_node.col + np.cos(theta) * step_size
            new_node = Node(new_row, new_col)

            # Check if this new node can be added
            if not self.check_collision(nearest_node, new_node):
                # Set the new node's parent to the nearest node and update its cost
                new_node.parent = nearest_node
                # Update the cost of the new node
                new_node.cost = nearest_node.cost + self.dis(nearest_node, new_node)
                # Add the new node to the list of vertices
                self.vertices.append(new_node)

                # Check if we are are nearby the goal and the step size is less than the distance to the goal
                # consider we will reach the goal and stop the search from now on with updating the goal's parent and cost
                if self.dis(new_node, self.goal) < step_size:
                    self.goal.parent = new_node
                    # conisder the overall goal and the final cost from the node to the goal 
                    self.goal.cost = new_node.cost + self.dis(new_node, self.goal)
                    self.found = True
                    break


        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        # self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()
        self.sampling_method = "RRT*"
        self.goal.cost = 0.0
        self.vertices = [self.start]

        ### YOUR CODE HERE ###
        for _ in range(n_pts):
             # get a newpoint
            rand_node = self.get_new_point(0.1)
            # get newpoint's nearest node
            nearest_node = self.get_nearest_node(rand_node)

            step_size = 10  
            # get the angle of the new point and the nearest node
            theta = np.arctan2(rand_node.row - nearest_node.row, rand_node.col - nearest_node.col)
            # get the new point
            new_row = nearest_node.row + np.sin(theta) * step_size
            new_col = nearest_node.col + np.cos(theta) * step_size
            new_node = Node(new_row, new_col)

            if not self.check_collision(nearest_node, new_node):

                # Set the new node's parent to the nearest node and update its cost
                new_node.parent = nearest_node
                # Update the cost of the new node
                new_node.cost = nearest_node.cost + self.dis(nearest_node, new_node)
                # get the negihbors of the new node to rewire it 
                neighbors = self.get_neighbors(new_node, neighbor_size)

                # Loop the found neighbors of the new node to find the best parent and update the cost
                for neighbor in neighbors:
                    if not self.check_collision(new_node, neighbor) and ((neighbor.cost + self.dis(neighbor, new_node)) < new_node.cost):
                        # Update the parent of the new node
                        new_node.parent = neighbor
                        new_node.cost = neighbor.cost + self.dis(neighbor, new_node)

                # Add the new node to the list of vertices same as RRT  
                self.vertices.append(new_node)

                # Rewire the tree
                self.rewire(new_node, neighbors)

                # Check if the new node is close enough to the goal
                if self.dis(new_node, self.goal) < step_size:
                    if not self.found or new_node.cost + self.dis(new_node, self.goal) < self.goal.cost:
                        self.goal.parent = new_node
                        self.goal.cost = new_node.cost + self.dis(new_node, self.goal)
                        self.found = True

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        # self.draw_map()