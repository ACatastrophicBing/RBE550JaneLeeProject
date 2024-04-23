# Standard Algorithm Implementation
# Sampling-based Algorithms PRM
import math

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import skimage as ski
from scipy import spatial


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array, neighborhood_search = 0.1, gaussian_search = 0.1):
        self.map_array = map_array            # map array, 0->free, 1->obstacle
        self.map_width, self.map_height = map_array.shape
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path
        self.neighborhood_search = neighborhood_search # The size in reference to the % of width of map which we will
        # search for neighbors, default is 10%, also known as dist in the algorithm
        self.gaussian_search = gaussian_search # The range in which to search for gaussian sampling in % map width


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1 index in self.samples
            p2 - point 2 index in self.samples

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###)
        point1 = self.samples[p1]
        point2 = self.samples[p2]
        line = ski.draw.line(point1[0], point1[1], point2[0], point2[1]) # ndarray of int, Indices of pixels that belong to the line
        for i in range(len(line[:][0])):
            if self.map_array[line[0][i], line[1][i]]:
                return True
        return False


    def dis(self, p1, p2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1 index in self.samples
            p2 - point 2 index in self.samples

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        # Calculate the sum of squared differences
        point1 = self.samples[p1]
        point2 = self.samples[p2]
        squared_diffs = [(a - b) ** 2 for a, b in zip(point1, point2)]
        return math.sqrt(sum(squared_diffs))


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, key word TRY
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        n_pts_w = math.floor(math.sqrt(n_pts) * (self.map_width / self.map_height))
        n_pts_h = math.floor(math.sqrt(n_pts) * (self.map_height / self.map_width))
        samples = [(math.floor(self.map_width / n_pts_w * i), math.floor(self.map_height / n_pts_h * j))
                   for i in range(n_pts_w) for j in range(n_pts_h)
                   if not self.map_array[math.floor(self.map_width / n_pts_w * i), math.floor(self.map_height / n_pts_h * j)]]
        self.samples = np.asarray(samples)

    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        # Generate a random point within the map boundaries from discrete uniform sampling
        x = np.random.randint(0, self.map_width, n_pts)
        y = np.random.randint(0, self.map_height, n_pts)
        samples = [[x[i], y[i]] for i in range(n_pts) if not self.map_array[x[i], y[i]]]
        self.samples = np.asarray(samples)


    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()
        samples = []
        obstacle_cells = [(i, j) for i in range(self.map_width) for j in range(self.map_height) if self.map_array[i, j]]
        for sample in range(n_pts):
            rand_index = np.random.randint(0, len(obstacle_cells))
            x = int(np.random.normal(0,self.map_width*self.gaussian_search)) + obstacle_cells[rand_index][0]
            y = int(np.random.normal(0,self.map_height*self.gaussian_search)) + obstacle_cells[rand_index][1]
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if not self.map_array[x,y]:
                    samples.append((x,y))
        self.samples = np.asarray(samples)

    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()
        samples = []
        obstacle_cells = [(i, j) for i in range(self.map_width) for j in range(self.map_height) if self.map_array[i, j]]
        for sample in range(n_pts):
            rand_index = np.random.randint(0, len(obstacle_cells))
            dx = int(np.random.normal(0, self.map_width * self.gaussian_search))
            x = dx +  + obstacle_cells[rand_index][0]
            dy = int(np.random.normal(0, self.map_height * self.gaussian_search))
            y = dy + obstacle_cells[rand_index][1]
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if self.map_array[x, y]:
                    xx = math.floor(x - (dx/2))
                    yy = math.floor(y-(dy/2))
                    if not self.map_array[xx, yy]:
                        samples.append((xx, yy))
        self.samples = np.asarray(samples)


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        arr = 1 - self.map_array
        img = 255 * np.dstack((arr, arr, arr))
        img_rotated = np.rot90(img, k=1)  # Rotate 90 degrees counterclockwise
        ax.imshow(img_rotated)


        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        # Rotate positions for the display
        rotated_pos = {i: (pos[1], arr.shape[0] - pos[0]) for i, pos in enumerate(node_pos)}
        rotated_pos['start'] = (node_pos[-2][1], arr.shape[0] - node_pos[-2][0])
        rotated_pos['goal'] = (node_pos[-1][1], arr.shape[0] - node_pos[-1][0])

        # draw constructed graph
        nx.draw(self.graph, rotated_pos, node_size=3, node_color='y', edge_color='y', ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=rotated_pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=rotated_pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=rotated_pos, nodelist=['start'], node_size=12, node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=rotated_pos, nodelist=['goal'], node_size=12, node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###
        self.kdtree = spatial.KDTree(self.samples)
        sampled_pairs = list(self.kdtree.query_pairs(self.neighborhood_search * self.map_width))
        pairs = [[pair[0], pair[1], self.dis(pair[0], pair[1])] for pair in sampled_pairs if not self.check_collision(pair[0], pair[1])]

        # Use sampled points and pairs of points to build a graph.
        self.graph.add_nodes_from(range(len(self.samples)))
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###
        # Use existing KD tree to search area without increasing runtime complexity

        self.samples = np.vstack((self.samples, np.asarray(start)))
        self.samples = np.vstack((self.samples, np.asarray(goal)))

        start_pairs = self.kdtree.query_ball_point(start, r=self.neighborhood_search * self.map_width)
        start_pears = [['start', pair, self.dis(len(self.samples) - 2, pair)] for pair in start_pairs if
                 not self.check_collision(len(self.samples) - 2, pair)]
        goal_pairs = self.kdtree.query_ball_point(goal, r=self.neighborhood_search * self.map_width)
        goal_pears = [['goal', pair, self.dis(len(self.samples) - 1, pair)] for pair in goal_pairs if
                       not self.check_collision(len(self.samples) - 1, pair)]


        if len(start_pears) < 2: # In case of emergency force a connection somehow because we can't be having a floater
            start_pairs = self.kdtree.query_ball_point(start, r=self.neighborhood_search * self.map_width*3)
            start_pears = [['start', pair, self.dis(len(self.samples) - 2, pair)] for pair in start_pairs if
                           not self.check_collision(len(self.samples) - 2, pair)]
        if len(goal_pears) < 2:
            goal_pairs = self.kdtree.query_ball_point(goal, r=self.neighborhood_search * self.map_width * 3)
            goal_pears = [['goal', pair, self.dis(len(self.samples) - 1, pair)] for pair in goal_pairs if
                          not self.check_collision(len(self.samples) - 1, pair)]


        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pears)
        self.graph.add_weighted_edges_from(goal_pears)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pears)
        self.graph.remove_edges_from(goal_pears)
        