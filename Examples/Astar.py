# Basic searching algorithms
import queue  
# NOTE: So I'm using queue here but i saw an example using heapq but queue is easier to use for me.
# Althoguh I'm not sure about the performance difference between the two for these functions.  I think it's not that significant for this assigment.
# But heapq is faster than queue for better more complex and larger usages. If the grid is larger and more complex, I would use heapq instead of queue.
# 
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv
import numpy as np 

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node


# Some same stuff from BFS and DFS
def invert_grid(grid):
    """ Invert grid values: Convert 0 to 1 and 1 to 0 """
    return [[1 if cell == 0 else 0 for cell in row] for row in grid]

def is_valid_position(grid, position):
    rows, cols = len(grid), len(grid[0])
    row, col = position
    return 0 <= row < rows and 0 <= col < cols

# Checks if the position is accaptable to start or end 
def is_valid_start_goal(grid, start, goal):
    return (is_valid_position(grid, start) and 
            is_valid_position(grid, goal) and 
            not grid[start[0]][start[1]] and 
            not grid[goal[0]][goal[1]])



# calculates the heuristic value form the given a to b 
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Since we are using the same structure for the nodes, we can use the same function to explore the nodes 
# the last parameter is the algorithm to be used, it can be either "dijkstra" or "astar"
# Based on the choice for the last paremater it updates the cost with hueristic or without 
# Similar to what I used in BFS and DFS
def exlpore_nodes(current_node, nodes, list_of_nodes, visited, counter, algorithm="dijkstra"):
    rows, cols = len(nodes), len(nodes[0])
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # right, down, left, up as given in the structures 
    for dr, dc in directions:
        #loop the directions to explore each nearby node
        r, c = current_node.row + dr, current_node.col + dc
        # check if the node is valid and not visited and not an obstacle
        if 0 <= r < rows and 0 <= c < cols and not nodes[r][c].is_obs and (r, c) not in visited:
            neighbor = nodes[r][c]
            new_g = current_node.g + 1
            if algorithm == "astar":
                # A*, cost = g + h (applys heuristic)
                new_cost = new_g + neighbor.h
            else:
                #Dijkstra just update the cost normally 
                new_cost = new_g
            # Update the node if this path is better than any previous one
            if new_g < neighbor.g if neighbor.g is not None else float('inf'):
                neighbor.g = new_g
                neighbor.cost = new_cost
                neighbor.parent = current_node
                list_of_nodes.put((neighbor.cost, counter, neighbor))
                counter += 1
    return counter





def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###

    if not is_valid_start_goal(grid, start, goal):
        print("Invalid start or goal in dijkstra")
        return [], 0

    path = []
    steps = 0
    grid = np.rot90(grid)
    rows, cols = len(grid), len(grid[0])
    nodes = [[Node(r, c, grid[r][c] == 1, 0) for c in range(cols)] for r in range(rows)]
    start_node = nodes[len(grid) - 1 - start[1]][start[0]]
    goal_node = nodes[len(grid) - 1 - goal[1]][goal[0]]
    start_node.g = 0
    start_node.cost = 0

    list_of_nodes = queue.PriorityQueue()
    list_of_nodes.put((start_node.cost, 0, start_node))  # (cost, counter, node)
    visited = set() # to store the visited nodes
    counter = 1 

    while not list_of_nodes.empty():
        # Get the node with the lowest cost
        current_cost, _, current_node = list_of_nodes.get()
        visited.add((current_node.row, current_node.col))
        steps += 1

        # Path found!  Return the path and the number of steps
        if current_node == goal_node:
            path = []
            while current_node:
                # back track the path
                path.append([current_node.row, current_node.col])
                current_node = current_node.parent
            print(f"It takes {steps} steps to find a path using Dijkstra")
            return path[::-1], steps

        # Explore the nearby nodes
        counter = exlpore_nodes(current_node, nodes, list_of_nodes, visited, counter, "dijkstra")


    print("No path found using Dijkstra")
    return [], 0



def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]

    
    '''
    
    ### YOUR CODE HERE ###
    if not is_valid_start_goal(grid, start, goal):
        print("Invalid start or goal in astar")
        return [], 0

    path = []
    steps = 0
    grid = np.rot90(grid)
    rows, cols = len(grid), len(grid[0])
    nodes = [[Node(r, c, grid[r][c] == 1, heuristic((r, c), goal)) for c in range(cols)] for r in range(rows)]
    start_node = nodes[len(grid) - 1 - start[1]][start[0]]
    goal_node = nodes[len(grid) - 1 - goal[1]][goal[0]]
    # start_node = nodes[start[0]][start[1]]
    # goal_node = nodes[goal[0]][goal[1]]
    start_node.g = 0
    start_node.cost = start_node.h  # A*, cost = g + h

    list_of_nodes = queue.PriorityQueue()
    list_of_nodes.put((start_node.cost, 0, start_node))  # (cost, counter, node)
    visited = set()
    counter = 1


    while not list_of_nodes.empty():
        # Get the node with the lowest cost
        current_cost, _, current_node = list_of_nodes.get()
        visited.add((current_node.row, current_node.col))
        steps += 1


        # Path found! Return the path and the number of steps
        if current_node == goal_node:
            path = []
            # back track the path
            while current_node:
                path.append([current_node.row, current_node.col])
                current_node = current_node.parent
            print(f"It takes {steps} steps to find a path using A*")
            return path[::-1], steps
        
        # Explore the nearby nodes
        counter = exlpore_nodes(current_node, nodes, list_of_nodes, visited, counter, "astar")


    print("No path found using A*")
    return [], 0


# Load map, start and goal point.
def load_map(file_path):
    grid = []
    start = [0, 0]
    goal = [0, 0]
    # Load from the file
    with open(file_path, 'r') as map_file:
        reader = csv.reader(map_file)
        for i, row in enumerate(reader):
            # load start and goal point
            if i == 0:
                start[0] = int(row[1])
                start[1] = int(row[2])
            elif i == 1:
                goal[0] = int(row[1])
                goal[1] = int(row[2])
            # load the map
            else:
                int_row = [int(col) for col in row]
                grid.append(int_row)
    return grid, start, goal



# Draw final results
def draw_path(grid, path, title="Path"):
    # Visualization of the found path using matplotlib
    fig, ax = plt.subplots(1)
    ax.margins()
    # Draw map
    row = len(grid)     # map size
    col = len(grid[0])  # map size
    for i in range(row):
        for j in range(col):
            if grid[i][j]: 
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='k'))  # obstacle
            else:          
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='w'))  # free space
    # Draw path
    for x, y in path:
        ax.add_patch(Rectangle((y-0.5, x-0.5),1,1,edgecolor='k',facecolor='b'))          # path
    ax.add_patch(Rectangle((start[1]-0.5, start[0]-0.5),1,1,edgecolor='k',facecolor='g'))# start
    ax.add_patch(Rectangle((goal[1]-0.5, goal[0]-0.5),1,1,edgecolor='k',facecolor='r'))  # goal
    # Graph settings
    plt.title(title)
    plt.axis('scaled')
    plt.gca().invert_yaxis()



# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()