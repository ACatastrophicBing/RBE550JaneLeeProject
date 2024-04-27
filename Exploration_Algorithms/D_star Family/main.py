<<<<<<< HEAD
import csv
from search import DStar


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


if __name__ == "__main__":
    # Load the map
    grid, start, goal = load_map('D_star/map1.csv')
    dynamic_grid, _, _ = load_map('D_star/dynamic_map1.csv')
    # grid, start, goal = load_map('map2.csv')
    # dynamic_grid, _, _ = load_map('dynamic_map2.csv')
    # grid, start, goal = load_map('map3.csv')
    # dynamic_grid, _, _ = load_map('dynamic_map3.csv')

    # Search
    d_star = DStar(grid, dynamic_grid, start, goal)

    # Visualize the map
    d_star.draw_path(grid, "static map")
    d_star.draw_path(dynamic_grid, "dynamic map")

    # Run D*
    d_star.run()
=======
import csv
from search import DStar


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


if __name__ == "__main__":
    # Load the map
    grid, start, goal = load_map('D_star/map1.csv')
    dynamic_grid, _, _ = load_map('D_star/dynamic_map1.csv')
    # grid, start, goal = load_map('map2.csv')
    # dynamic_grid, _, _ = load_map('dynamic_map2.csv')
    # grid, start, goal = load_map('map3.csv')
    # dynamic_grid, _, _ = load_map('dynamic_map3.csv')

    # Search
    d_star = DStar(grid, dynamic_grid, start, goal)

    # Visualize the map
    d_star.draw_path(grid, "static map")
    d_star.draw_path(dynamic_grid, "dynamic map")

    # Run D*
    d_star.run()
>>>>>>> main
