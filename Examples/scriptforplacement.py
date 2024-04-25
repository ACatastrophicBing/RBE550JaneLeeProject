def calculateBoxCenter(width, height, upper_left_x, upper_left_y):
    center_x = upper_left_x + (width / 2)
    center_y = upper_left_y + (height / 2)
    print(f'self.obstacles.append(Wall(self.env, x={center_x}, y={center_y}, width={width}, height={height}, angle=0, color=obstacle_color))')

# boxData = [[20, 2, 3, 3], [20, 2, 3, 9], [20, 2, 3, 15], [20, 2, 3, 21], [20, 2, 27, 5], [20, 2, 27, 11], [20, 2, 27, 17], [20, 2, 27, 23], [2, 11, 29, 29], [2, 6, 34, 30], [26, 1, 24, 46], [5, 1, 13, 46], [1, 4, 12, 46], [1, 4, 3, 46], [3, 1, 4, 49], [1, 4, 7, 46]]

# for object in boxData:
#     width = object[0]
#     height = object[1]
#     x_point = object[2]
#     y_point = object[3]
#     calculateBoxCenter(width, height, x_point, y_point)

diskData = [[4, 3, 28], [4, 8, 28], [4, 5, 30], [4, 11, 30], [4, 8, 33], [3, 39, 29], [3, 41, 28], [3, 42, 30], [4, 41, 39], [4, 45, 36], [4, 45, 40]]
def calculateDiskCenter(width, upper_left_x, upper_left_y):
    radius = 0.5*width
    center_x = upper_left_x - radius
    center_y = upper_left_y - radius
    print(f'self.obstacles.append(Disk(self.env,x={center_x},y={center_y},angle=0,radius={radius},density=1000,color=obstacle_color))')

for object in diskData:
    radius = object[0]
    x_point = object[1]
    y_point = object[2]
    calculateDiskCenter(radius, x_point, y_point)