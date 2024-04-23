def calculate_center(width, height, upper_left_x, upper_left_y):
    center_x = upper_left_x + (width / 2)
    center_y = upper_left_y + (height / 2)
    print(f'Center coords: x={center_x}, y={center_y}')

calculate_center(48, 2, 1, 0)