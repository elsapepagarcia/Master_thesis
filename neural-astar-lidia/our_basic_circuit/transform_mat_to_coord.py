import numpy as np

# Define the grid with the planned path
planned_path = np.array([
    [0, 0, 0], 
    [1, 0, 0], 
    [1, 1, 1], 
    [0, 0, 1], 
    [0, 0, 0]
])

# Define start and end indices
start_idx = [1, 0]  # (row, col) for the start point
end_idx = [3, 2]    # (row, col) for the end point

# Define the grid cell size in meters (59 cm)
cell_size = 0.59

# Find the coordinates of each point in the planned path
def get_coordinates(matrix, start_idx, cell_size):
    coords = []
    rows, cols = matrix.shape

    # Iterate through the matrix to get the indices of the planned path (1s)
    for i in range(rows):
        for j in range(cols):
            if matrix[i, j] == 1:  # Check if it's part of the path
                # Translate matrix indices (i, j) to real-world coordinates
                coord_x = (i - start_idx[0]) * cell_size  # Adjust row by start_idx
                coord_y = (j - start_idx[1]) * cell_size  # Adjust column by start_idx
                coords.append((coord_x, coord_y))
    
    return coords

# Get the coordinates of the planned path
path_coordinates = get_coordinates(planned_path, start_idx, cell_size)

# Print the path coordinates
print("Path coordinates (in meters):")
for coord in path_coordinates:
    print(coord)
