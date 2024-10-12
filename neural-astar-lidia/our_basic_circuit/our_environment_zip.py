import numpy as np

# Define the grid again (5x3)
grid = np.array([
    [1, 1, 1],
    [1, 0, 1],
    [1, 1, 1],
    [1, 0, 1],
    [1, 1, 1]
])

rows, cols = grid.shape  # Get the dimensions of the grid

# Create the adjacency matrix as explained earlier
adj_matrix = np.zeros((rows * cols, rows * cols), dtype=int)

def node_index(r, c):
    return r * cols + c

for r in range(rows):
    for c in range(cols):
        if grid[r, c] == 1:
            current_node = node_index(r, c)
            if r > 0 and grid[r-1, c] == 1:
                adj_matrix[current_node, node_index(r-1, c)] = 1
            if r < rows - 1 and grid[r+1, c] == 1:
                adj_matrix[current_node, node_index(r+1, c)] = 1
            if c > 0 and grid[r, c-1] == 1:
                adj_matrix[current_node, node_index(r, c-1)] = 1
            if c < cols - 1 and grid[r, c+1] == 1:
                adj_matrix[current_node, node_index(r, c+1)] = 1

# Create the dimensions array
dimensions = np.array([rows, cols])

# Save the two arrays in a .npz file
np.savez('circuit_data.npz', out=adj_matrix, dim=dimensions)

print("Circuit data saved in 'circuit_data.npz'.")
