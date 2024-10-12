import numpy as np

# Define the grid
grid = np.array([
    [1, 1, 1],
    [1, 0, 1],
    [1, 1, 1],
    [1, 0, 1],
    [1, 1, 1]
])

# Get the dimensions of the grid
rows, cols = grid.shape

# Create an adjacency matrix of size rows*cols x rows*cols (since each cell is a node)
adj_matrix = np.zeros((rows * cols, rows * cols), dtype=int)

# Function to get the index of a node in the flattened version of the grid
def node_index(r, c):
    return r * cols + c

# Iterate over each cell in the grid
for r in range(rows):
    for c in range(cols):
        if grid[r, c] == 1:  # Only process traversable nodes
            # Current node index
            current_node = node_index(r, c)
            
            # Check neighboring nodes (up, down, left, right) and connect if valid
            # Up
            if r > 0 and grid[r-1, c] == 1:
                adj_matrix[current_node, node_index(r-1, c)] = 1
            # Down
            if r < rows - 1 and grid[r+1, c] == 1:
                adj_matrix[current_node, node_index(r+1, c)] = 1
            # Left
            if c > 0 and grid[r, c-1] == 1:
                adj_matrix[current_node, node_index(r, c-1)] = 1
            # Right
            if c < cols - 1 and grid[r, c+1] == 1:
                adj_matrix[current_node, node_index(r, c+1)] = 1

# Save the adjacency matrix to a .npy file
np.save('adjacency_matrix.npy', adj_matrix)

print("Adjacency matrix saved as 'adjacency_matrix.npy'.")
print("Adjacency Matrix:\n", adj_matrix)
