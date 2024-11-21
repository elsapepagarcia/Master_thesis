import numpy as np
from scipy.optimize import minimize

# Example path (x, y) positions
positions = np.array([
    [0, 0],
    [1, 1],
    [2, 2],
    [3, 3],
    [4, 4],
    [5, 3],  # Drift occurs here
    [6, 2],
    [7, 1],
    [8, 0]
])

# Simulate a loop closure detected between position 8 and position 0
loop_closure_indices = (8, 0)

# Error function: Sum of squared differences between consecutive positions
def error_function(positions_flattened):
    positions_reshaped = positions_flattened.reshape(-1, 2)
    error = 0
    
    # Add odometry errors (difference between consecutive positions)
    for i in range(len(positions_reshaped) - 1):
        error += np.sum((positions_reshaped[i+1] - positions_reshaped[i]) ** 2)
    
    # Add loop closure error
    if loop_closure_indices:
        i, j = loop_closure_indices
        error += np.sum((positions_reshaped[i] - positions_reshaped[j]) ** 2)
    
    return error

# Optimize the positions
initial_positions = positions.flatten()  # Flatten to a 1D array
result = minimize(error_function, initial_positions, method='L-BFGS-B')

# Get optimized positions
optimized_positions = result.x.reshape(-1, 2)

# Print optimized positions
print("Optimized Positions:\n", optimized_positions)

# You can plot the original and optimized paths to visualize the correction
import matplotlib.pyplot as plt

plt.plot(positions[:, 0], positions[:, 1], 'ro-', label='Original Path')
plt.plot(optimized_positions[:, 0], optimized_positions[:, 1], 'bo-', label='Optimized Path')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Path Optimization with Loop Closure')
plt.legend()
plt.grid(True)
plt.show()
