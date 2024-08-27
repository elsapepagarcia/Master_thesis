import numpy as np
from scipy.optimize import minimize

input_file = 'modified_data.txt'
# Output file path
output_file = 'output_file.txt'
# Output_file 2 paht
output_file_2 = 'output_file_2.txt'


# Open the input file and output file
with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
    for line in infile:
        # Remove trailing commas and any surrounding whitespace
        line = line.rstrip(', \n')
        
        # Split the line into components based on whitespace
        columns = line.split()
        
        # If the line contains at least 3 columns, remove the third one
        if len(columns) >= 3:
            # Keep only the first two columns
            cleaned_line = f"{columns[0]} {columns[1]}\n"
            # Write the cleaned line to the output file
            outfile.write(cleaned_line)

# Open the input file and output file
with open(output_file, 'r') as infile, open(output_file_2, 'w') as outfile:
    for line in infile:
        # Remove the trailing comma and any surrounding whitespace
        line = line.rstrip(', \n')
        
        # Write the cleaned line to the output file
        outfile.write(line + '\n')



# Step 1: Read the file
filename = 'output_file_2.txt'  # Replace with the path to your .txt file

with open(filename, 'r') as file:
    lines = file.readlines()

# Step 2: Parse the contents
data = []
for line in lines:
    # Strip any leading/trailing whitespace and split by commas
    coordinates = line.strip().split(',')
    # Convert the strings to floats and add to the data list
    data.append([float(coordinate) for coordinate in coordinates])

# Step 3: Convert the list to a NumPy array
array_data = np.array(data)

print(array_data)  # Optional: Print the array to verify

import matplotlib.pyplot as plt

# Assuming you want to plot the points in 3D space
#fig = plt.figure()
#ax = fig.add_subplot(11, projection='2d')

# Unpacking x, y, z coordinates from the array
x = array_data[:, 0]
y = array_data[:, 1]
#z = array_data[:, 2]

rows, columns = array_data.shape

loop_closure_indices = (rows-1, 0)



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
initial_positions = array_data.flatten()  # Flatten to a 1D array
result = minimize(error_function, initial_positions, method='L-BFGS-B')


# Get optimized positions
optimized_positions = result.x.reshape(-1, 2)

# Print optimized positions
print("Optimized Positions:\n", optimized_positions)


# You can plot the original and optimized paths to visualize the correction
import matplotlib.pyplot as plt

plt.plot(array_data[:, 0], array_data[:, 1], 'ro-', label='Original Path')
plt.plot(optimized_positions[:, 0], optimized_positions[:, 1], 'bo-', label='Optimized Path')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Path Optimization with Loop Closure')
plt.legend()
plt.grid(True)
plt.show()



#plt.plot(x, y)  # Scatter plot of the points
#plt.xlabel('X Coordinate')
#plt.ylabel('Y Coordinate')
#plt.title('2D Points Plot')
#plt.grid(True)  # Optional: Add a grid for better visibility
#plt.show()


