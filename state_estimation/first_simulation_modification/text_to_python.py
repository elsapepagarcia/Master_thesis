import numpy as np

# Step 1: Read the file
filename = 'modified_data.txt'  # Replace with the path to your .txt file

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


plt.scatter(x, y)  # Scatter plot of the points
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('2D Points Plot')
plt.grid(True)  # Optional: Add a grid for better visibility
plt.show()


