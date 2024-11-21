import matplotlib.pyplot as plt

# Data for the three curves
x_values = [35, 81, 225, 225, 256, 256, 289, 324, 676, 1024]

y_curve1 = [0, 26.32, 16.67, 24.14, 2.35, 26.25, 51.61, 16.25, 50.75, 66.78]
y_curve2 = [0, 31.58, 33.33, 20.69, 25.88, 31.25, 60.22, 18.75, 66.67, 69.86]
y_curve3 = [0, 36.84, 1.49, 31.03, 21.18, 22.5, 29.03, 25, 55.22, 50]

# Plotting the data
plt.figure(figsize=(10, 6))
plt.plot(x_values, y_curve1, marker='o', label='Standard CNN')
plt.plot(x_values, y_curve2, marker='s', label='Half amount of channels')
plt.plot(x_values, y_curve3, marker='^', label='Half amount of layers')

# Adding labels and title
plt.xlabel("Total number of Nodes in Grid", fontsize=12)
plt.ylabel("Reduction ratio of node explorations (Exp)", fontsize=12)
plt.title("Explorations vs Number of Tiles in Grid", fontsize=14)
plt.legend()
plt.grid(True)

# Display the plot
plt.show()
