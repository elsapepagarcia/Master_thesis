import torch
import sys
import numpy as np
import matplotlib.pyplot as plt
from src.neural_astar.planner import NeuralAstar, VanillaAstar
from src.neural_astar.utils.training import load_from_ptl_checkpoint
from src.neural_astar.utils.lidia_data import create_dataloader
from src.neural_astar.utils.lidia_data import visualize_results
import time

device = "cuda" if torch.cuda.is_available() else "cpu"

print("REMINDER: in the model(mazes_032_moore_c8/lightning_logs/ folder you need to copy the folders with the weights of the model you want to test. These weights are saved in folder 'my_training'. You also need to remember to specify your model in line 62 of files 'encoder.py' of folders 'src/neural_astar/planner' and 'src/neural_astar/utils'.")


neural_astar = NeuralAstar(encoder_arch='CNN', encoder_depth=4).to(device)
neural_astar.load_state_dict(load_from_ptl_checkpoint("./model/mazes_032_moore_c8/lightning_logs/"))
#print("HERE")

flag_abort = 0

vanilla_astar = VanillaAstar().to(device)


# Here you define the number of rows and columns of our circuit (including margin)
#num_rows = 7
#num_cols = 5

#start = 4
#end = 11

# For adding the margins
#start = 11
#end = 23

#start_idx = (start // num_cols, start % num_cols)
#end_idx = (end // num_cols, end % num_cols)


folder_name = input("Enter the name of the environment you want to load (it should be saved in generated_maps folder): ")


# Get the coordinates from the user
col_start, row_start = map(int, input("Enter the row and column indices (separated by a space) of the starting point: ").split())

# Get the coordinates from the user
col_end, row_end = map(int, input("Enter the row and column indices (separated by a space) of the ending point: ").split())
    



dataloader = create_dataloader("./generated_maps/" + folder_name + ".npz", [row_start, col_start], [row_end, col_end])
map_designs, start_maps, goal_maps = next(iter(dataloader))

num_cols, num_rows = map_designs.shape


start = col_start * num_rows + row_start
end = col_end * num_rows + row_end


start_idx = (start // num_cols, start % num_cols)
end_idx = (end // num_cols, end % num_cols)


# Validate the input
if row_start < 0 or row_start >= num_rows or col_start < 0 or col_start >= num_cols:
	print("Invalid coordinates. Please enter values within the matrix dimensions.")
	sys.exit()

    
# Validate the input
if row_end < 0 or row_end >= num_rows or col_end < 0 or col_end >= num_cols:
	print("Invalid coordinates. Please enter values within the matrix dimensions.")
	sys.exit()












#dataloader = create_dataloader("./adj_matrix.npz", 0, 8)
#dataloader = create_dataloader("./generated_maps/adj_matrix_1.npz", 35, 37)
#dataloader = create_dataloader("./generated_maps/adj_matrix_2.npz", 18, 270)
#dataloader = create_dataloader("./generated_maps/adj_matrix_3.npz", 29, 648)
#dataloader = create_dataloader("./generated_maps/adj_matrix_4b.npz", 17, 133)
#dataloader = create_dataloader("./generated_maps/adj_matrix_5.npz", 19, 162)
#dataloader = create_dataloader("./generated_maps/adj_matrix_6.npz", 33, 989)
#dataloader = create_dataloader("./generated_maps/adj_matrix_7.npz", 11, 51)
#dataloader = create_dataloader("./generated_maps/adj_matrix_8.npz", 19, 238)
#dataloader = create_dataloader("./generated_maps/adj_matrix_9.npz", 34, 222)
#dataloader = create_dataloader("./generated_maps/adj_matrix_10.npz", 24, 304)
#dataloader = create_dataloader("./super_dense_circuit.npz", 24, 36)
#dataloader = create_dataloader("./our_environment.npz", start, end)
#dataloader = create_dataloader("./our_env_with_margin.npz", start, end)
#dataloader = create_dataloader("./circuit_data.npz", start, end)
#dataloader = create_dataloader("./our_basic_circuit.npz", start, end)
#map_designs, start_maps, goal_maps = next(iter(dataloader))
#print("Hola")
#print(type(map_designs))

#dataloader = create_dataloader_paper("./planning-datasets/data/mpd/mazes_032_moore_c8.npz", "test", 8)
#map_designs, start_maps, goal_maps, _ = next(iter(dataloader))



#print("Notebook this is the map")
#print(map_designs)
#print(start_maps)
#print(goal_maps)





fig, axes = plt.subplots(1, 3, figsize=[8, 5])

check = np.asarray(map_designs)*np.asarray(start_maps)
if np.all(check ==0):
	print("The starting point is wrong, it is an obstacle")
	flag_abort = 1 
	
check = np.asarray(map_designs)*np.asarray(goal_maps)
if np.all(check ==0):
	print("The end point is wrong, it is an obstacle")
	flag_abort = 1 

if flag_abort == 0:
	axes[0].imshow(np.asarray(map_designs))
	axes[0].set_title("map_design")
	axes[0].axis("off")
	axes[1].imshow(np.asarray(start_maps))
	axes[1].set_title("start_map")
	axes[1].axis("off")
	axes[2].imshow(np.asarray(goal_maps))
	axes[2].set_title("goal_map")
	axes[2].axis("off")

	plt.show()


	'''fig, axes = plt.subplots(2, 3, figsize=[8, 5])
	for i in range(2):
	    axes[i, 0].imshow(map_designs.numpy()[i, 0])
	    axes[i, 0].set_title("map_design")
	    axes[i, 0].axis("off")
	    axes[i, 1].imshow(start_maps.numpy()[i, 0])
	    axes[i, 1].set_title("start_map")
	    axes[i, 1].axis("off")
	    axes[i, 2].imshow(goal_maps.numpy()[i, 0])
	    axes[i, 2].set_title("goal_map")
	    axes[i, 2].axis("off")
	'''
	neural_astar.eval()
	vanilla_astar.eval()

	map_designs_tensor = torch.tensor(map_designs, dtype=torch.float32).to(device)
	map_designs_tensor = map_designs_tensor.unsqueeze(0).unsqueeze(0) # for shape [1, 1, 3, 3]
	start_maps_tensor = torch.tensor(start_maps, dtype=torch.float32).to(device)
	start_maps_tensor = start_maps_tensor.unsqueeze(0).unsqueeze(0) # for shape [1, 1, 3, 3]
	goal_maps_tensor = torch.tensor(goal_maps, dtype=torch.float32).to(device)
	goal_maps_tensor = goal_maps_tensor.unsqueeze(0).unsqueeze(0) # for shape [1, 1, 3, 3]

	start_time = time.time()
	va_outputs = vanilla_astar(map_designs_tensor.to(device), start_maps_tensor.to(device), goal_maps_tensor.to(device))
	end_time = time.time()
	print("This is the time it takes to predict a path with Vanilla A*:")
	print(end_time-start_time)
	start_time = time.time()
	na_outputs = neural_astar(map_designs_tensor.to(device), start_maps_tensor.to(device), goal_maps_tensor.to(device))
	end_time = time.time()
	print("This is the time it takes to predict a path with Neural A*:")
	print(end_time - start_time)


	#print("This is what was predicted:")
	#print(na_outputs)
	history = (na_outputs.histories).detach().numpy()
	total_ones = np.sum(history == 1)
	print("This is the total number of ones for neural A*:")
	print(total_ones)
	history = (va_outputs.histories).detach().numpy()
	total_ones = np.sum(history == 1)
	print("This is the total number of ones for simple A*:")
	print(total_ones)

	fig, axes = plt.subplots(1, 2, figsize=[12, 4])
	axes[0].imshow(visualize_results(map_designs_tensor, na_outputs, scale=1, start=start_idx, end=end_idx))
	axes[0].set_title("Neural A*")
	axes[0].axis("off")
	axes[1].imshow(visualize_results(map_designs_tensor, va_outputs, start=start_idx, end=end_idx))
	axes[1].set_title("Vanilla A*")
	axes[1].axis("off")
	plt.show()
