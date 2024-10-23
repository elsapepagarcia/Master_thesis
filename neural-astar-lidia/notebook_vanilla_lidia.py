import torch
import numpy as np
import matplotlib.pyplot as plt
from src.neural_astar.planner import NeuralAstar, VanillaAstar
from src.neural_astar.utils.training import load_from_ptl_checkpoint
from src.neural_astar.utils.lidia_data import create_dataloader
from src.neural_astar.utils.lidia_data import visualize_results

device = "cuda" if torch.cuda.is_available() else "cpu"

neural_astar = NeuralAstar(encoder_arch='CNN', encoder_depth=4).to(device)
neural_astar.load_state_dict(load_from_ptl_checkpoint("./model/mazes_032_moore_c8/lightning_logs/"))
print("HERE")

vanilla_astar = VanillaAstar().to(device)

#start = 2+22
#end = 20 + (22*15)

#FOR SUPER DENSE CIRCUIT
#start = 24
#end = 36

num_rows = 5
num_cols = 3

start = 3
end = 11

start_idx = (start // num_cols, start % num_cols)
end_idx = (end // num_cols, end % num_cols)


#start = 1 + 5
#end = 3 + (5*3)

#dataloader = create_dataloader("./adj_matrix.npz", 0, 8)
#dataloader = create_dataloader("./super_dense_circuit.npz", start, end)
dataloader = create_dataloader("./our_environment.npz", start, end)
#dataloader = create_dataloader("./our_basic_circuit.npz", start, end)
map_designs, start_maps, goal_maps = next(iter(dataloader))
#print("Hola")
#print(type(map_designs))
print("Notebook this is the map")
print(map_designs)
print(start_maps)
print(goal_maps)


fig, axes = plt.subplots(1, 3, figsize=[8, 5])

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
neural_astar.eval()
vanilla_astar.eval()

map_designs_tensor = torch.tensor(map_designs, dtype=torch.float32).to(device)
map_designs_tensor = map_designs_tensor.unsqueeze(0).unsqueeze(0) # for shape [1, 1, 3, 3]
start_maps_tensor = torch.tensor(start_maps, dtype=torch.float32).to(device)
start_maps_tensor = start_maps_tensor.unsqueeze(0).unsqueeze(0) # for shape [1, 1, 3, 3]
goal_maps_tensor = torch.tensor(goal_maps, dtype=torch.float32).to(device)
goal_maps_tensor = goal_maps_tensor.unsqueeze(0).unsqueeze(0) # for shape [1, 1, 3, 3]

va_outputs = vanilla_astar(map_designs_tensor.to(device), start_maps_tensor.to(device), goal_maps_tensor.to(device))
na_outputs = neural_astar(map_designs_tensor.to(device), start_maps_tensor.to(device), goal_maps_tensor.to(device))


print("This is what was predicted:")
print(na_outputs)

fig, axes = plt.subplots(2, 1, figsize=[12, 4])
axes[0].imshow(visualize_results(map_designs_tensor, na_outputs, scale=1, start=start_idx, end=end_idx))
axes[0].set_title("Neural A*")
axes[0].axis("off")
axes[1].imshow(visualize_results(map_designs_tensor, va_outputs, start=start_idx, end=end_idx))
axes[1].set_title("Vanilla A*")
axes[1].axis("off")
plt.show()
