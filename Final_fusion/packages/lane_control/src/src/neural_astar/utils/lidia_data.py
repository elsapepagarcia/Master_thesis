from __future__ import annotations, print_function

import numpy as np
import torch
import torch.utils.data as data
#from neural_astar.planner.differentiable_astar import AstarOutput
from .differentiable_astar import AstarOutput
from PIL import Image
from torchvision.utils import make_grid


def visualize_results(
    map_designs: torch.tensor, planner_outputs: AstarOutput, start, end, scale: int = 1
) -> np.ndarray:
    """
    Create a visualization of search results

    Args:
        map_designs (torch.tensor): input maps
        planner_outputs (AstarOutput): outputs from planner
        scale (int): scale factor to enlarge output images. Default to 1.

    Returns:
        np.ndarray: visualized results
    """

    if type(planner_outputs) == dict:
        histories = planner_outputs["histories"]
        paths = planner_outputs["paths"]
    else:
        histories = planner_outputs.histories
        paths = planner_outputs.paths
    
    '''
    print("Here the path:")
    print(paths)
    an_array = paths.cpu().numpy()
    an_array = np.squeeze(an_array)
    print(an_array)
    
        
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
    path_coordinates = get_coordinates(an_array, start, cell_size)

    # Print the path coordinates
    print("Path coordinates (in meters):")
    for coord in path_coordinates:
        print(coord)
        
    
    print(np.where(an_array == 1))'''
    
    # Make sure all tensors are on the same device
    device = map_designs.device
    results = make_grid(map_designs).permute(1, 2, 0).to(device)
    h = make_grid(histories).permute(1, 2, 0).to(device)
    p = make_grid(paths).permute(1, 2, 0).float().to(device)
    results[h[..., 0] == 1] = torch.tensor([0.2, 0.8, 0]).to(device)
    results[p[..., 0] == 1] = torch.tensor([1.0, 0.0, 0]).to(device)

    results = (np.asarray(results.cpu()) * 255.0).astype("uint8")

    if scale > 1:
        results = Image.fromarray(results).resize(
            [x * scale for x in results.shape[:2]], resample=Image.NEAREST
        )
        results = np.asarray(results)

    return results


def create_dataloader(
    filename: str,
    start_idx: int, 
    goal_idx: int
) -> data.DataLoader:
    """
    Create dataloader from npz file

    Args:
        filename (str): npz file that contains train, val, and test data
    Returns:
        torch.utils.data.DataLoader: dataloader
    """

    dataset = MazeDataset(filename, start_idx, goal_idx)
    #print("Create data loader")
    #print(type(dataset[0]))
    return dataset

def node_to_grid(node, grid_size):
	x = node // grid_size[1]
	y = node % grid_size[1]
	return x, y 


class MazeDataset(data.Dataset):
    def __init__(
        self,
        filename: str,
	start_idx: int, 
	goal_idx: int
    ):
        """
        Custom dataset for shortest path problems
        See planning-datasets repository for how to create original file.

        Args:
            filename (str): npz file that contains train, val, and test data

        Note:
            __getitem__ will return the following matrices:
            - map_design [1, 1, W, W]: obstacles map that takes 1 for passable locations
            - start_map [1, 1, W, W]: one-hot matrices indicating (num_starts) starting points
            - goal_map [1, 1, W, W]: one-hot matrix indicating the goal point
        """
        assert filename.endswith("npz")  # Must be .npz format
        self.filename = filename
        self.start_idx = start_idx
        self.goal_idx = goal_idx

        (
            self.map,
            self.start_map,
            self.goal_map
        ) = self._process(filename, start_idx, goal_idx)

    def _process(self, filename: str, start_idx: int, goal_idx: int):
        with np.load(filename) as f:
            with np.load(filename) as f:
                adj_map = f["out"]
                dims = f["dims"]
            	# adjust adj_map
                map = np.zeros(dims)
                
                for i in range(adj_map.shape[0]):
                    for j in range(adj_map.shape[1]):
                        if adj_map[i, j] == 1:
                            row_i, col_i = node_to_grid(i, dims)
                            row_j, col_j = node_to_grid(j, dims)
                            map[row_i, col_i] = 1
                            map[row_j, col_j] = 1
                print("This is the map")
                print(map)
                print("This is the adjacent matrix")
                print(adj_map)
            	# Start point
                start_map = np.zeros(dims)
                start_map.ravel()[start_idx] = 1.0
            	
            	# End point
                goal_map = np.zeros(dims)
            	#print(goal_map)
            	#print(goal_map.ravel())
            	#print(dims)
                goal_map.ravel()[goal_idx] = 1.0
            	
                map = map.astype(np.float32)
                print("This is the map again")
                print(map)
                start_map = start_map.astype(np.float32)
                goal_map = goal_map.astype(np.float32)
        
        return map, start_map, goal_map 
        #return map_designs, goal_maps, opt_policies, opt_dists

    def __getitem__(self, index: int):
        print(index)
        map = self.map
        print("And.. This is the map again")
        print(map)
        goal_map = self.goal_map
        start_map = self.start_map
        print("Type getitem:")
        print(type(map))
        print(map.shape)
        print(start_map.shape)
        print(goal_map.shape)
        return map, start_map, goal_map


