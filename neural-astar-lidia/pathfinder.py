import sys
import torch
import numpy as np
import argparse
from src.neural_astar.planner import NeuralAstar, VanillaAstar
from src.neural_astar.utils.training import load_from_ptl_checkpoint
from src.neural_astar.utils.lidia_data import create_dataloader
from src.neural_astar.utils.lidia_data import visualize_results


MAP  = []
ONES = []
WEIGHTED_GRAPH = {}
HEIGHT = 0
WIDTH  = 0


def main(file, st, end):
    global HEIGHT
    global WIDTH
    data = np.load(file)
    # adj is a lower triangular matrix
    adj  = data['out']
    dims = data['dims']
    HEIGHT = dims[0]
    WIDTH = dims[1]
    to_graph(adj)

    sys.stdout.write("\n")
    ascii_map()
    sys.stdout.write("\n")
    set_weighted(st, end)

    # ============================================================
    # Change THIS FUNCTION to use different path finding algorithm.
    # Must return an array of node coordinates within MAP,
    # where st comes first and end comes last.
    # elements are COORDINATES
    
    
    device = "cuda" if torch.cuda.is_available() else "cpu"

    neural_astar = NeuralAstar(encoder_arch='CNN').to(device)
    neural_astar.load_state_dict(load_from_ptl_checkpoint("./model/mazes_032_moore_c8/lightning_logs/"))
    
    st = c2u(st)
    end = c2u(end)
    #start=24
    #end = 36
    
    dataloader = create_dataloader(file, st, end)
    #dataloader = create_dataloader("./super_dense_circuit.npz", start, end)
    map_designs, start_maps, goal_maps = next(iter(dataloader))
    neural_astar.eval()
    
    map_designs_tensor = torch.tensor(map_designs, dtype=torch.float32).to(device)
    map_designs_tensor = map_designs_tensor.unsqueeze(0).unsqueeze(0) # for shape [1, 1, 3, 3]
    start_maps_tensor = torch.tensor(start_maps, dtype=torch.float32).to(device)
    start_maps_tensor = start_maps_tensor.unsqueeze(0).unsqueeze(0) # for shape [1, 1, 3, 3]
    goal_maps_tensor = torch.tensor(goal_maps, dtype=torch.float32).to(device)
    goal_maps_tensor = goal_maps_tensor.unsqueeze(0).unsqueeze(0) # for shape [1, 1, 3, 3]

    na_outputs = neural_astar(map_designs_tensor.to(device), start_maps_tensor.to(device), goal_maps_tensor.to(device))
    
    
    if type(na_outputs) == dict:
        histories = na_outputs["histories"]
        paths = na_outputs["paths"]
    else:
        histories = na_outputs.histories
        paths = na_outputs.paths
    path_y, path_x = np.where(np.squeeze(paths.cpu().numpy()) == 1)
    path = list(zip(path_x, path_y))
    #path = shortest_path()

    # example path
    #path = [(0, 0), (1, 0), (2, 0), (3, 0), (3, 1), (3, 2)]
    output_path(path, u2c(st), u2c(end))



# "unique" to coordinate: converts the unique node number to a coordinate tuple
def u2c(u):
    if (len(MAP) == 0):
        raise ValueError("MAP not yet initialized")
    x = int(u % WIDTH)
    y = int(u / WIDTH)
    return (x, y)

# coordinate to "unique": converts coordinate tuple (x, y) to its unique equivalent
def c2u(c):
    #print("Here c2u:")
    #print(c[1])
    #print(c[0])
    #print(WIDTH)
    return (c[0] * WIDTH) + c[1]

# initialises the global variable MAP with a node graph
# based on the adjacency matrix
def to_graph(adj):
    global MAP
    MAP = [[None for x in range(WIDTH)] for y in range(HEIGHT)]

    # initialize the nodes
    for y in range(0, HEIGHT):
        for x in range(0, WIDTH):
            MAP[y][x] = Node(x, y)

    for j in range(0, HEIGHT):
        for i in range(0, WIDTH):
            MAP[j][i].set_neighbours()

    for b in range(0, HEIGHT * WIDTH):
        for a in range(0, b):
            if (adj[b][a]):
                if (a == b - WIDTH):
                    neigh_pos = "N"
                elif (a == b + WIDTH):
                    neigh_pos = "S"
                elif (a == b + 1):
                    neigh_pos = "E"
                elif (a == b - 1):
                    neigh_pos = "W"
                else:
                    raise ValueError("neighbour not in right place")

                coords = u2c(b)
                if not (MAP[coords[1]][coords[0]].connected[neigh_pos]):
                    MAP[coords[1]][coords[0]].connect(neigh_pos)


def set_weighted(st, end):
    global WEIGHTED_GRAPH
    # start and end must both be connected to the road network
    if (
            (st[0] < 0) or (st[0] >= WIDTH) or (st[1] < 0) or (st[1] >= HEIGHT)
            or (end[0] < 0) or (end[0] >= WIDTH) or (end[1] < 0) or (end[1] >= HEIGHT)
            or not ((MAP[st[1]][st[0]].deg() > 1) and (MAP[end[1]][end[0]].deg() > 1))
    ):
        print("Please select a start and end point within the road network.")
        sys.exit()

    # do this because adjacency lists require unique nodes
    # and coordinates don't work correctly in dictionaries
    st_1d  = c2u(st)
    end_1d = c2u(end)

    # list of nodes
    to_visit = [MAP[st[1]][st[0]]]

    visited = []

    # element format: <(y*WIDTH) + x>: [(x1, y1, d1), (x2, y2, d2), ...]
    #           curr 1d coords    neighbour coords, distance
    adj_list  = {
        st_1d: [],
        end_1d: []
    }

    curr = MAP[st[1]][st[0]]
    while(len(to_visit) > 0):
        # check neighbour in each direction to find next node
        for i in range(0, 4):
            direction = ["N", "E", "S", "W"][i]
            if (curr.connected[direction]):
                sub = curr.neighbours[direction]
                distance = 1
                last_dir = find_rel_dirs(direction)["f"]
                while(sub.deg() < 3 and not ((sub.x == end[0]) and (sub.y == end[1]))):
                    rel_dirs = find_rel_dirs(last_dir)
                    if (sub.connected[rel_dirs["f"]]):
                        sub = sub.neighbours[rel_dirs["f"]]
                        last_dir = rel_dirs["b"]
                        distance += 1
                    elif (sub.connected[rel_dirs["l"]]):
                        sub = sub.neighbours[rel_dirs["l"]]
                        last_dir = rel_dirs["r"]
                        distance += 1
                    elif (sub.connected[rel_dirs["r"]]):
                        sub = sub.neighbours[rel_dirs["r"]]
                        last_dir = rel_dirs["l"]
                        distance += 1

                if (sub.deg() > 2) or ((sub.x == end[0]) and (sub.y == end[1])):
                    if (sub.x, sub.y, distance) not in adj_list[c2u((curr.x, curr.y))]:
                        adj_list[c2u((curr.x, curr.y))].append((sub.x, sub.y, distance))
                    if c2u((sub.x, sub.y)) not in adj_list:
                        adj_list[c2u((sub.x, sub.y))] = []

                    if (sub not in visited):
                        to_visit.append(sub)

        to_visit.remove(curr)
        visited.append(curr)
        if (len(to_visit) > 0):
            curr = to_visit[0]

    WEIGHTED_GRAPH = adj_list


def shortest_path():
    pass

# takes array containing node order of path and
# outputs a .yaml file
def output_path(path, st, end):
    with open('path.yaml', 'w+') as f:
        for j in range(0, len(MAP)):
            line = ""
            # fill up arrays
            for i in range(0, len(MAP[0])):
            	if ((i, j) == st):
            	    line += "S "
            	elif((i, j) == end):
            	    line += "X "
            	elif (i, j) in path:
            	    line += "1 "
            	else:
            	    line += "0 "
            	    # [:-1] takes away the trailing space
            f.write(line[:-1] + "\n")
    print("Path written to path.yaml")


def ascii_map():
    global MAP
    for j in range(0, len(MAP)):
        down_edges = []
        right_edges = []
        # fill up arrays
        for i in range(0, len(MAP[0])):
            if (MAP[j][i].connected["E"] == 1):
                right_edges.append(str(MAP[j][i].deg()) + "-")
            else:
                right_edges.append(str(MAP[j][i].deg()) + " ")

            if (MAP[j][i].connected["S"] == 1):
                down_edges.append("| ")
            else:
                down_edges.append("  ")

        # print right edges
        for k in range(0, len(right_edges)):
            sys.stdout.write(right_edges[k])
        sys.stdout.write("\n")

        # print down edges
        for m in range(0, len(down_edges)):
            sys.stdout.write(down_edges[m])
        sys.stdout.write("\n")

    sys.stdout.write("\n")
    sys.stdout.flush()

# returns set of relative directions on a degree 1 node
# according to its only connection
def find_rel_dirs(leading_connection):
    if (leading_connection == "S"):
        return {"f": "N",  # relative forward "f" is north "N"
                "r": "E",  # relative right "r" is east "E"
                "b": "S",  # relative backwards "b" is south "S"
                "l": "W"}  # relative left "l" is west "W"
    elif (leading_connection == "W"):
        return {"f": "E",
                "r": "S",
                "b": "W",
                "l": "N"}
    elif (leading_connection == "N"):
        return {"f": "S",
                "r": "W",
                "b": "N",
                "l": "E"}
    elif (leading_connection == "E"):
        return {"f": "W",
                "r": "N",
                "b": "E",
                "l": "S"}
    else:
        raise ValueError("ERROR: unrecognized direction string")
# ============================= NODE CLASS ====================================
class Node():
    global MAP
    def __init__(self, x, y):
        # cartesian coordinates in the map
        self.x = x
        self.y = y

        # the neighbouring Nodes
        self.neighbours = {"N":None, "E":None, "S":None, "W":None}

        # represents if self is connected to neighbours
        self.connected = {"N":0, "E":0, "S":0, "W":0}

    def deg(self):
        return self.connected["N"] + self.connected["E"] + \
               self.connected["S"] + self.connected["W"]

    def connect(self, direction):
        if (self.neighbours[direction] is not None):
            global ONES
            # connect on self's side
            if (self.deg() == 0):
                ONES.append((self.x, self.y))
            elif (self.deg() == 1):
                ONES.remove((self.x, self.y))
            self.connected[direction] = 1

            # connect on the conectee's side
            if (self.neighbours[direction].deg()==0):
                ONES.append((self.neighbours[direction].x, self.neighbours[direction].y))
            elif (self.neighbours[direction].deg()==1):
                ONES.remove((self.neighbours[direction].x, self.neighbours[direction].y))

            if   direction == "N":
                self.neighbours[direction].connected["S"] = 1
            elif direction == "E":
                self.neighbours[direction].connected["W"] = 1
            elif direction == "S":
                self.neighbours[direction].connected["N"] = 1
            elif direction == "W":
                self.neighbours[direction].connected["E"] = 1
        else:
            raise ValueError('ERROR: tried to connect "{}" to non-existant node from ({}, {})'.format(direction, str(self.x), str(self.y)))


    def disconnect(self, direction):
        if (self.neighbours[direction] is not None):
            global ONES
            # disconnect on self's side
            if (self.deg() == 1):
                ONES.remove((self.x, self.y))
            elif (self.deg() == 2):
                ONES.append((self.x, self.y))
            self.connected[direction] = 0


            # disconnect on the disconectee's side
            if (self.neighbours[direction].deg() == 1):
                ONES.remove((self.neighbours[direction].x, self.neighbours[direction].y))
            elif (self.neighbours[direction].deg() == 2):
                ONES.append((self.neighbours[direction].x, self.neighbours[direction].y))

            if   direction == "N":
                self.neighbours[direction].connected["S"] = 0
            elif direction == "E":
                self.neighbours[direction].connected["W"] = 0
            elif direction == "S":
                self.neighbours[direction].connected["N"] = 0
            elif direction == "W":
                self.neighbours[direction].connected["E"] = 0
        else:
            raise ValueError("ERROR: tried to connect to non-existant node")



    # IMPORTANT: Y-axis comes before X-axis due to how 2D arrays are represented
    # ( MAP[y][x] );    ex. MAP[5][0] => cartesian coordinates (0, 5)
    def set_neighbours(self):
        # north neighbour
        # if node is in the top row (y == 0), self.neighbours["N"] remains None
        try:
            if (self.y > 0):
                self.neighbours["N"] = MAP[self.y - 1][self.x]
        except:
            raise ValueError("ERROR: undefined north neighbour")

        # east neighbour
        # if node is in the rightmost column (x == len(MAP[0])-1), self.neighbours["E"] remains None
        try:
            if (self.x < len(MAP[0])-1):
                self.neighbours["E"] = MAP[self.y][self.x + 1]
        except:
            raise ValueError("ERROR: undefined east neighbour")

        # south neighbour
        # if node is in the bottom row (y == len(MAP)), self.neighbours["S"] remains None
        try:
            if (self.y < len(MAP)-1):
                self.neighbours["S"] = MAP[self.y + 1][self.x]
        except:
            raise ValueError("ERROR: undefined south neighbour")

        # west neighbour
        # if node is in the leftmost column (x == 0), self.neighbours["W"] remains None
        try:
            if (self.x > 0):
                self.neighbours["W"] = MAP[self.y][self.x - 1]
        except:
            raise ValueError("ERROR: undefined west neighbour")




#====================================== END ==============================================
#===========================================================================================


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Outputs the shortest path from startpoint to \
                        endpoint in the specified adjacency matrix .npz file")
    parser.add_argument("file", help="enter path of the.npz file generated by generator.py")
    parser.add_argument("start_x", help="x-coordinate of startpoint", type=int)
    parser.add_argument("start_y", help="y-coordinate of startpoint", type=int)
    parser.add_argument('end_x', help="x-coordinate of endpoint", type=int)
    parser.add_argument('end_y', help="y-coordinate of endpoint", type=int)

    args = parser.parse_args()

    main( args.file, (args.start_x, args.start_y), (args.end_x, args.end_y) )
