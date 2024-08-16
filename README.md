# Master_thesis

## neural_astar
Here, you can run notebook_vanilla_lidia.py in order to plan the path on circuit "super_dense_circuit.npz" for going from the given start point to the given goal point (given in notebook_vanilla_lidia.py file). The path is predicted using A* and Neural A* algorithms.

## neural_astar/map-utils
In this folder we can generate a circuit (We used as example "super_dense_circuit.npz", that was generated using this repository). In order to learn how to use it, check the repository documentation at https://github.com/duckietown/map-utils). Out .npz circuit is an adjacent matrix that repesents the environment. This repository also generates a .yaml file, that will be use to render the environment in Gym-Duckietown simulation (https://github.com/duckietown/gym-duckietown).
