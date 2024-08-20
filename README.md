# Master_thesis

## neural_astar
Here, you can run notebook_vanilla_lidia.py in order to plan the path on circuit "super_dense_circuit.npz" for going from the given start point to the given goal point (given in notebook_vanilla_lidia.py file). The path is predicted using A* and Neural A* algorithms. The original version of this repository is at https://github.com/omron-sinicx/neural-astar.

## neural_astar/map-utils
This folder is not accesable, as it is basically a clone repository in order to generate the environment where we will perform the path planning algorithm (we used as example "super_dense_circuit.npz", that was generated using this repository). In order to learn how to use it, check the repository documentation at https://github.com/duckietown/map-utils). Out .npz circuit is an adjacent matrix that repesents the environment. This repository also generates a .yaml file, that will be use to render the environment in Gym-Duckietown simulation (https://github.com/duckietown/gym-duckietown).

## gym-duckietown
This folder is the one that has the files needed to run the gym-duckietown simulation. It is a clone of the repository https://github.com/duckietown/gym-duckietown. We created a new file, path-planning.py that runs the duckiebot in the simulation autonomously from one point to another of a given array of points saved at path_points in this file. It is designed for the --map-name "small_loop", that is the one that we currently have in the lab at university. We just need to execute the command './path_planning.py --env-name Duckietown-udem1-v0 --map-name "small_loop"' to make it run. The idea in future is to link this with the predicted path by neural_astar.

## my-ros-project
This is the first ros project I created following the documentation of "Duckietown". I created the node "twist_control_node.py" to access the wheel encoders and publish different linear and angular speed values to move the robot towards the objective. It is linked with the odometry_activity_given_speed.py" file, also saved in the same directory (./packages/my_package/src/twist_control_node.py). Currently it is not working as expected, as the displayed coordinates when moving in the room don't correspond to the real ones and the predicted speed it is not computed correctly. The commands to make it run are:
dts devel build -H <name-of-the-robot> -f

dts devel run -H <name-of-the-robot> -L twist-control

## state-estimation
Here we track the yellow and white lines. We can see them in the simulation and with the ground_projection node we can see the relation of the duckiebot respect to the lines that it observes. Also, we tried to mix state-estimation with obtaining the odometry. For the simulation it worked, but for the real life the robot is not doing the line following (future problem to check why this happens) and if we move the robot manually, the wheels encoders error is pretty big.
