# Master_thesis
Small remark for all these projects: the wheels encoders are not at cero once some program have been runned in the robot and it moved. If we run another program without first switching off the robot, the calculated initial coordinates won't be (0,0) but another value. This problem can be a future work to improve he working flow.

For the graphs of the position of the robot in the environment, there are the files "remove_rubish.py" and "text_to_python.py" in folders "follow_coordinates", "Final_fusion", "my-ros-project" and "state-estimation". In order to use them, we first save everything that the comand window is printing while moving the robot around. Then, with the file "remove_rubish.py" we clean every information that is not the calculated robot position. Finally, with the "text_to_python.py" file we transform the data to python and create the plot.

## Final_fusion
This is the final goal of the thesis. It is basically the Lane Following demo adapted for detecting the intersections and decide when to turn or not depending on the planned path with Neural A*. Some important nodes in order to understand the basic concept of how the code works:
  - Lane_controller_node.py:
    -   Line 234: It specified the folder where we have the training weights.
    -   Line 212: It specifies the architecture that is being used in the Neural Network.It is the default one in the Neural A* repository.
    -   Line 245-249: It specifies the size of the environment and the start and ending point as an index, not as a coordinate.
    -   Line 572-579: It calculates different paramenters, as distance to the current goal and different angles and when the robot is close to the current goal.
    -   Line 642-664: It is where it decides to turn or to go straight when an intersection is detected.
  - Lane_filter_node.py:
    - Line 233-266: The position of the robot is calculated based on the wheel encoders.
  - Stop_line_filter_node.py: It is in charge of detecting the red lines, the intersection lines.
  - When we run the program in the robot, we can see in the command window (after some time waiting) the displayed matrix of the prediction of the path in the given environment. In order to execute the code, it is necessary to copy everything that run from this folder the following commands:
    -   dts devel build -f -H duckie1 (name  of the robot)
    -   dts devel run -L lane_following -H duckie1 (name of the robot)
    It will be important to have opened "dts duckiebot keyboard_control duckie1" (name of the robot) virtual keyboard and press "a" in the keyboard for the robot to begin to move. The predicted path is the one described in red in photo path_comparison.png.

## follow_coordinates
In this project, we wanted to make the robot move on a square in an empty domain. So, assuming the robot is initially positioned at coordinate (0,0), we wanted it to go from here to (0, 1), (1, 1) and (1, 0). In order to track the coordinates of the robot we used the wheels encoders. We realised they are not really accurate and the results depend on the surface where the robot is moving or the calibration of the wheels among other factors. To run the code we must execute command "dts code workbench --duckiebot duckie1" (name of the robot) from the folder state-estimation. A couple of remarks:
  - State-estimation is an exercise project from the Duckietown-lx repository (https://github.com/duckietown/duckietown-lx). Despite of not really using any output of this exercise, we took the project as a base to modify it and achieve our results. Another important remark is that, once we have run the command to run the code in the robot, we need also to open the Virtual Joystick window with "dts duckiebot keyboard_control duckie1" and move the robot a bit to move the wheels so that the encoders are not exactly at cero. After doing this, we should close this virtual keyboard window and the robot will begin moving. Three important files to check to understand how the code works are in histogram_lane/histogram_lane_filter_node.py and solution/histogram_filter.py and solution/lane_filter.py.

## gym-duckietown
This folder is the one that has the files needed to run the gym-duckietown simulation. It is a clone of the repository https://github.com/duckietown/gym-duckietown. We created a new file, path-planning.py that runs the duckiebot in the simulation autonomously from one point to another of a given array of points saved at path_points in this file. It is designed for the --map-name "small_loop", that is the one that we currently have in the lab at university. We just need to execute the command './path_planning.py --env-name Duckietown-udem1-v0 --map-name "small_loop"' to make it run. 

## indefinite_navigation
This is an edition of the demo "Indefinite_navigation" whose code we can find at dt-core/packages/duckietown_demos in https://github.com/duckietown/dt-core/tree/daffy/packages. It is an old project developed for the Raspberry PI Duckiebots. We can read the small documentation in this link: https://docs-old.duckietown.org/daffy/opmanual_duckiebot/out/demo_indefinite_navigation.html. However, if we try to clone dt-core and run it as we would run the lane_following demo: dts devel build -f -H duckie1      and      dts devel run -L indefinite_navigation (or lane_following) -H duckie1, this does not work directly. Here, we have fixed the compatibility problems but still is not running as fully expected. When we place the duckiebot in the circuit with the ArUco codes as they should, these are detected and sometimes the robot is able to randomly select if turning or continuing straight, but other times it fails or stops. With a bit more of time invested, this code could be easily fixed and modified to work properly with these intersections. While working on this code we learnt how, later on, to deal with the intersections in our problem. 

## intersection_turning
This is a previous step to the Final_fusion project. In here, the path is not planned, but we give an array of coordinates, as if it was the planned path, and then the robot goes through these. In order to run it, we should execute the code the same way as for Final_fusion, so, 
  - dts devel build -f -H duckie1 (name  of the robot)
  - dts devel run -L lane_following -H duckie1 (name of the robot) 
The given path is the one described in red in photo path_comparison.png.

## line_following_with_coordinates
This project is the simple Duckietown Lane Following Demo with a modification in order to track the position of the robot. In order to make it run, it is just necessary to run the normal lane following demo and, in the command window we'll see the coordinates displayed:
  - dts devel build -f -H duckie1 (name  of the robot)
  - dts devel run -L lane_following -H duckie1 (name of the robot) 

## my-ros-project
This is the first ros project I created following the documentation of "Duckietown". I created the node "twist_control_node.py" to access the wheel encoders and publish different linear and angular speed values to move the robot towards the objective. It is linked with the odometry_activity_given_speed.py" file, also saved in the same directory (./packages/my_package/src/twist_control_node.py). If we move the robot manually with the virtual Joystick, we can see displayed in the command window the calculated position with the IMU and with the wheels encoders. With the IMU we reached the conclusion that there is a lot of noise and it is not calculating it properly. The commands to make it run are:
  - dts devel build -f -H <name-of-the-robot>
  - dts devel run -L twist-control -H <name-of-the-robot> 

## neural-astar-lidia
In this project we will select a virtual environment (we can generate it randomly in neural_astar/map-utils or manually by using the file in our_basic_circuit/our_environment_zip.py). In folder  generated_maps we have different environments in shape of adjacent matrices and some folders with some images of these environments and its predictions for testing different architectures. In folder model/mazes_032..../version_X, we'll have the trained weights of the model we are working with. If we want to test a different model, the weights for other models that we have trained are stored in folder "my_training". We should copy the desired weights from here into the model/mazes_blablabla folder when wanting to use them for testing this model. Before testing, we need to specify the model that we used for training in line 62 of files 'encoder.py' of folders 'src/neural_astar/planner' and 'src/neural_astar/utils'. One this is done we will run the program my running:
  - python notebook_vanilla_lidia.py
It will first display the architecture that we are using and, then, it will ask you to give the name of the environment you want to load form generated_maps folder. Next, it will ask you for the coordinates of the initial and ending point of the path we want to plan (top-left corner of the images will be coordinate (0, 0)). Finally, it will show you an image of the environment and the selected start and ending points. Then, it will display the predicted path. In the comand window we will see the path represented with coordinates, being (0,0) the starting point. Each tile will be separated by a distance of 0.59 meters. 

The original project that we took to develop out program is at https://github.com/omron-sinicx/neural-astar.

## neural_astar/map-utils
This folder is not accesable, as it is basically a clone repository in order to generate the environment where we will perform the path planning algorithm. In order to learn how to use it, check the repository documentation at https://github.com/duckietown/map-utils). Out .npz circuits are an adjacent matrix that repesents the environment. This repository also generates a .yaml file, that will be use to render the environment in Gym-Duckietown simulation (https://github.com/duckietown/gym-duckietown). The proper command to create an environment will be:
  - python ./generator.py --height 13 --width 13 --map-density "medium" --matrix-output, where we are specifying the dimensions of it and the density.

## powerpoint_presentations
This is just a summary of all the powerpoint presentations I developed during the thesis in order to track my work.

## state-estimation
Here we can obtain the coordinates calculated with the wheels encoders while moving the robot manually. In order to make it work we need to run "dts code workbench --duckiebot duckie1" (name of the robot).

## videos_and_photos
Here I just attach some videos of some real life tests
