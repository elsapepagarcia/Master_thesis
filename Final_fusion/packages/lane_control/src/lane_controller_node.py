#!/usr/bin/env python3
import numpy as np
import shutil
import rospy
import time
import torch
import numpy as np
import matplotlib.pyplot as plt
import os
from src.neural_astar.planner import NeuralAstar, VanillaAstar
from src.neural_astar.utils.training import load_from_ptl_checkpoint
from src.neural_astar.utils.lidia_data import create_dataloader
from src.neural_astar.utils.lidia_data import visualize_results


from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import (
    Twist2DStamped,
    LanePose,
    WheelsCmdStamped,
    BoolStamped,
    FSMState,
    StopLineReading,
    WheelEncoderStamped,
)

from lane_controller.controller import LaneController
from geometry_msgs.msg import Pose2D, Point, PoseStamped
from nav_msgs.msg import Path

class LaneControllerNode(DTROS):
    """Computes control action.
    The node compute the commands in form of linear and angular velocities, by processing the estimate error in
    lateral deviationa and heading.
    The configuration parameters can be changed dynamically while the node is running via ``rosparam set`` commands.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
    Configuration:
        ~v_bar (:obj:`float`): Nominal velocity in m/s
        ~k_d (:obj:`float`): Proportional term for lateral deviation
        ~k_theta (:obj:`float`): Proportional term for heading deviation
        ~k_Id (:obj:`float`): integral term for lateral deviation
        ~k_Iphi (:obj:`float`): integral term for lateral deviation
        ~d_thres (:obj:`float`): Maximum value for lateral error
        ~theta_thres (:obj:`float`): Maximum value for heading error
        ~d_offset (:obj:`float`): Goal offset from center of the lane
        ~integral_bounds (:obj:`dict`): Bounds for integral term
        ~d_resolution (:obj:`float`): Resolution of lateral position estimate
        ~phi_resolution (:obj:`float`): Resolution of heading estimate
        ~omega_ff (:obj:`float`): Feedforward part of controller
        ~verbose (:obj:`bool`): Verbosity level (0,1,2)
        ~stop_line_slowdown (:obj:`dict`): Start and end distances for slowdown at stop lines

    Publisher:
        ~car_cmd (:obj:`Twist2DStamped`): The computed control action
    Subscribers:
        ~lane_pose (:obj:`LanePose`): The lane pose estimate from the lane filter
        ~intersection_navigation_pose (:obj:`LanePose`): The lane pose estimate from intersection navigation
        ~wheels_cmd_executed (:obj:`WheelsCmdStamped`): Confirmation that the control action was executed
        ~stop_line_reading (:obj:`StopLineReading`): Distance from stopline, to reduce speed
        ~obstacle_distance_reading (:obj:`stop_line_reading`): Distancefrom obstacle virtual stopline, to reduce speed
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Add the node parameters to the parameters dictionary
        # TODO: MAKE TO WORK WITH NEW DTROS PARAMETERS
        self.params = dict()
        self.params["~v_bar"] = DTParam("~v_bar", param_type=ParamType.FLOAT, min_value=0.0, max_value=5.0)
        self.params["~k_d"] = DTParam("~k_d", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_theta"] = DTParam(
            "~k_theta", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0
        )
        self.params["~k_Id"] = DTParam("~k_Id", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_Iphi"] = DTParam(
            "~k_Iphi", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0
        )
        #self.params["~theta_thres"] = rospy.get_param("~theta_thres", None)
        #Breaking up the self.params["~theta_thres"] parameter for more finer tuning of phi
        self.params["~theta_thres_min"] = DTParam("~theta_thres_min", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)  #SUGGESTION mandatorizing the use of DTParam inplace of rospy.get_param for parameters in the entire dt-core repository as it allows active tuning while Robot is in action.
        self.params["~theta_thres_max"] = DTParam("~theta_thres_max", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0) 
        self.params["~d_thres"] = rospy.get_param("~d_thres", None)
        self.params["~d_offset"] = rospy.get_param("~d_offset", None)
        self.params["~integral_bounds"] = rospy.get_param("~integral_bounds", None)
        self.params["~d_resolution"] = rospy.get_param("~d_resolution", None)
        self.params["~phi_resolution"] = rospy.get_param("~phi_resolution", None)
        self.params["~omega_ff"] = rospy.get_param("~omega_ff", None)
        self.params["~verbose"] = rospy.get_param("~verbose", None)
        self.params["~stop_line_slowdown"] = rospy.get_param("~stop_line_slowdown", None)

        # Need to create controller object before updating parameters, otherwise it will fail
        self.controller = LaneController(self.params)
        # self.updateParameters() # TODO: This needs be replaced by the new DTROS callback when it is implemented

        # Initialize variables
        self.fsm_state = None
        self.wheels_cmd_executed = WheelsCmdStamped()
        self.pose_msg = LanePose()
        self.pose_initialized = False
        self.pose_msg_dict = dict()
        self.last_s = None
        self.stop_line_distance = None
        self.stop_line_detected = False
        self.at_stop_line = False
        self.obstacle_stop_line_distance = None
        self.obstacle_stop_line_detected = False
        self.at_obstacle_stop_line = False

        self.current_pose_source = "lane_filter"

        # Flag for prediction #
        self.path_predicted = False

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )
        self.pub_flag_lane = rospy.Publisher("/lane_following", BoolStamped, queue_size=1)
        self.pub_flag_int = rospy.Publisher("/end_intersection", BoolStamped, queue_size=1)
        rospy.Subscriber("/lane_following", BoolStamped, self.lane_callback)
        rospy.Subscriber("/end_intersection", BoolStamped, self.intersection_callback)

        self.pub_position = rospy.Publisher("/lane_filter_node/lane_position", Pose2D, queue_size=1, dt_topic_type=TopicType.PERCEPTION
)



        self.right_encoder_ticks = 0
        self.left_encoder_ticks = 0
        self.right_encoder_ticks_delta = 0
        self.left_encoder_ticks_delta = 0
        self.last_encoder_stamp = None
        self.x_g = 0
        self.y_g = 0
        self.ang = 0

        self.i = 1
        self.time_end = 0
        #self.path_points = np.array([[0.81, 0.0, 0.71], [1.2, 0.0, 1.21], [1.71, 0.0, 0.7], [1.21, 0.0, 0.21]]) #
        #self.path_points = np.array([[0.0, 0.0, 0.0], [0.59, 0.0, 0.0], [0.59, 0.0, 0.59], [0.59, 0.0, 1.18], [1.18, 0.0, 1.18], [1.77, 0.0, 1.18]]) #
        self.path_points = np.array([0.0, 0.0, 0.0])

        self.path_point = np.array([0.0, 0.0, 0.0])


        self.lane_follow = False
        self.intersection = False


        self.log("Initialized!")

        # Wait a short time to ensure that the publisher is registered
        #rospy.sleep(1)
        
        # Predict and publish the path once
        self.predict_and_publish_path()



        # Construct subscribers
        self.sub_lane_reading = rospy.Subscriber(
            "~lane_pose", LanePose, self.cbAllPoses, "lane_filter", queue_size=1
        )
        self.sub_intersection_navigation_pose = rospy.Subscriber(
            "~intersection_navigation_pose",
            LanePose,
            self.cbAllPoses,
            "intersection_navigation",
            queue_size=1,
        )
        self.sub_wheels_cmd_executed = rospy.Subscriber(
            "~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmdExecuted, queue_size=1
        )
        self.sub_stop_line = rospy.Subscriber(
            "~stop_line_reading", StopLineReading, self.cbStopLineReading, queue_size=1
        )
        self.sub_obstacle_stop_line = rospy.Subscriber(
            "~obstacle_distance_reading", StopLineReading, self.cbObstacleStopLineReading, queue_size=1
        )

        self.sub_encoder_left = rospy.Subscriber(
            "~left_wheel_encoder_node/tick", WheelEncoderStamped, self.cbProcessLeftEncoder, queue_size=1
        )

        self.sub_encoder_right = rospy.Subscriber(
            "~right_wheel_encoder_node/tick", WheelEncoderStamped, self.cbProcessRightEncoder, queue_size=1
        )

        rospy.Subscriber("/lane_filter_node/lane_position", Pose2D, self.position_callback)

        #self.path_publisher = rospy.Publisher("/predicted_path", Path, queue_size=1)
        #self.path_subscriber = rospy.Subscriber("/predicted_path", Path, self.predicted_path)


    def predict_and_publish_path(self):
        import numpy as np
        if not self.path_predicted:
            rospy.loginfo("Predicting and publishing the path...")

            # Create a Path message
            #path_msg = Path()
            #path_msg.header.stamp = rospy.Time.now()
            #path_msg.header.frame_id = "map"  # You can change this to an appropriate frame

            # The Neural A* implementation #

            device = "cuda" if torch.cuda.is_available() else "cpu"

            neural_astar = NeuralAstar(encoder_arch='CNN').to(device)
            # Get the current working directory
            current_directory = os.getcwd()

            print("This is the current directory")
            print(current_directory)
            # List all files and directories in the current directory
            all_items = os.listdir(current_directory)

            # Filter and print only directories
            folders = [item for item in all_items if os.path.isdir(os.path.join(current_directory, item))]
            print("Folders:", folders)

            current_directory = "/code/catkin_ws/src/dt-core/packages/lane_control/src/"
            # List all files and directories in the current directory
            all_items = os.listdir(current_directory)

            # Filter and print only directories
            folders = [item for item in all_items if os.path.isdir(os.path.join(current_directory, item))]
            print("Folders:", folders)


            neural_astar.load_state_dict(load_from_ptl_checkpoint("/code/catkin_ws/src/dt-core/packages/lane_control/src/model/mazes_032_moore_c8/lightning_logs/"))

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
            dataloader = create_dataloader("/code/catkin_ws/src/dt-core/packages/lane_control/src/our_environment.npz", start, end)
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


            # Access the paths tensor
            paths_tensor = na_outputs.paths

            # Convert the PyTorch tensor to a NumPy array
            # If the tensor is on GPU, you'll need to move it to CPU first using .cpu() before calling .numpy()
            #paths_numpy = paths_tensor.cpu().numpy() if paths_tensor.is_cuda else paths_tensor.numpy()


            import sys
            print("Python executable:", sys.executable)
            try:
                import numpy as np
                print("Numpy version:", np.__version__)
            except ImportError:
                print("Numpy is not available in this environment.")



            #paths_numpy = np.asarray(paths_tensor.cpu()) if paths_tensor.is_cuda else np.asarray(paths_tensor)

            '''if paths_tensor.is_cuda:
                paths_tensor = paths_tensor.cpu()
            paths_numpy = np.asarray(paths_tensor)


            # Optionally, reshape or remove any extra dimensions (depends on what you need)
            paths_numpy = paths_numpy.squeeze()  # This will remove dimensions of size 1 if necessary

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
                
                return coords'''
            

            # Ensure the tensor is on CPU if necessary, otherwise keep on GPU
            if paths_tensor.is_cuda:
                paths_tensor = paths_tensor.cpu()

            # Optionally, squeeze the tensor to remove dimensions of size 1
            paths_tensor = torch.squeeze(paths_tensor)

            # Define the grid cell size in meters (59 cm)
            cell_size = 0.59

            # Find the coordinates of each point in the planned path
            
            # Find the coordinates of each point in the planned path
            
            # Find the coordinates of each point in the planned path
            def get_coordinates_from_tensor(matrix_tensor, start_idx_tensor, cell_size):
                coords = []
                rows, cols = matrix_tensor.shape

                # Convert start_idx_tensor to a tuple of Python scalars
                start_idx = (start_idx_tensor[0], start_idx_tensor[1])

                # Iterate through the matrix to get the indices of the planned path (1s)
                for i in range(rows):
                    for j in range(cols):
                        if matrix_tensor[i, j].item() == 1:  # Check if it's part of the path
                            # Translate matrix indices (i, j) to real-world coordinates using start_idx_tensor as origin
                            coord_x = (i - start_idx[0]) * cell_size  # Adjust row by start_idx
                            coord_y = (j - start_idx[1]) * cell_size  # Adjust column by start_idx
                            coords.append((coord_x, coord_y))
                
                return coords

            # Example usage
            # Assuming start_idx is a PyTorch tensor like: start_idx_tensor = torch.tensor([start_row, start_col])
            #start_idx_tensor = torch.tensor([0, 0])  # Example start index (adapt this to your scenario)
            self.path_points = get_coordinates_from_tensor(paths_tensor, start_idx, cell_size)



            # Get the coordinates of the planned path
            #self.path_points = get_coordinates(paths_numpy, start, cell_size)

            print("These are the predicted coordinates in the lane_controller file:")
            print(self.path_points)

            '''
            fig, axes = plt.subplots(2, 1, figsize=[12, 4])
            axes[0].imshow(visualize_results(map_designs_tensor, na_outputs, scale=1, start=start_idx, end=end_idx))
            axes[0].set_title("Neural A*")
            axes[0].axis("off")
            axes[1].imshow(visualize_results(map_designs_tensor, va_outputs, start=start_idx, end=end_idx))
            axes[1].set_title("Vanilla A*")
            axes[1].axis("off")
            plt.show()
            '''

            # Fill the path message with points
            #for point in self.path_points:
            #    pose = PoseStamped()
            #    pose.header.stamp = rospy.Time.now()
            #    pose.header.frame_id = "map"
            #    pose.pose.position = Point(x=point[0], y=point[1], z=point[2])
            #    pose.pose.orientation.w = 1.0  # No orientation, just a straight path
            #    path_msg.poses.append(pose)

            # Publish the predicted path
            #self.path_publisher.publish(path_msg)

            # Set the flag to true to prevent re-publishing
            self.path_predicted = True
            rospy.loginfo("Path published successfully!")

    def path_callback(self, msg):
        # This is where you handle the incoming path message
        rospy.loginfo("Received a path message, continuing with the rest of the node logic...")

        # Example of processing the received path (just logging in this case)
        for pose in msg.poses:
            rospy.loginfo("Point: x=%f, y=%f, z=%f", 
                          pose.pose.position.x, 
                          pose.pose.position.y, 
                          pose.pose.position.z)

        # Continue with the rest of the node logic here
        # Example: Your main loop or other code that reacts to the new path
        rospy.loginfo("Executing the rest of the node...")



    def position_callback(self, msg):
        rospy.loginfo("Received Position - x: {}, y: {}, theta: {}".format(msg.x, msg.y, msg.theta))
        self.x_g = msg.x
        self.y_g = msg.y
        self.ang = msg.theta
    
    def lane_callback(self, msg):
        rospy.loginfo("This is the value of the lane flag: {}".format(msg.data))
        self.lane_follow = msg.data

    def intersection_callback(self, msg):
        rospy.loginfo("This is the value of the intersection flag: {}".format(msg.data))
        self.intersection = msg.data



    def cbProcessLeftEncoder(self, left_encoder_msg):
        if not self.filter.initialized:
            self.filter.encoder_resolution = left_encoder_msg.resolution
            self.filter.initialized = True
        self.left_encoder_ticks_delta = left_encoder_msg.data - self.left_encoder_ticks
        self.last_encoder_stamp = left_encoder_msg.header.stamp

    def cbProcessRightEncoder(self, right_encoder_msg):
        if not self.filter.initialized:
            self.filter.encoder_resolution = right_encoder_msg.resolution
            self.filter.initialized = True
        self.right_encoder_ticks_delta = right_encoder_msg.data - self.right_encoder_ticks
        self.last_encoder_stamp = right_encoder_msg.header.stamp



    def cbObstacleStopLineReading(self, msg):
        """
        Callback storing the current obstacle distance, if detected.

        Args:
            msg(:obj:`StopLineReading`): Message containing information about the virtual obstacle stopline.
        """
        self.obstacle_stop_line_distance = np.sqrt(msg.stop_line_point.x**2 + msg.stop_line_point.y**2)
        self.obstacle_stop_line_detected = msg.stop_line_detected
        self.at_stop_line = msg.at_stop_line

    def cbStopLineReading(self, msg):
        """Callback storing current distance to the next stopline, if one is detected.

        Args:
            msg (:obj:`StopLineReading`): Message containing information about the next stop line.
        """
        self.stop_line_distance = np.sqrt(msg.stop_line_point.x**2 + msg.stop_line_point.y**2)
        self.stop_line_detected = msg.stop_line_detected
        self.at_obstacle_stop_line = msg.at_stop_line

    def cbMode(self, fsm_state_msg):

        self.fsm_state = fsm_state_msg.state  # String of current FSM state

        if self.fsm_state == "INTERSECTION_CONTROL":
            self.current_pose_source = "intersection_navigation"
        else:
            self.current_pose_source = "lane_filter"

        if self.params["~verbose"] == 2:
            self.log("Pose source: %s" % self.current_pose_source)

    def cbAllPoses(self, input_pose_msg, pose_source):
        """Callback receiving pose messages from multiple topics.

        If the source of the message corresponds with the current wanted pose source, it computes a control command.

        Args:
            input_pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
            pose_source (:obj:`String`): Source of the message, specified in the subscriber.
        """

        if pose_source == self.current_pose_source:
            self.pose_msg_dict[pose_source] = input_pose_msg

            self.pose_msg = input_pose_msg

            self.getControlAction(self.pose_msg)

    def cbWheelsCmdExecuted(self, msg_wheels_cmd):
        """Callback that reports if the requested control action was executed.

        Args:
            msg_wheels_cmd (:obj:`WheelsCmdStamped`): Executed wheel commands
        """
        self.wheels_cmd_executed = msg_wheels_cmd

    def publishCmd(self, car_cmd_msg):
        """Publishes a car command message.

        Args:
            car_cmd_msg (:obj:`Twist2DStamped`): Message containing the requested control action.
        """
        self.pub_car_cmd.publish(car_cmd_msg)

    def getControlAction(self, pose_msg):
        """Callback that receives a pose message and updates the related control command.

        Using a controller object, computes the control action using the current pose estimate.

        Args:
            pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
        """
        current_s = rospy.Time.now().to_sec()
        dt = None
        if self.last_s is not None:
            dt = current_s - self.last_s

        #print(str(time.time() - self.time_end))
        #self.path_points = self.path_points.numpy()
        

        if self.i != len(self.path_points):
            self.path_point = self.path_points[self.i]

            distance = np.sqrt((self.path_point[0] - self.x_g)**2 + (self.path_point[1] - self.y_g)**2)

        if self.i == len(self.path_points):
            v = 0
            omega = 0

            car_control_msg = Twist2DStamped()
            car_control_msg.header = pose_msg.header
            car_control_msg.v = v
            car_control_msg.omega = omega

            self.publishCmd(car_control_msg)


        elif distance < 0.5:
            self.i = self.i + 1
            pose_msg = Pose2D()
            pose_msg.x = self.path_point[0]
            pose_msg.y = self.path_point[1]
            self.pub_position.publish(pose_msg)
            print("This is self.i:")
            print(self.i)


        elif (self.at_stop_line or self.at_obstacle_stop_line) and (time.time() - self.time_end > 0.5) and (self.x_g != 0 and self.y_g !=0):
        #elif (self.at_stop_line or self.at_obstacle_stop_line):
            
            # IZQUIERDA path_point = np.array([0.85, 0.0, -0.71])
            #self.path_point = self.path_points[self.i]
            print("LANE DETECTED")
            print("This is self.i:")
            print(self.i)
            #if self.i != (len(self.path_points) - 1):
            if self.i ==1:
                self.i = self.i + 1 # I am not sure if this works
            if self.i != len(self.path_points):
                

                #ticks_to_meters = 2*np.pi*0.038/135
                #v_left =  self.left_encoder_ticks_delta * ticks_to_meters
                #v_right = self.right_encoder_ticks_delta * ticks_to_meters
                #v = (v_left + v_right)/2
                #w = (v_right - v_left)/0.1
                # KEEP THIS #
                #self.x_g = self.x_g + v*np.cos(self.ang)
                #self.y_g = self.y_g + v*np.sin(self.ang)
                #self.ang = self.ang + w

                #self.left_encoder_ticks += self.left_encoder_ticks_delta
                #self.right_encoder_ticks += self.right_encoder_ticks_delta
                #self.left_encoder_ticks_delta = 0
                #self.right_encoder_ticks_delta = 0




                self.angle = self.ang #+ np.pi/2
                
                #target_angle = np.arctan2(path_point[0]-self.x, path_point[2] - self.y)
                target_angle = np.arctan2(self.path_points[self.i][1] - self.y_g, self.path_points[self.i][0] - self.x_g)
                print("INTERSECTION!")
                msg = BoolStamped()
                msg.data = True
                self.pub_flag_int.publish(msg)

                print("This is the target_angle" + str(target_angle))
                if target_angle < 0:
                    if target_angle < -np.pi/2:
                        target_angle = 2*np.pi + target_angle
                    angle_error = target_angle - self.angle
                    
                    if angle_error < -np.pi:
                        target_angle = 2*np.pi + target_angle
                
                angle_error = target_angle - self.angle

                
                
                #if target_angle >= 0:
                #    if (self.angle - np.pi) > target_angle:
                #        angle_error = 2*np.pi - self.angle + target_angle


                #if self.path_point[2] > self.y_g or self.path_point[0] < self.x_g:
                if self.path_points[self.i][1] == self.path_points[self.i-2][1] or self.path_points[self.i][0] == self.path_points[self.i-2][0]:

                    print("This is the first comparison:")
                    print(self.path_points[self.i][1] )
                    print(self.path_points[self.i-2][1])
                    print("This is the second comparison:")
                    print(self.path_points[self.i][0])
                    print(self.path_points[self.i-2][0])
                    v = 0.4
                    omega = 0.0
                elif angle_error >= 0:
                    # Left
                    ka = 0.6
                    kp = 0.9 #0.45
                    print("LEFT: " + str(self.y_g) + ", " + str(self.x_g))
                    print("This is angle_error: " + str(angle_error))
                    omega = 2.0
                else:
                    ka = 0.4 #0.3
                    kp = 1.1 #3.1
                    print("RIGHT: " + str(self.y_g) + ", " + str(self.x_g))
                    print("This is angle_error: " + str(angle_error))
                    omega = -7.0
                
                print("This is the point position: ")
                print(self.i)
                #print("This is the position:")
                #print(self.x_g)
                #print(self.y_g)
                #print("This is the point: ")
                #print(self.path_point[0])
                #print(self.path_point[2])

                v = ka
                #omega = 3*kp*angle_error
                
                print("This is the speed in the turn:")
                print(str(omega))



                #v = 0.7
                #omega = 4

                # Initialize car control msg, add header from input message
                car_control_msg = Twist2DStamped()
                car_control_msg.header = pose_msg.header
                car_control_msg.v = v
                car_control_msg.omega = omega

                self.publishCmd(car_control_msg)
                time.sleep(3.0)

                #car_control_msg.v = 0
                #car_control_msg.omega = 0

                #self.publishCmd(car_control_msg)
                #time.sleep(5)
                self.time_end = time.time()
            '''else:
                self.i = self.i + 1
                print("This is self.i in the else:")
                print(self.i)
                v = 0
                omega = 0

                car_control_msg = Twist2DStamped()
                car_control_msg.header = pose_msg.header
                car_control_msg.v = v
                car_control_msg.omega = omega

                self.publishCmd(car_control_msg)'''


        

        else:

            if self.lane_follow and self.intersection:
                msg = BoolStamped()
                msg.data = False
                self.pub_flag_lane.publish(msg)
                msg = BoolStamped()
                msg.data = False
                self.pub_flag_int.publish(msg)

            

            print("FOLLOW THE LANE")
            msg = BoolStamped()
            msg.data = True
            self.pub_flag_lane.publish(msg)

            # Compute errors
            d_err = pose_msg.d - self.params["~d_offset"]
            phi_err = pose_msg.phi

            # We cap the error if it grows too large
            if np.abs(d_err) > self.params["~d_thres"]:
                self.log("d_err too large, thresholding it!", "error")
                d_err = np.sign(d_err) * self.params["~d_thres"]
            
            if phi_err > self.params["~theta_thres_max"].value or phi_err < self.params["~theta_thres_min"].value:
                self.log("phi_err too large/small, thresholding it!", "error")
                phi_err = np.maximum(self.params["~theta_thres_min"].value, np.minimum(phi_err, self.params["~theta_thres_max"].value))

            wheels_cmd_exec = [self.wheels_cmd_executed.vel_left, self.wheels_cmd_executed.vel_right]
            if self.obstacle_stop_line_detected:
                v, omega = self.controller.compute_control_action(
                    d_err, phi_err, dt, wheels_cmd_exec, self.obstacle_stop_line_distance
                )
                # TODO: This is a temporarily fix to avoid vehicle image detection latency caused unable to stop in time.
                v = v * 0.25
                omega = omega * 0.25

            else:
                v, omega = self.controller.compute_control_action(
                    d_err, phi_err, dt, wheels_cmd_exec, self.stop_line_distance
                )

            # For feedforward action (i.e. during intersection navigation)
            omega += self.params["~omega_ff"]

        print("This is the v: " + str(v))
        print("This is the omega: " + str(omega))
        print("This is the path point: " + str(self.path_point[0]) + " " + str(self.path_point[1]))
        # Initialize car control msg, add header from input message
        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header

        # Add commands to car message
        car_control_msg.v = v
        car_control_msg.omega = omega

        self.publishCmd(car_control_msg)
        self.last_s = current_s

    def cbParametersChanged(self):
        """Updates parameters in the controller object."""

        self.controller.update_parameters(self.params)


if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = LaneControllerNode(node_name="lane_controller_node")
    # Keep it spinning
    rospy.spin()
