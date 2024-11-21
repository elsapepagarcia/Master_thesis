#!/usr/bin/env python3

import os
import time
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
from sensor_msgs.msg import Imu
from odometry_activity_given_speed import pose_estimation





# Twist command for controlling the linear and angular velocity of the frame
VELOCITY = 0  # linear vel    , in m/s    , forward (+)
OMEGA = 0    # angular vel   , rad/s     , counter clock wise (+)
X = 0.0
Y = 0.0
THETA = np.pi
R = 0.038
L = 0.1
N = 135
delta_phi_r = 0
delta_phi_l = 0



path_points = np.array([[1.0, 0.00, 1.0], [1.0, 0.00, 0.5],[1.0, 0.00, 0.0],[0.5, 0.00, 0.0],[0.0, 0.00, 0.0],  [0.0, 0.00, 0.5],[0.0, 0.00, 1.0],[0.5, 0.00, 1.0]])
  
path_point = path_points[1]
i = 1

class TwistControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"
        self._imu_topic = f"/{vehicle_name}/imu_node/data"  # IMU topic
        self.x, self.y = 0.0, 0.0
        self.vx, self.vy = 0.0, 0.0
        imu_topic_p =  f"/{vehicle_name}/imu_node/data"  # IMU topic
        self.time_now = time.time()
        self.time_before = time.time()
        
        # form the message
        self._v = VELOCITY
        self._omega = OMEGA

        # temporary data storage
        self._ticks_left = 0
        self._ticks_right = 0
        self._ticks_left_prev = self._ticks_left
        self._ticks_right_prev = self._ticks_right
        self._imu_data = None

        # Initialize IMU data attributes
        self._orientation = None
        self._angular_velocity = None
        self._linear_acceleration = None

        # Flag initializing
        self.initiate = False

        # construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self._publisher_imu = rospy.Publisher(imu_topic_p, Imu, queue_size=1)
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.sub_imu = rospy.Subscriber(self._imu_topic, Imu, self.callback_imu)  # IMU subscriber

        # Bias for accelerometer (calibrated when IMU is stationary)
        self.ax_bias, self.ay_bias = 0.0, 0.0

        # Calibrate the IMU to remove biases
        #self.calibrate_imu(self._linear_acceleration)
        self.start_calibration = False


    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        # store data value
        self._ticks_left = data.data - self._ticks_left 
        #self._ticks_left_prev = self._ticks_left

    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        # store data value
        self._ticks_right = data.data - self._ticks_right
        #self._ticks_right_prev = self._ticks_right
    

    def callback_imu(self, data):
        # log general information once at the beginning

        # store data value
        self._orientation = data.orientation
        self._angular_velocity = data.angular_velocity
        self._linear_acceleration = data.linear_acceleration

        rospy.loginfo_once(f"Orientation_callback: {data.orientation}")
        rospy.loginfo_once(f"Angular velocity_callback: {data.angular_velocity}")
        rospy.loginfo_once(f"Linear acceleration_callback: {data.linear_acceleration}")

    '''def calibrate_imu(self, imu_data):
        # Take some readings to calculate the average bias when the IMU is stationary
        bias_samples = 100
        ax_sum, ay_sum = 0.0, 0.0
        
        rospy.loginfo("Calibrating IMU, please keep it stationary...")
        while imu_data is None:
            # You can perform other tasks or simply pass
            pass
        for i in range(bias_samples):
            #imu_data = rospy.wait_for_message("/{vehicle_name}/imu_node/data", Imu)
            ax_sum += imu_data.linear_acceleration.x
            ay_sum += imu_data.linear_acceleration.y
            rospy.loginfo("Calibrating IMU...")
            rospy.loginfo("{i}")
            rospy.sleep(0.01)  # Sleep for a short duration between samples

        self.ax_bias = ax_sum / bias_samples
        self.ay_bias = ay_sum / bias_samples
        rospy.loginfo(f"Calibration complete: ax_bias={self.ax_bias}, ay_bias={self.ay_bias}")
'''

    def run(self):
        # publish 10 messages every second (10 Hz)
        global X, Y, THETA, delta_phi_l, delta_phi_r
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            delta_phi_r = self._ticks_right*(2*np.pi)/N - delta_phi_r
            delta_phi_l = self._ticks_left*(2*np.pi)/N - delta_phi_l
            #print("This is delta_phi of r:")
            #print(delta_phi_r)
            #print("This is delta_phi of l:")
            #print(delta_phi_l)
            X, Y, THETA, self._v, self._omega = pose_estimation(R, L, X, Y, THETA, delta_phi_l, delta_phi_r)
            message = Twist2DStamped(v=self._v, omega=self._omega)
            # Create the Imu message
            if self._imu_data is not None:
                msg_imu = Imu()
                msg_imu.header = self._imu_data.header
                msg_imu.orientation = self._imu_data.orientation
                msg_imu.orientation_covariance = self._imu_data.orientation_covariance
                msg_imu.angular_velocity = self._imu_data.angular_velocity
                msg_imu.angular_velocity_covariance = self._imu_data.angular_velocity_covariance
                msg_imu.linear_acceleration = self._imu_data.linear_acceleration
                msg_imu.linear_acceleration_covariance = self._imu_data.linear_acceleration_covariance
                
                # Publish the IMU message
                self._publisher_imu.publish(msg_imu)

            #self._publisher.publish(message)
            msg = f"Wheel encoder ticks [LEFT, RIGHT]: {self._ticks_left}, {self._ticks_right}"
            

            #orientation = f"Orientation: {self._orientation}"
            #angular_velocity = f"Angular velocity: {self._angular_velocity}"
            #linear_acceleration = f"Linear acceleration: {self._linear_acceleration}"
            
            if self.start_calibration == False:
                self.start_calibration = True
                bias_samples = 100
                ax_sum, ay_sum = 0.0, 0.0
                
                rospy.loginfo("Calibrating IMU, please keep it stationary...")
                while self._linear_acceleration is None:
                    # You can perform other tasks or simply pass
                    #print("It is None")
                    pass
                for i in range(bias_samples):
                    #imu_data = rospy.wait_for_message("/{vehicle_name}/imu_node/data", Imu)
                    ax_sum += self._linear_acceleration.x
                    ay_sum += self._linear_acceleration.y
                    #rospy.loginfo("Calibrating IMU...")
                    #rospy.loginfo("{i}")
                    rospy.sleep(0.01)  # Sleep for a short duration between samples

                self.ax_bias = ax_sum / bias_samples
                self.ay_bias = ay_sum / bias_samples
                #self._ticks_left = 0
                #self._ticks_right = 0
                #self._ticks_left_prev = self._ticks_left
                #self._ticks_right_prev = self._ticks_right
                print("Here!")
                print(self._ticks_left_prev)
                print(self._ticks_left)
                rospy.loginfo(f"Calibration complete: ax_bias={self.ax_bias}, ay_bias={self.ay_bias}")





            rospy.loginfo(msg)
            msg_sub = f"Orientation: {self._orientation}"
            rospy.loginfo(msg_sub)
            msg_sub = f"Angular: {self._angular_velocity}"
            rospy.loginfo(msg_sub)
            msg_sub = f"Acceleration: {self._linear_acceleration}"
            rospy.loginfo(msg_sub)
            self.time_now = time.time()

            if self._ticks_right_prev != self._ticks_right:
                print("This is self._ticks_right_prev")
                print(self._ticks_right)
                print("This is self._ticks_right")
                print(self._ticks_right_prev)

                if self._ticks_left_prev != self._ticks_left:
                    print("This is self._ticks_left_prev")
                    print(self._ticks_left_prev)
                    print("This is self._ticks_left")
                    print(self._ticks_left)

                    if  np.abs(self._ticks_left_prev-self._ticks_left) < 100:
                        if  np.abs(self._ticks_right_prev-self._ticks_right) < 100:

                            if self._linear_acceleration is not None:
                                # Extract acceleration
                                ax = self._linear_acceleration.x - self.ax_bias
                                ay = self._linear_acceleration.y - self.ay_bias
                                print("This is ax")
                                print(ax)
                                print(ay)


                                self.time_now = time.time()
                                print("This is the time")
                                print(self.time_now)
                                print("This is the time before")
                                print(self.time_before)
                                time_step = self.time_now - self.time_before
                                print("This is the time step")
                                print(time_step)
                                self.vx = ax*time_step
                                self.vy = ay*time_step

                                self.time_before = self.time_now
                                msg_sub = f"Velocity: {self.vx}, {self.vy}"
                                rospy.loginfo(msg_sub)
                                self.x += self.vx*time_step
                                self.y += self.vy*time_step
                                #msg_sub = f"Position: {self.x}, {self.y}"
                                msg_sub = f"Position with wheels encoders: {X}, {Y}"
                                rospy.loginfo(msg_sub)
                                msg_sub = f"Position calculated with IMU: {self.x}, {self.y}"
                                rospy.loginfo(msg_sub)
                
            self._ticks_left_prev = self._ticks_left
            self._ticks_right_prev = self._ticks_right
            self.time_before = self.time_now
            #rospy.loginfo(angular_velocity)
            #rospy.loginfo(linear_acceleration)
            rate.sleep()

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)


if __name__ == '__main__':
    # create the node
    node = TwistControlNode(node_name='twist_control_node')
    # run node
    node.run()
    
    # keep the process from terminating
    rospy.spin()