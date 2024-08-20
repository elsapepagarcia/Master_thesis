#!/usr/bin/env python3

import os
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

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"
        self._imu_topic = f"/{vehicle_name}/imu/data_raw"  # IMU topic
        imu_topic_p =  f"/{vehicle_name}/imu/data"  # IMU topic
        
        # form the message
        self._v = VELOCITY
        self._omega = OMEGA

        # temporary data storage
        self._ticks_left = 0
        self._ticks_right = 0
        self._imu_data = None

        # Initialize IMU data attributes
        self._orientation = None
        self._angular_velocity = None
        self._linear_acceleration = None

        # construct publisher
        #self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        #self._publisher_imu = rospy.Publisher(imu_topic_p, Imu, queue_size=1)
        #self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        #self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.sub_imu = rospy.Subscriber(self._imu_topic, Imu, self.callback_imu)  # IMU subscriber

    def callback_imu(self, data):
        # log general information once at the beginning
        
        rospy.loginfo_once(f"Orientation: {data.orientation}")
        rospy.loginfo_once(f"Angular velocity: {data.angular_velocity}")
        rospy.loginfo_once(f"Linear acceleration: {data.linear_acceleration}")

        # store data value
        self._orientation = data.orientation
        self._angular_velocity = data.angular_velocity
        self._linear_acceleration = data.linear_acceleration

    def run(self):
        # publish received tick messages every 0.05 second (20 Hz)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            #if self._orientation is not None:
                # start printing values when received from both encoders
            #    msg = f"Orientation: {self._orientation}"
            #    rospy.loginfo(msg)
            # start printing values when received from both encoders
            msg = f"Orientation: {self._orientation}"
            rospy.loginfo(msg)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node2 = MySubscriberNode(node_name='my_subscriber_node')
    # run node
    node2.run()

    
    # keep the process from terminating
    rospy.spin()
