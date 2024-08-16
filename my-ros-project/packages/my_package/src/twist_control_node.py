#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
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
        
        # form the message
        self._v = VELOCITY
        self._omega = OMEGA

        # temporary data storage
        self._ticks_left = 0
        self._ticks_right = 0

        # construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)


    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        # store data value
        self._ticks_left = data.data - self._ticks_left 

    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        # store data value
        self._ticks_right = data.data - self._ticks_right

    def run(self):
        # publish 10 messages every second (10 Hz)
        global X, Y, THETA, delta_phi_l, delta_phi_r
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            delta_phi_r = self._ticks_right*(2*np.pi)/N - delta_phi_r
            delta_phi_l = self._ticks_left*(2*np.pi)/N - delta_phi_l
            print("This is delta_phi of r:")
            print(delta_phi_r)
            print("This is delta_phi of l:")
            print(delta_phi_l)
            X, Y, THETA, self._v, self._omega = pose_estimation(R, L, X, Y, THETA, delta_phi_l, delta_phi_r)
            message = Twist2DStamped(v=self._v, omega=self._omega)
        
            self._publisher.publish(message)
            msg = f"Wheel encoder ticks [LEFT, RIGHT]: {self._ticks_left}, {self._ticks_right}"
            rospy.loginfo(msg)
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
