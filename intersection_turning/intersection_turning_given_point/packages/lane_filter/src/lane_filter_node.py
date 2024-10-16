#!/usr/bin/env python3
import json

import numpy as np

import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import FSMState, LanePose, SegmentList, Twist2DStamped, WheelEncoderStamped
from lane_filter import LaneFilterHistogram
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D



class LaneFilterNode(DTROS):
    """Generates an estimate of the lane pose.

    Creates a `lane_filter` to get estimates on `d` and `phi`, the lateral and heading deviation from the
    center of the lane.
    It gets the segments extracted by the line_detector as input and output the lane pose estimate.


    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~filter (:obj:`list`): A list of parameters for the lane pose estimation filter
        ~debug (:obj:`bool`): A parameter to enable/disable the publishing of debug topics and images

    Subscribers:
        ~segment_list (:obj:`SegmentList`): The detected line segments from the line detector
        ~car_cmd (:obj:`Twist2DStamped`): The car commands executed. Used for the predict step of the filter
        ~change_params (:obj:`String`): A topic to temporarily changes filter parameters for a finite time
        only
        ~switch (:obj:``BoolStamped): A topic to turn on and off the node. WARNING : to be replaced with a
        service call to the provided mother node switch service
        ~fsm_mode (:obj:`FSMState`): A topic to change the state of the node. WARNING : currently not
        implemented

    Publishers:
        ~lane_pose (:obj:`LanePose`): The computed lane pose estimate
        ~belief_img (:obj:`Image`): A debug image that shows the filter's internal state
        ~seglist_filtered (:obj:``SegmentList): a debug topic to send the filtered list of segments that
        are considered as valid

    """

    filter: LaneFilterHistogram
    bridge: CvBridge

    def __init__(self, node_name):
        super(LaneFilterNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self._filter = rospy.get_param("~lane_filter_histogram_configuration", None)
        self._debug = rospy.get_param("~debug", False)

        # Create the filter
        self.filter = LaneFilterHistogram(**self._filter)

        # Creating cvBridge
        self.bridge = CvBridge()

        self.t_last_update = rospy.get_time()
        self.currentVelocity = None

        self.latencyArray = []

        # Subscribers
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.cbProcessSegments, queue_size=1)

        self.sub_velocity = rospy.Subscriber("~car_cmd", Twist2DStamped, self.updateVelocity)

        self.sub_change_params = rospy.Subscriber("~change_params", String, self.cbTemporaryChangeParams)

        self.sub_encoder_left = rospy.Subscriber(
            "~left_wheel_encoder_node/tick", WheelEncoderStamped, self.cbProcessLeftEncoder, queue_size=1
        )

        self.sub_encoder_right = rospy.Subscriber(
            "~right_wheel_encoder_node/tick", WheelEncoderStamped, self.cbProcessRightEncoder, queue_size=1
        )



        # Publishers
        self.pub_lane_pose = rospy.Publisher(
            "~lane_pose", LanePose, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )

        self.pub_belief_img = rospy.Publisher(
            "~belief_img", Image, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.pub_seglist_filtered = rospy.Publisher(
            "~seglist_filtered", SegmentList, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        # Create the publisher for the position (x, y, theta)
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

        #rospy.Timer(rospy.Duration(1 / self._predict_freq), self.cbProcessSegments)

        # FSM
        # self.sub_switch = rospy.Subscriber(
        #     "~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)


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



    def cbTemporaryChangeParams(self, msg):
        """Callback that changes temporarily the filter's parameters.

        Args:
            msg (:obj:`String`): list of the new parameters

        """
        # This weird callback changes parameters only temporarily - used in the unicorn intersection.
        # comment from 03/2020
        data = json.loads(msg.data)
        params = data["params"]
        reset_time = data["time"]
        # Set all paramters which need to be updated
        for param_name in list(params.keys()):
            param_val = params[param_name]
            params[param_name] = eval("self.filter." + str(param_name))  # FIXME: really?
            exec("self.filter." + str(param_name) + "=" + str(param_val))  # FIXME: really?

        # Sleep for reset time
        rospy.sleep(reset_time)

        # Reset parameters to old values
        for param_name in list(params.keys()):
            param_val = params[param_name]

            exec("self.filter." + str(param_name) + "=" + str(param_val))  # FIXME: really?

    #    def nbSwitch(self, switch_msg):
    #        """Callback to turn on/off the node
    #
    #        Args:
    #            switch_msg (:obj:`BoolStamped`): message containing the on or off command
    #
    #        """
    #        # All calls to this message should be replaced directly by the srvSwitch
    #        request = SetBool()
    #        request.data = switch_msg.data
    #        eelf.nub_switch(request)

    def cbProcessSegments(self, segment_list_msg):
        """Callback to process the segments

        Args:
            segment_list_msg (:obj:`SegmentList`): message containing list of processed segments

        """
        # Get actual timestamp for latency measurement
        timestamp_before_processing = rospy.Time.now()

        # Step 1: predict

        
        
        current_time = rospy.get_time()
        if self.currentVelocity:
            dt = current_time - self.t_last_update
            #print("This is dt")
            #print(dt)
            self.filter.predict(dt=dt, v=self.currentVelocity.v, w=self.currentVelocity.omega)
            # Calculate v and w from ticks using kinematics
            ticks_to_meters = 2*np.pi*0.038/135
            v_left =  self.left_encoder_ticks_delta * ticks_to_meters
            v_right = self.right_encoder_ticks_delta * ticks_to_meters
            v = (v_left + v_right)/2
            w = (v_right - v_left)/0.1
            # KEEP THIS #
            self.x_g = self.x_g + v*np.cos(self.ang)
            self.y_g = self.y_g + v*np.sin(self.ang)
            self.ang = self.ang + w

            # Publish the current position
            pose_msg = Pose2D()
            pose_msg.x = self.x_g
            pose_msg.y = self.y_g
            pose_msg.theta = self.ang
            self.pub_position.publish(pose_msg)

            self.left_encoder_ticks += self.left_encoder_ticks_delta
            self.right_encoder_ticks += self.right_encoder_ticks_delta
            self.left_encoder_ticks_delta = 0
            self.right_encoder_ticks_delta = 0

            print("\n" + str(self.x_g) + ", " + str(self.y_g) + ", " + str(self.ang))

        self.t_last_update = current_time

        # Step 2: update
        self.filter.update(segment_list_msg.segments)

        # Step 3: build messages and publish things
        [d_max, phi_max] = self.filter.getEstimate()
        # print "d_max = ", d_max
        # print "phi_max = ", phi_max

        # Getting the highest belief value from the belief matrix
        max_val = self.filter.getMax()
        # Comparing it to a minimum belief threshold to make sure we are certain enough of our estimate
        in_lane = max_val > self.filter.min_max

        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = segment_list_msg.header.stamp
        lanePose.d = d_max
        lanePose.phi = phi_max
        lanePose.in_lane = in_lane
        # XXX: is it always NORMAL?
        lanePose.status = lanePose.NORMAL

        self.pub_lane_pose.publish(lanePose)
        self.debugOutput(segment_list_msg, d_max, phi_max, timestamp_before_processing)

    def debugOutput(self, segment_list_msg, d_max, phi_max, timestamp_before_processing):
        """Creates and publishes debug messages

        Args:
            segment_list_msg (:obj:`SegmentList`): message containing list of filtered segments
            d_max (:obj:`float`): best estimate for d
            phi_max (:obj:``float): best estimate for phi
            timestamp_before_processing (:obj:`float`): timestamp dating from before the processing

        """
        if self._debug:
            # Latency of Estimation including curvature estimation
            estimation_latency_stamp = rospy.Time.now() - timestamp_before_processing
            estimation_latency = estimation_latency_stamp.secs + estimation_latency_stamp.nsecs / 1e9
            self.latencyArray.append(estimation_latency)

            if len(self.latencyArray) >= 20:
                self.latencyArray.pop(0)

            # print "Latency of segment list: ", segment_latency
            # self.loginfo(f"Mean latency of Estimation:................. {np.mean(self.latencyArray)}")

            # Get the segments that agree with the best estimate and publish them
            inlier_segments = self.filter.get_inlier_segments(segment_list_msg.segments, d_max, phi_max)
            inlier_segments_msg = SegmentList()
            inlier_segments_msg.header = segment_list_msg.header
            inlier_segments_msg.segments = inlier_segments
            self.pub_seglist_filtered.publish(inlier_segments_msg)

            # Create belief image and publish it
            belief_img = self.bridge.cv2_to_imgmsg(
                np.array(255 * self.filter.belief).astype("uint8"), "mono8"
            )
            belief_img.header.stamp = segment_list_msg.header.stamp
            self.pub_belief_img.publish(belief_img)

    def cbMode(self, msg):
        return  # TODO adjust self.active

    def updateVelocity(self, twist_msg):
        self.currentVelocity = twist_msg


if __name__ == "__main__":
    lane_filter_node = LaneFilterNode(node_name="lane_filter_node")
    rospy.spin()