#!/usr/bin/env python
import rospy

from __future__ import division

from sensor_msgs.msg import CameraInfo, CompressedImage
from duckietown_msgs.msg import Twist2DStamped

import duckietown_utils as dtu
from duckietown_visualodo.algo.visual_odometry import VisualOdometry


class VisualOdometryNode(object):

    def __init__(self):

        # Set parameters
        self.node_name = "Visual Odometry Node"
        self.bridge = CvBridge()


        robot_name = rospy.get_param("~config_file_name", None)

        if robot_name is None:
            robot_name = dtu.get_current_robot_name()


        # Set subscribers
        self.sub_img = rospy.Subscriber("~image/compressed", CompressedImage, self.cbImage, queue_size=1)
        self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.cbCmd,queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)

        camera_info_topic = "/" + self.robot_name + "/camera_node/camera_info"
        rospy.loginfo("camera info topic is " + camera_info_topic)
        rospy.loginfo("waiting for camera info")
        camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo)
        rospy.loginfo("camera info received")

        # Set publishers
        self.path_publisher = rospy.Publisher("~path", Path, queue_size=1)
        self.odom_publisher = rospy.Publisher("~odometry", Odometry, queue_size=1)

    def setupParams(self):

        self.threshold_angle = self.setupParam("~threshold_angle")
        self.threshold_length = self.setupParam("~threshold_length")

        self.shrink_x_ratio = self.setupParam("~shrink_x_ratio")
        self.shrink_y_ratio = self.setupParam("~shrink_y_ratio")

        self.plot_matches = self.setupParam("~plot_matches")
        self.plot_histogram_filtering = self.setupParam("~plot_histogram_filtering")

        self.feature_extractor = self.setupParam("~feature_extractor")
        self.matcher = self.setupParam("~matcher")
        self.knn_neighbors = self.setupParam("~knn_neighbors")
        self.filter_by_histogram = self.setupParam("~filter_by_histogram")

        self.knn_weight = self.setupParam("~knn_weight")

    def setupParam(self,param_name,default_value):
		value = rospy.get_param(param_name,default_value)
		rospy.set_param(param_name,value) #Write to parameter server for transparancy
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value


    def cbImage(self, image_msg):
        # print('line_detector_node: image received!!')
        self.stats.received()

        if not self.active:
            return

        # TODO put here main logic!
        # Start a daemon thread to process the image

        #NOTE investigate on these lines
        # thread = threading.Thread(target=self.processImage,args=(image_msg,))
        # thread.setDaemon(True)
        # thread.start()
        # Returns rightaway

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

    def onShutdown(self):
        self.loginfo("Shutdown.")

if __name__ == '__main__':
    rospy.init_node('visual_odometry', anonymous=False)


    visual_odometry_node = VisualOdometryNode()

    rospy.on_shutdown(visual_odometry_node.onShutdown)
    rospy.spin()
