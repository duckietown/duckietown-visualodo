#!/usr/bin/env python
from __future__ import division

import rospy
import yaml
import time
import numpy as np

from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo, CompressedImage
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, FSMState
from nav_msgs.msg import Odometry, Path

# import duckietown_utils as dtu
from duckietown_visualodo.algo.visual_odometry import VisualOdometry, VisualOdometryParameters


class VisualOdometryNode:

    def __init__(self):
        # Set parameters
        self.node_name = "Visual Odometry Node"
        self.bridge = CvBridge()

        self.active = True
        self.v = 0.0

        self.robot_name = rospy.get_param("~config_file_name", None)

        # if robot_name is None:
        #    self.robot_name = dtu.get_current_robot_name()

        self.visual_odometer = VisualOdometry()

        self.images = np.array([None, None])

        camera_info_topic = "/" + self.robot_name + "/camera_node/camera_info"
        rospy.loginfo("Camera info topic is " + camera_info_topic)
        rospy.loginfo("Waiting for camera info")
        camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo)
        rospy.loginfo("Camera info received")

        self.visual_odometer.get_camera_info(camera_info)

        # Set subscribers
        self.sub_img = rospy.Subscriber(
            "~image/compressed", CompressedImage, self.cb_image, queue_size=1)
        self.sub_for_kin = rospy.Subscriber("~for_kin_node_velocities", Twist2DStamped, self.cv_command, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cb_switch, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cb_mode, queue_size=1)

        # Set publishers
        self.path_publisher = rospy.Publisher("~path", Path, queue_size=1)
        self.odom_publisher = rospy.Publisher("~odometry", Odometry, queue_size=1)
        # TODO the visual odometry class instantiates an object of DataPlotter, which has more subscribers.
        # This is going to be a pain, so for the moment forget about it. I'll fix it

    def setup_params(self):
        # self.threshold_angle = self.setupParam("~threshold_angle")
        # self.threshold_length = self.setupParam("~threshold_length")
        # self.shrink_x_ratio = self.setupParam("~shrink_x_ratio")
        # self.shrink_y_ratio = self.setupParam("~shrink_y_ratio")
        #
        # self.plot_matches = self.setupParam("~plot_matches")
        # self.plot_histogram_filtering = self.setupParam("~plot_histogram_filtering")
        #
        # self.feature_extractor = self.setupParam("~feature_extractor")
        # self.matcher = self.setupParam("~matcher")
        # self.knn_neighbors = self.setupParam("~knn_neighbors")
        # self.filter_by_histogram = self.setupParam("~filter_by_histogram")
        # self.knn_weight = self.setupParam("~knn_weight")

        params_dir = "../../lib-visualodo/src/duckietown_visualodo/data/default.yaml"

        with open(params_dir, 'r') as stream:
            data_loaded = yaml.load(stream)
            params = data_loaded["parameters"]
            for param_name in params.keys():
                param_val = params[param_name]
                self.visual_odometer.set_parameter(param_name, param_val)
                rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, param_val))

    def cb_image(self, image_msg):
        self.stats.received()

        if not self.active:
            return

        start = time.time()

        cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        self.visual_odometer.get_image_and_trigger_vo(cv_image)

        rospy.logwarn("TIME: Total time: %s", time.time() - start)
        rospy.logwarn("===================================================")

        # TODO put here the main logic
        # Start a daemon thread to process the image

        # NOTE investigate on these lines -> in the thread run self.visual_odometer.visual_odometry_core(train_img, query_img)
        # thread = threading.Thread(target=self.processImage,args=(image_msg,))
        # thread.setDaemon(True)
        # thread.start()
        # Returns rightaway

    def cv_command(self, msg):
        # We just need to pass linear velocity
        self.visual_odometer.get_duckiebot_velocity(msg)

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

    def on_shutdown(self):
        self.loginfo("Shutdown.")


if __name__ == '__main__':
    rospy.init_node('visual_odometry', anonymous=False)

    visual_odometry_node = VisualOdometryNode()
    
    rospy.on_shutdown()
    rospy.spin()
