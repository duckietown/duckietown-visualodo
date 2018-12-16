#!/usr/bin/env python

from __future__ import division

import rospy

from sensor_msgs.msg import CameraInfo, CompressedImage
from duckietown_msgs.msg import Twist2DStamped

from duckietown_visualodo.algo.visual_odometry import VisualOdometry

############################################
#              HYPERPARAMETERS             #
############################################


class AlignmentParameters:
    def __init__(self):
        self.threshold_angle = 0
        self.threshold_length = 0
        self.shrink_x_ratio = 0
        self.shrink_y_ratio = 0

        self.plot_matches = False
        self.plot_histogram_filtering = False

        self.feature_extractor = 'ORB'
        self.matcher = 'BF'
        self.knn_neighbors = 0
        self.filter_by_histogram = False

        self.knn_weight = [1.5]


def define_parameters():
    parameters = AlignmentParameters()

    # Accepted std deviations from average
    parameters.angle_th = 1  # Angular distribution
    parameters.length_th = 1.5  # Length distribution

    # Knn neighbors used. Cannot be changed from 2 right now
    parameters.knn_neighbors = 2

    # Image shrink factors in both dimensions
    parameters.shrink_x_ratio = 1 / 2
    parameters.shrink_y_ratio = 1 / 2

    # Publish debug images
    parameters.plot_matches = True
    parameters.plot_histogram_filtering = False

    # Knn weight ratio exploration. Relates how bigger must the first match be wrt the second to be considered a match
    # parameters.histogram_weigh = np.arange(1.9, 1.3, -0.05)
    parameters.knn_weight = [1.4]

    # Crop iterations counter. During each iteration the area of matching is reduced based on the most likely
    # region of last iteration
    parameters.crop_iterations = 1

    # Feature extraction and matching parameters
    parameters.matcher = 'BF'
    parameters.feature_extractor = 'ORB'

    parameters.filter_by_histogram = True

    return parameters


def call_save_image(data, vo_object):
    vo_object.save_image_and_trigger_vo(data)


def call_save_joy_command(data, vo_object):
    vo_object.save_command(data)


def call_save_camera_calibration(data, vo_object):
    vo_object.save_camera_calibration(data)


if __name__ == '__main__':
    rospy.init_node("image_listener", anonymous=True, log_level=rospy.INFO)

    try:
        input_parameters = define_parameters()

        visual_odometry = VisualOdometry(input_parameters)
        rospy.Subscriber("/maserati/camera_node/image/compressed", CompressedImage, call_save_image, visual_odometry)
        rospy.Subscriber("/maserati/joy_mapper_node/car_cmd", Twist2DStamped, call_save_joy_command, visual_odometry)
        rospy.Subscriber("/maserati/camera_node/camera_info", CameraInfo, call_save_camera_calibration, visual_odometry)

    except rospy.ROSInterruptException:
        pass

    rospy.spin()
