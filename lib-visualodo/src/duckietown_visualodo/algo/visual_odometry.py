#!/usr/bin/env python

# -*- coding: utf-8 -*-

from __future__ import division

import time
import cv2
import numpy as np

import tf.transformations

from cv_bridge import CvBridge

from data_plotter import DataPlotter
from match_filters import HistogramLogicFilter, DistanceFilter
from utils import knn_match_filter, rotation_matrix_to_euler_angles, create_geometric_mask
from image_manager import ImageManager

from geometry_msgs.msg import Vector3, Quaternion, Transform

from duckietown_msgs.msg import Twist2DStamped


class VisualOdometry:
    def __init__(self):
        self.images = np.array([ImageManager(), ImageManager()])
        self.bridge = CvBridge()

        # Intrinsic camera calibration matrices
        self.camera_K = None
        self.camera_D = None

        # Image sizes
        self.image_h = 1
        self.image_w = 1

        # Current command sent to duckiebot
        self.joy_command = Twist2DStamped()

        # Initiate the feature detector
        self.cv2_detector = None

        self.last_ros_time = time.time()
        self.last_theta = 0

        # Initialize parameters
        self.parameters = VisualOdometryParameters()

        self.mask_params = [0.5, 0.7, 0.4]
        self.stingray_mask = []



        self.histogram_img = None
        self.mask_img = None

    def set_parameter(self, param_name, param_val, string_param_val):
        """
        Sets an inner configuration parameter

        :param param_name: name of the parameter as it appears in VisualOdometryParameters
        :type param_name: str
        :param param_val: value of the parameter
        :type param_val: int/float/str/array...
        :param string_param_val: string version of the value of the parameter
        :type string_param_val: str
        """

        try:
            exec ("self.parameters." + param_name + "=" + string_param_val)
            if param_name == 'feature_extractor':
                self.initialize_extractor(param_val)
            elif param_name == 'shrink_x_ratio':
                self.image_w *= param_val
            elif param_name == 'shrink_y_ratio':
                self.image_h *= param_val
        except Exception as e:
            raise NameError("Couldn't set parameter \'" + param_name + "\' with value: " + string_param_val, e)

    def initialize_extractor(self, extractor_type):
        """
        Initializes an openCV feature extractor

        :param extractor_type: type of feature extractor ('SIFT', 'SURF', or 'ORB' expected)
        :type extractor_type: str
        """

        if extractor_type == 'SURF':
            self.cv2_detector = cv2.xfeatures2d.SURF_create()
        elif extractor_type == 'ORB':
            self.cv2_detector = cv2.ORB_create(nfeatures=80)
        else:
            self.cv2_detector = cv2.xfeatures2d.SIFT_create()
        print("Feature extractor initialized")

    def get_camera_info(self, camera_info):
        """
        Save intrinsic camera configuration matrices

        :param camera_info: Information message on the camera
        :type camera_info: sensor_msgs.CameraInfo
        """

        self.camera_K = np.resize(camera_info.K, [3, 3])
        self.camera_D = np.asarray(camera_info.D)

        # image_h and image_w contain the shrink factors at this point. Multiply them by the image dimensions
        self.image_h = self.image_h * camera_info.height
        self.image_w = self.image_w * camera_info.width

        # Generate the depth filtering mask
        self.stingray_mask = create_geometric_mask(self.image_h, self.image_w, self.mask_params)

    def get_duckiebot_velocity(self, joy_command):
        """
        Saves joystick command to private variables

        :param joy_command: joystick command from duckiebot
        :type joy_command: duckietown_msgs.Twist2DStamped
        """

        self.joy_command = joy_command

    def get_image_and_trigger_vo(self, image):
        """
        Reads a new image, extracts its features, and flips the image vector to place it in the first position

        :param image: openCV image for visual odometry pipeline
        :type image: openCV mat

        :return: The estimated transformation between the current image and the one from last call. Returns 'None' if
        there is no previous image
        :rtype: geometry_msgs.TransformStamped
        """

        self.images[1].load_image(image, gray_scale=True)
        self.extract_image_features(self.images[1])
        self.images = self.images[::-1]

        if self.images[1].height > 0:
            return self.visual_odometry_core()

        self.last_ros_time = time.time()
        return None

    def extract_image_features(self, image):
        """
        Extracts pairs of descriptors and keypoints from an image

        :param image: image manager object containing the image whose features should be extracted
        :type image: ImageManager
        """

        parameters = self.parameters

        # Down-sample image
        start = time.time()
        image.downsample(parameters.shrink_x_ratio, parameters.shrink_y_ratio)
        end = time.time()
        print("TIME: Down-sample image. Elapsed time: %s", end - start)

        # Find the key points and descriptors for train image
        start = time.time()
        image.find_keypoints(self.cv2_detector)
        print("Number or keypoints: %s", len(image.keypoints))
        end = time.time()
        print("TIME: Extract features of image. Elapsed time: %s", end - start)

    def visual_odometry_core(self):
        """
        Runs pose estimation using a visual odometry pipeline assuming that images are obtained from a monocular camera
        set on a duckiebot wondering around duckietown

        :return: The estimated transformation between the current image and the one from last call.
        :rtype: geometry_msgs.TransformStamped
        """

        parameters = self.parameters
        train_image = self.images[1]
        query_image = self.images[0]

        # Initialize transformation between camera frames
        t = Transform()

        ############################################
        #                MAIN BODY                 #
        ############################################

        processed_data_plotter = DataPlotter(train_image, query_image, parameters)

        # Instantiate histogram logic filter
        histogram_filter = HistogramLogicFilter(train_image.width, train_image.height)

        start = time.time()
        if parameters.matcher == 'KNN':
            # K-Nearest Neighbors matcher
            bf = cv2.BFMatcher()
            matches = bf.knnMatch(train_image.descriptors, query_image.descriptors, k=parameters.knn_neighbors)
        else:
            # Brute force matcher
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(train_image.descriptors, query_image.descriptors)
        end = time.time()
        print("TIME: Matching done. Elapsed time: %s", end - start)

        # Initialize fitness trackers
        fitness = float('-inf')
        max_fit = fitness

        # Explore all the weight values
        for weight in parameters.knn_weight:

            # Filter knn matches by best to second best match ratio
            if parameters.matcher == 'KNN':
                start = time.time()
                matches = knn_match_filter(matches, weight)
                end = time.time()
                print("TIME: Distance filtering of matches done. Elapsed time: %s", end - start)

            # Filter histograms by gaussian function fitting
            if parameters.filter_by_histogram:
                start = time.time()
                histogram_filter.filter_matches_by_histogram_fitting(
                    matches, train_image.keypoints, query_image.keypoints, parameters.threshold_angle,
                    parameters.threshold_length)
                end = time.time()
                print("TIME: Histogram filtering done. Elapsed time: %s", end - start)

                fitness = histogram_filter.angle_fitness + histogram_filter.length_fitness

                # Store current configuration as best configuration if fitness is new maximum
                if fitness > max_fit:
                    histogram_filter.save_configuration()
                    max_fit = fitness

        # Recover best configuration from histogram filtering (for best weight)
        if parameters.filter_by_histogram and histogram_filter.saved_configuration is not None:
            unfiltered_matches = matches
            matches = histogram_filter.saved_configuration.filter_data_by_histogram()

            # Publish the results of histogram filtering
            if parameters.plot_histogram_filtering:
                self.histogram_img = processed_data_plotter.plot_histogram_filtering(unfiltered_matches, matches, histogram_filter)

        n_final_matches = len(matches)

        # Lists of final filtered matches
        matched_train_points = [None] * n_final_matches
        matched_query_points = [None] * n_final_matches
        for match_index, match_object in enumerate(matches):
            matched_train_points[match_index] = train_image.keypoints[match_object.queryIdx].pt
            matched_query_points[match_index] = query_image.keypoints[match_object.trainIdx].pt

        try:
            [h, w] = [query_image.height, query_image.width]

            start = time.time()
            # Split between far-region and close region matches
            match_distance_filter = \
                DistanceFilter(matched_query_points, matched_train_points, self.camera_K,
                               self.camera_D, (h, w), (parameters.shrink_x_ratio, parameters.shrink_y_ratio))
            match_distance_filter.split_by_distance_mask(self.stingray_mask)

            end = time.time()
            print("TIME: Mask filtering done. Elapsed time: %s", end - start)

            if parameters.plot_masking:
                self.mask_img = processed_data_plotter.plot_displacements_from_distance_mask(match_distance_filter)

            start = time.time()
            n_distant_matches = len(match_distance_filter.rectified_distant_query_points)
            if n_distant_matches > 0:
                rot_hypothesis = []

                # Average sign of rotation
                rot_sign = np.sign(np.average(np.array(matched_query_points) - np.array(matched_train_points), axis=0)[0])

                # Calculate two rotation hypothesis for all far matches assuming that they lay distant to camera
                for distant_q_p, distant_t_p in \
                        zip(match_distance_filter.rectified_distant_query_points,
                            match_distance_filter.rectified_distant_train_points):

                    a = (distant_t_p[0] - distant_q_p[0]) \
                        / np.sqrt((distant_t_p[0]*distant_q_p[0])**2 + distant_t_p[0]**2 + distant_q_p[0]**2 + 1)
                    rot = np.arcsin(a)

                    # Use hypothesis whose sign is consistent with the average sign
                    for rot_h in np.unique(rot):
                        if np.sign(rot_h) == rot_sign:
                            rot_hypothesis.append(-rot_h)
                        else:
                            rot_hypothesis.append(rot_h)

                rot_hypothesis = np.unique(rot_hypothesis)
                rot_hypothesis_rmse = np.zeros((len(rot_hypothesis), 1))

                # Select the best hypothesis using 1 point RANSAC
                for hypothesis_index in range(0, len(rot_hypothesis)):
                    hypothesis = rot_hypothesis[hypothesis_index]
                    rot_mat = np.array([[np.cos(hypothesis), 0, np.sin(hypothesis)],
                                        [0, 1, 0],
                                        [-np.sin(hypothesis), 0, np.cos(hypothesis)]])

                    rotated_train_points = np.matmul(
                        np.append(np.reshape(match_distance_filter.rectified_distant_train_points, (n_distant_matches, 2)),
                                  np.ones((n_distant_matches, 1)), axis=1),
                        rot_mat)

                    # Calculate rmse of hypothesis with all peripheral points
                    error = rotated_train_points[:, 0:2] - np.reshape(
                        match_distance_filter.rectified_distant_query_points, (n_distant_matches, 2))

                    rot_hypothesis_rmse[hypothesis_index] = np.sum(np.sqrt(np.sum(np.power(error, 2), axis=1)))

                theta = rot_hypothesis[np.argmin(rot_hypothesis_rmse)]
                self.last_theta = theta

            else:
                # If there were not enough matches to estimate a new theta, use the one from previous iteration
                theta = self.last_theta

            z_rot_mat = np.array([[np.cos(theta), np.sin(theta), 0], [-np.sin(theta), np.cos(theta), 0], [0, 0, 1]])

            t_vec = [self.joy_command.v / 30.0, 0, 0]

            # Calculate quaternion of z-mapped rotation
            [roll, pitch, yaw] = rotation_matrix_to_euler_angles(z_rot_mat)
            z_quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

            end = time.time()
            print("TIME: Pose estimation done. Elapsed time: %s", end - start)

            t.translation = Vector3(t_vec[0], t_vec[1], t_vec[2])
            t.rotation = Quaternion(z_quaternion[0], z_quaternion[1], z_quaternion[2], z_quaternion[3])

        except Exception:
            raise

        return t, self.histogram_img, self.mask_img


class VisualOdometryParameters:
    def __init__(self):
        self.filter_by_histogram = False
        self.threshold_angle = 10
        self.threshold_length = 10

        self.shrink_x_ratio = 1.0
        self.shrink_y_ratio = 1.0

        self.plot_masking = False
        self.plot_histogram_filtering = False
        self.plot_ransac = False

        self.feature_extractor = 'ORB'

        self.matcher = 'BF'
        self.knn_neighbors = 2
        self.knn_weight = [1.5]
