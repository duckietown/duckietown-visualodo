#!/usr/bin/env python

from __future__ import division

import time
import cv2
import numpy as np
import rospy
import tf2_ros as tf2
import tf.transformations

from math import isnan
from cv_bridge import CvBridge

from data_plotter import DataPlotter
from match_geometric_filters import HistogramLogicFilter, DistanceFilter
from utils import knn_match_filter, rotation_matrix_to_euler_angles, qv_multiply, create_circular_mask
from image_manager import ImageManager

from geometry_msgs.msg import Vector3, Quaternion, Pose, TransformStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path

from duckietown_msgs.msg import Twist2DStamped


class VisualOdometry:
    def __init__(self, parameters):
        self.parameters = parameters
        self.images = np.array([ImageManager(), ImageManager()])
        self.bridge = CvBridge()

        # Intrinsic camera calibration matrix
        self.camera_K = None

        # Ros stuff
        self.path_publisher = rospy.Publisher("path", Path, queue_size=2)
        self.odom_publisher = rospy.Publisher("odometry", Odometry, queue_size=2)
        self.transform_broadcaster = tf2.TransformBroadcaster()

        # Transformation from world to duckiebot frame
        self.path = Path()
        self.stacked_position = Vector3(0, 0, 0)
        self.stacked_rotation = tf.transformations.quaternion_from_euler(0, 0, 0)

        # Current command sent to duckiebot
        self.joy_command = Twist2DStamped()

        self.last_ros_time = rospy.get_time()

        # Initiate the feature detector
        if parameters.feature_extractor == 'SURF':
            self.cv2_detector = cv2.xfeatures2d.SURF_create()
        elif parameters.feature_extractor == 'ORB':
            self.cv2_detector = cv2.ORB_create(nfeatures=80)
        else:
            self.cv2_detector = cv2.xfeatures2d.SIFT_create()

        # aux_image_manager = ImageManager()
        # aux_image_manager.read_image(
        #     '/home/guillem/Documents/feature_alignment/catkin_ws/src/image_provider/Images/IMG_0568.JPG')
        # image1 = self.bridge.cv2_to_compressed_imgmsg(aux_image_manager.image)
        # self.save_image_and_trigger_vo(image1)
        # aux_image_manager.read_image(
        #     '/home/guillem/Documents/feature_alignment/catkin_ws/src/image_provider/Images/IMG_0570.JPG')
        # image2 = self.bridge.cv2_to_compressed_imgmsg(aux_image_manager.image)
        # self.save_image_and_trigger_vo(image2)

    def save_command(self, data):
        self.joy_command = data

    def save_camera_calibration(self, data):
        self.camera_K = np.resize(data.K, [3, 3])

    def save_image_and_trigger_vo(self, data):
        start = time.time()
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data)

        # Read new image, extract features, and flip vector to place it in the first position
        self.images[1].load_image(cv_image, gray_scale=True)
        self.extract_image_features(self.images[1])
        self.images = np.flip(self.images)

        if self.images[1].height > 0:
            self.visual_odometry_core()

        self.last_ros_time = rospy.get_time()

        rospy.logwarn("TIME: Total time: %s", time.time() - start)
        rospy.logwarn("===================================================")

    def extract_image_features(self, image):
        parameters = self.parameters

        # Down-sample image
        start = time.time()
        image.downsample(parameters.shrink_x_ratio, parameters.shrink_y_ratio)
        end = time.time()
        rospy.logwarn("TIME: Down-sample image. Elapsed time: %s", end - start)

        # Find the key points and descriptors for train image
        start = time.time()
        image.find_keypoints(self.cv2_detector)
        rospy.logwarn("Number or keypoints: %s", len(image.keypoints))
        end = time.time()
        rospy.logwarn("TIME: Extract features of image. Elapsed time: %s", end - start)

    def visual_odometry_core(self):

        parameters = self.parameters
        train_image = self.images[1]
        query_image = self.images[0]

        ############################################
        #                MAIN BODY                 #
        ############################################

        processed_data_plotter = DataPlotter(train_image, query_image)

        # Instantiate histogram logic filter
        histogram_filter = HistogramLogicFilter()

        start = time.time()
        if parameters.matcher == 'KNN':
            bf = cv2.BFMatcher()
            matches = bf.knnMatch(train_image.descriptors, query_image.descriptors, k=parameters.knn_neighbors)
        else:
            # Brute force matcher
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(train_image.descriptors, query_image.descriptors)
        end = time.time()
        rospy.logwarn("TIME: Matching done. Elapsed time: %s", end - start)

        # Initialize fitness trackers
        fitness = float('-inf')
        max_fit = fitness
        max_weight = parameters.knn_weight[0]

        # Explore all the weight values
        for weight in parameters.knn_weight:

            # Filter knn matches by best to second best match ratio
            if parameters.matcher == 'KNN':
                start = time.time()
                matches = knn_match_filter(matches, weight)
                end = time.time()
                rospy.logwarn("TIME: Distance filtering of matches done. Elapsed time: %s", end - start)

            # Filter histograms by gaussian function fitting
            if parameters.filter_by_histogram:
                start = time.time()
                histogram_filter.fit_gaussian(matches, train_image.keypoints, query_image.keypoints,
                                              parameters.angle_th, parameters.length_th)
                end = time.time()
                rospy.logwarn("TIME: Histogram filtering done. Elapsed time: %s", end - start)

                fitness = histogram_filter.angle_fitness + histogram_filter.length_fitness

                # Store current configuration as best configuration if fitness is new maximum
                if fitness > max_fit:
                    histogram_filter.save_configuration()
                    max_fit = fitness
                    max_weight = weight

        # Recover best configuration from histogram filtering (for best weight)
        if parameters.filter_by_histogram and histogram_filter.saved_configuration is not None:
            unfiltered_matches = matches
            matches = histogram_filter.saved_configuration.filter_data_by_histogram()

            # Publish the results of histogram filtering
            if parameters.plot_histogram_filtering:
                processed_data_plotter.plot_histogram_filtering(
                    unfiltered_matches, matches, histogram_filter, max_weight, max_fit)

        n_final_matches = len(matches)

        # Initialize final displacement vectors; x and y will contain the initial points and Dx and Dy the
        # corresponding deformations
        x = np.zeros([n_final_matches, 1])
        y = np.zeros([n_final_matches, 1])

        matched_train_points = [None] * n_final_matches
        matched_query_points = [None] * n_final_matches

        # Proceed to calculate deformations and point maps
        for match_index, match_object in enumerate(matches):
            x[match_index] = int(round(train_image.keypoints[match_object.queryIdx].pt[0]))
            y[match_index] = int(round(train_image.keypoints[match_object.queryIdx].pt[1]))

            matched_train_points[match_index] = train_image.keypoints[match_object.queryIdx].pt
            matched_query_points[match_index] = query_image.keypoints[match_object.trainIdx].pt

        try:
            [h, w] = [query_image.height, query_image.width]

            # Split between far-region and close region matches
            # TODO: improve mask -> make it a function of intrinsic camera calibration / estimated depth
            proximity_r = ((0.2 * h) ** 2 + (0.5 * w) ** 2) / (0.4 * h)
            proximity_mask = create_circular_mask(h, w, center=(w / 2, 0.3 * h + proximity_r), radius=proximity_r)

            match_distance_filter = DistanceFilter(matched_query_points, matched_train_points, self.camera_K, (h, w))
            match_distance_filter.split_by_distance(proximity_mask)

            # Remove me
            if parameters.plot_matches:
                processed_data_plotter.plot_point_correspondences(
                    matched_query_points, matched_train_points, proximity_mask)

            n_distant_matches = len(match_distance_filter.distant_query_points)
            rot_hypothesis = []

            # Calculate two rotation hypothesis for all far matches assuming that they lay close to the ground plane
            for distant_q_p, distant_t_p in \
                    zip(match_distance_filter.distant_query_points, match_distance_filter.distant_train_points):

                a = (distant_t_p[0] - distant_q_p[0]) \
                    / np.sqrt((distant_t_p[0]*distant_q_p[0])**2+distant_t_p[0]**2+distant_q_p[0]**2+1)
                rot = np.arccos((distant_t_p[1]+distant_t_p[0]*distant_q_p[1]*np.array([a, -a]))/distant_q_p[1])
                rot = [rot_h for rot_h in rot if not isnan(rot_h)]

                for rot_h in np.unique(rot):
                    rot_hypothesis.append(rot_h)
                    rot_hypothesis.append(-rot_h)

            rot_hypothesis = np.unique(rot_hypothesis)
            rot_hypothesis_rmse = np.zeros((len(rot_hypothesis), 1))

            # Select the best hypothesis using 1 point RANSAC
            for hypothesis_index in range(0, len(rot_hypothesis)):
                hypothesis = rot_hypothesis[hypothesis_index]
                rot_mat = np.array([[np.cos(hypothesis), 0, np.sin(hypothesis)],
                                    [0, 1, 0],
                                    [-np.sin(hypothesis), 0, np.cos(hypothesis)]])

                rotated_train_points = np.matmul(
                    np.append(np.reshape(match_distance_filter.distant_train_points, (n_distant_matches, 2)),
                              np.ones((n_distant_matches, 1)), axis=1),
                    rot_mat)

                # Calculate rmse of hypothesis with all peripheral points
                error = rotated_train_points[:, 0:2] - np.reshape(
                    match_distance_filter.distant_query_points, (n_distant_matches, 2))

                rot_hypothesis_rmse[hypothesis_index] = np.sum(np.sqrt(np.sum(np.power(error, 2), axis=1)))

            # TODO: make the scaling adjustable from the features
            theta = rot_hypothesis[np.argmin(rot_hypothesis_rmse)]
            y_rot_mat = np.array([[np.cos(theta), 0, np.sin(theta)],
                                  [0, 1, 0],
                                  [-np.sin(theta), 0, np.cos(theta)]])
            z_rot_mat = np.array([[np.cos(theta), np.sin(theta), 0],
                                  [-np.sin(theta), np.cos(theta), 0],
                                  [0, 0, 1]])

            t_vec = [self.joy_command.v / 30.0, 0, 0]

            n_proximal_matches = n_final_matches - n_distant_matches
            t_hypothesis = []

            # Estimate translation vector
            for proximal_q_p, proximal_t_p in \
                    zip(match_distance_filter.close_query_points, match_distance_filter.close_train_points):
                proximal_q_p = np.matmul(np.transpose(z_rot_mat), np.append([proximal_q_p], [1]))

                ty = (proximal_t_p[1] - proximal_q_p[1]) * self.joy_command.v
                tx = (proximal_t_p[0] - proximal_q_p[0]) * self.joy_command.v

                t_hypothesis.append(np.matmul(np.transpose(y_rot_mat), [tx, ty, 0]))

            t_hypothesis = np.unique(t_hypothesis, axis=0)
            t_hypothesis_rmse = np.zeros((len(t_hypothesis), 1))

            # Select the best hypothesis using 1 point RANSAC
            for hypothesis_index in range(0, len(t_hypothesis)):
                t_vec_hyp = t_hypothesis[hypothesis_index]

                h_matrix = np.append(y_rot_mat, np.expand_dims(t_vec_hyp.T, axis=1), axis=1)
                h_matrix = np.append(h_matrix, np.expand_dims(np.array([0, 0, 0, 1]).T, axis=0), axis=0)

                transformed_train_points = np.matmul(
                    np.append(np.reshape(match_distance_filter.close_train_points, (n_proximal_matches, 2)),
                              np.ones((n_proximal_matches, 2)), axis=1),
                    h_matrix)

                # Calculate rmse of hypothesis with all peripheral points
                error = transformed_train_points[:, 0:2] - np.reshape(
                    match_distance_filter.close_query_points, (n_proximal_matches, 2))
                t_hypothesis_rmse[hypothesis_index] = np.sum(np.sqrt(np.sum(np.power(error, 2), axis=1)))

            t_vec = t_hypothesis[np.argmin(t_hypothesis_rmse)]

            """
            # Extract essential matrix
            start = time.time()
            [h_matrix, mask] = cv2.findEssentialMat(np.array(matched_train_points, dtype=float),
                                                    np.array(matched_query_points, dtype=float), self.camera_K,
                                                    method=cv2.RANSAC, prob=0.999, threshold=1.0)

            # Remove RANSAC outliers
            matched_query_points = np.reshape(
                np.array(matched_query_points, dtype=float)[mask * np.ones([1, 2]) == 1], [-1, 2])
            matched_train_points = np.reshape(
                np.array(matched_train_points, dtype=float)[mask * np.ones([1, 2]) == 1], [-1, 2])

            rot_sign = np.sign(np.average(matched_query_points - matched_train_points, axis=0)[0])

            # Stream points used for essential matrix calculation for debugging
            if parameters.plot_matches:
                processed_data_plotter.plot_point_correspondances(
                    distant_query_matches, distant_train_matches, proximity_mask)
                    #matched_train_points, matched_query_points, proximity_mask)

            # If no displacement has occurred in any of the points, assume that rotation and translation were 0
            if not any((matched_train_points - matched_query_points != np.zeros(matched_train_points.shape)).ravel()):
                rot_mat = np.eye(3)
                t_vec = np.array([0, 0, 0], dtype=float)
                rot_mag = 0

            else:
                # Recover rotation and translation from essential matrix
                [_, rot_mat, t_vec, _] = \
                    cv2.recoverPose(h_matrix, matched_train_points, matched_query_points, self.camera_K)

                rot_mag = ((np.trace(rot_mat) - 1) / 2)

                if rot_mag < 0:
                    rot_mag = np.arccos(max([rot_mag, -1.0]))
                else:
                    rot_mag = np.arccos(min([rot_mag, 1.0]))

                # If rotation modulus is superior than 45 degrees, suppress it
                if rot_mag > np.pi/8:
                    rot_mat = np.eye(3)
                    t_vec = np.array([0, 0, 1], dtype=float)
                    rot_mag = 0

            rot_mag *= rot_sign

            # Remap the rotation onto the Z plane
            z_rot_mat = np.array([[np.cos(rot_mag), -np.sin(rot_mag), 0],
                                 [np.sin(rot_mag), np.cos(rot_mag), 0],
                                 [0, 0, 1]])

            # Calculate euler rotation from matrix, and quaternion from euler rotation
            [roll, pitch, yaw] = rotation_matrix_to_euler_angles(rot_mat)
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            """

            # Calculate quaternion of z-mapped rotation
            [roll, pitch, yaw] = rotation_matrix_to_euler_angles(z_rot_mat)
            z_quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

            # t_q = (z_quaternion * np.multiply(quaternion, [-1, -1, -1, 1]))
            # t_q = t_q / (np.sqrt(t_q[0]**2 + t_q[1]**2 + t_q[2]**2 + t_q[3]**2))
            # t_vec = np.multiply(qv_multiply(t_q, t_vec), np.array([1, 0, 0], dtype=float))

            current_time = rospy.Time.now()

            # If velocity command is 0, set displacement to 0
            if abs(self.joy_command.v) < 0.01:
                t_vec = np.array([0.0, 0.0, 0.0], dtype=float)

            # t_vec = np.multiply(np.squeeze(t_vec), np.array([1, 0, 0]))
            t = TransformStamped()
            t.header.frame_id = "world"
            t.child_frame_id = "axis"
            t.header.stamp = current_time

            # Rotate displacement vector by duckiebot rotation wrt world frame and add it to stacked displacement
            t_vec = np.squeeze(qv_multiply(self.stacked_rotation, t_vec))

            translation = Vector3(t_vec[0], t_vec[1], t_vec[2])
            self.stacked_position = Vector3(
                self.stacked_position.x + translation.x,
                self.stacked_position.y + translation.y,
                self.stacked_position.z + translation.z)

            # Add quaternion rotation to stacked rotation to obtain the rotation transformation between world and
            # duckiebot frames
            quaternion = tf.transformations.quaternion_multiply(self.stacked_rotation, z_quaternion)

            # Store quaternion and transform it to geometry message
            self.stacked_rotation = quaternion
            quaternion = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

            # Broadcast transform
            t.transform.translation = self.stacked_position
            t.transform.rotation = quaternion
            self.transform_broadcaster.sendTransform(t)

            # Create odometry and Path msgs
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "world"

            self.path.header = odom.header

            # set the position
            odom.pose.pose = Pose(self.stacked_position, quaternion)
            pose_stamped = PoseStamped()
            pose_stamped.header = odom.header
            pose_stamped.pose = odom.pose.pose
            self.path.poses.append(pose_stamped)

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(self.joy_command.v, 0, 0), Vector3(0, 0, self.joy_command.omega))

            # publish the messages
            self.odom_publisher.publish(odom)
            self.path_publisher.publish(self.path)

            end = time.time()
            rospy.logwarn("TIME: RANSAC homography done. Elapsed time: %s", end - start)

        except AssertionError as e:
            rospy.logerr(e)
            raise
        except Exception as e:
            rospy.logerr(e)
            rospy.logwarn("Not enough matches for RANSAC homography")
            raise
