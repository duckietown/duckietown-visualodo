import numpy as np
import cv2

from histogram_manager import HistogramManager, MatchData
from utils import rectifier


class HistogramLogicFilter:

    def __init__(self):
        self.angle_fitness = 0
        self.length_fitness = 0
        self.angle_histogram = None
        self.length_histogram = None
        self.saved_configuration = None

        self.match_vector = []
        self.angle_threshold = None
        self.length_threshold = None

    def fit_gaussian(self, match_vector, kp1, kp2, angle_threshold, length_threshold):
        """
        Fits a gaussian function to the length and angle distributions of keypoint matches between two images. Saves the
        results in a HistogramManager class type

        :param match_vector: array of match pairs (point in image 1 vs point in image 2)
        :type match_vector: ndarray (nx2)
        :param kp1: array of 2D keypoints of train image
        :type kp1: ndarray(nx2)
        :param kp2: array of 2D keypoints of query image
        :type kp2: ndarray(nx2)
        :param angle_threshold: threshold (in terms of standard deviations) used to determine an inlier of the angle
        distribution
        :type angle_threshold: (positive) float
        :param length_threshold: threshold (in terms of standard deviations) used to determine an inlier of the length
        distribution
        :type length_threshold: (positive) float
        """

        self.match_vector = match_vector
        self.angle_threshold = angle_threshold
        self.length_threshold = length_threshold

        # Initialize and fill angle and length arrays of the vectors of the matched points
        angle_vec = np.zeros([len(match_vector), 1])
        length_vec = np.zeros([len(match_vector), 1])

        for i, match_object in enumerate(match_vector):
            dist = [a_i - b_i for a_i, b_i in zip(kp2[match_object.trainIdx].pt, kp1[match_object.queryIdx].pt)]
            angle_vec[i] = np.arctan(dist[1] / max(dist[0], 0.01))
            length_vec[i] = np.sqrt(dist[0] ** 2 + dist[1] ** 2)

        # Compute histograms of the two distributions (angle and length of the displacement vectors)
        self.angle_histogram = HistogramManager(angle_vec, 8)
        self.length_histogram = HistogramManager(length_vec, 8)

        # Fit gaussian functions to distribution and remove outliers
        self.angle_histogram.fit_gaussian(angle_threshold)
        self.length_histogram.fit_gaussian(length_threshold)

        # Calculate fitness
        if self.angle_histogram.success and self.length_histogram.success:
            self.angle_fitness = self.angle_histogram.area_under_curve / rectifier(self.angle_histogram.fano_factor)
            self.length_fitness = self.length_histogram.area_under_curve / rectifier(self.length_histogram.fano_factor)

            # print("Fano factors: angle = ", self.angle_histogram.fano_factor, "  length = ",
            #       self.length_histogram.fano_factor)
            # print("Total fitness: ", self.angle_fitness + self.length_fitness, " Split attributes: angle = ",
            #       self.angle_fitness, "  length = ", self.length_fitness)

    def save_configuration(self):
        self.saved_configuration = MatchData(self.match_vector, self.angle_histogram, self.length_histogram)


class RansacFilter:
    @staticmethod
    def ransac_homography(kp1, kp2, match_vector, data_plotter, plot_data):

        src_pts = np.float32([kp1[m.queryIdx].pt for m in match_vector]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in match_vector]).reshape(-1, 1, 2)

        h_matrix, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matches_mask = mask.ravel().tolist()

        if data_plotter is not None and plot_data:
            data_plotter.plot_ransac_homography(match_vector, h_matrix, matches_mask)

        return h_matrix


class DistanceFilter:
    def __init__(self, query_points, train_points, camera_K, image_shape):
        """
        Class for processing matches based on their position with respect to the camera. Points are assumed to match
        in the same order than they appear in their vectors

        :param query_points: array of 2D keypoints of train image
        :type query_points: ndarray(nx2)
        :param train_points: array of 2D keypoints of query image
        :type train_points: ndarray(nx2)
        :param camera_K: camera intrinsic calibration matrix
        :type camera_K: ndarray(3x3)
        :param image_shape: shape of images (height, width)
        :type image_shape: tuple(2,1)
        """
        self.query_points = query_points
        self.train_points = train_points
        self.camera_K = camera_K
        self.image_shape = image_shape

        self.norm_close_query_points = []
        self.norm_close_train_points = []
        self.norm_distant_query_points = []
        self.norm_distant_train_points = []

        self.close_query_points = []
        self.close_train_points = []
        self.distant_query_points = []
        self.distant_train_points = []

        self.proximity_mask = []

    def split_by_distance(self, proximity_mask):
        """
        Separates the matches in two categories depending on whether they lie inside the provided mask or not

        :param proximity_mask: Boolean mask identifying the close region to the camera view. Must have same size as
        original images
        :type proximity_mask: Boolean ndarray
        """

        assert proximity_mask.shape[0:2] == self.image_shape

        self.proximity_mask = proximity_mask

        # Get far/close-region matches
        for query_point, train_point in zip(self.query_points, self.train_points):

            # Transform points to homogeneous coordinate system
            query_point = np.append([query_point], [1])
            train_point = np.append([train_point], [1])

            if not proximity_mask[int(query_point[1]), int(query_point[0])] and \
                    not proximity_mask[int(train_point[1]), int(train_point[0])]:
                # self.distant_query_points.append(np.matmul(np.linalg.inv(self.camera_K), query_point)[0:2])
                # self.distant_train_points.append(np.matmul(np.linalg.inv(self.camera_K), train_point)[0:2])
                self.distant_query_points.append(query_point[0:2])
                self.distant_train_points.append(train_point[0:2])

            else:
                self.close_query_points.append(query_point[0:2])
                self.close_train_points.append(train_point[0:2])

                # proximal_query_matches.append(np.matmul(np.linalg.inv(self.camera_K), query_point)[0:2])
                # proximal_train_matches.append(np.matmul(np.linalg.inv(self.camera_K), train_point)[0:2])

        normalization_vec = [[1 / self.image_shape[1], 0], [0, 1 / self.image_shape[0]]]

        self.norm_distant_query_points = np.matmul(self.distant_query_points, normalization_vec)
        self.norm_distant_train_points = np.matmul(self.distant_train_points, normalization_vec)
        self.norm_close_query_points = np.matmul(self.close_query_points, normalization_vec)
        self.norm_close_train_points = np.matmul(self.close_train_points, normalization_vec)
