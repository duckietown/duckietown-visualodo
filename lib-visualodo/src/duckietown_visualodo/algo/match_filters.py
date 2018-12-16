import numpy as np
import cv2

from histogram_manager import HistogramManager, MatchData
from utils import rectifier, camera_inverse_projection


class HistogramLogicFilter:

    def __init__(self, w, h):
        """
        Initializes the histogram filter

        :param w: width of the input images
        :type w: float
        :param h: height of the input images
        :type h: float
        """
        self.angle_fitness = 0
        self.length_fitness = 0
        self.angle_histogram = None
        self.length_histogram = None
        self.saved_configuration = None

        self.image_w = w
        self.image_h = h

        self.match_vector = []
        self.angle_threshold = None
        self.length_threshold = None

    def filter_matches_by_histogram_fitting(self, match_vector, kp1, kp2, angle_threshold, length_threshold):
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
            angle_vec[i] = np.arctan(dist[1] / (dist[0] - self.image_w))
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
    def __init__(self, query_points, train_points, camera_k, camera_d, image_shape, shrink_factors):
        """
        Class for processing matches based on their position with respect to the camera. Points are assumed to match
        in the same order than they appear in their vectors

        :param query_points: array of 2D key-points of train image
        :type query_points: ndarray(nx2)
        :param train_points: array of 2D key-points of query image
        :type train_points: ndarray(nx2)
        :param camera_k: camera intrinsic calibration matrix
        :type camera_k: ndarray(3x3)
        :param camera_d: camera deformation matrix
        :type camera_d: ndarray(4x1) or (5x1)
        :param image_shape: shape of images (height, width)
        :type image_shape: tuple(2,1)
        :param shrink_factors: ratio of down-sampling, in the range (0,1]
        :type shrink_factors: tuple (2,1)
        """

        self.query_points = query_points
        self.train_points = train_points
        self.camera_K = camera_k
        self.camera_D = camera_d
        self.image_shape = image_shape
        self.shrink_factors = shrink_factors

        self.rectified_close_query_points = []
        self.rectified_close_train_points = []
        self.rectified_distant_query_points = []
        self.rectified_distant_train_points = []

        self.close_query_points = []
        self.close_train_points = []
        self.distant_query_points = []
        self.distant_train_points = []

        self.proximity_mask = []

    def split_by_distance_mask(self, proximity_mask):
        """
        Separates the matches in two categories depending on whether they lie inside the provided mask or not

        :param proximity_mask: Boolean mask identifying the close region to the camera view. Must have same size as
        original images
        :type proximity_mask: Boolean ndarray
        """

        assert proximity_mask.shape[0:2] == self.image_shape

        self.proximity_mask = proximity_mask

        selected_distant_points = np.zeros(len(self.query_points))

        # Get far/close-region matches
        for i in range(0, len(self.query_points)):
            query_point = self.query_points[i]
            train_point = self.train_points[i]

            if not proximity_mask[int(query_point[1]), int(query_point[0])] and \
                    not proximity_mask[int(train_point[1]), int(train_point[0])]:
                selected_distant_points[i] = 1

        # Split image points according to mask
        self.distant_query_points = np.asarray(self.query_points)[selected_distant_points == 1]
        self.distant_train_points = np.asarray(self.train_points)[selected_distant_points == 1]
        self.close_query_points = np.asarray(self.query_points)[selected_distant_points == 0]
        self.close_train_points = np.asarray(self.train_points)[selected_distant_points == 0]

        # Obtain normalized homogeneous pixel coordinates and split points in the same way
        rectified_query_points = \
            camera_inverse_projection(self.query_points, self.camera_K, self.camera_D, self.shrink_factors)
        rectified_train_points = \
            camera_inverse_projection(self.train_points, self.camera_K, self.camera_D, self.shrink_factors)

        self.rectified_distant_query_points = np.asarray(rectified_query_points)[selected_distant_points == 1]
        self.rectified_distant_train_points = np.asarray(rectified_train_points)[selected_distant_points == 1]
        self.rectified_close_query_points = np.asarray(rectified_query_points)[selected_distant_points == 0]
        self.rectified_close_train_points = np.asarray(rectified_train_points)[selected_distant_points == 0]
