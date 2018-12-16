from __future__ import division

import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage

from match_filters import HistogramLogicFilter
from utils import gauss


class DataPlotter:

    def __init__(self, train_image, query_image):
        self.train_image_manager = train_image
        self.query_image_manager = query_image
        self.image_bridge = CvBridge()
        self.ransac_publisher = rospy.Publisher("ransac/image/compressed", CompressedImage, queue_size=2)
        self.histogram_filter_publisher = rospy.Publisher("histograms/image/compressed", CompressedImage, queue_size=2)
        self.match_publisher = rospy.Publisher("masking/image/compressed", CompressedImage, queue_size=2)

    def plot_histogram_filtering(self, good_matches, best_matches, histogram_filter):
        """
        Plots the result of the match histogram filtering

        :param good_matches: array of match pairs (point in image 1 vs point in image 2)
        :type good_matches: ndarray (nx2)
        :param best_matches: array of histogram-filtered match pairs (point in image 1 vs point in image 2)
        :type best_matches: ndarray (nx2)
        :param histogram_filter: histogram filtering object
        :type histogram_filter: HistogramLogicFilter
        """

        img1 = self.train_image_manager.image
        img2 = self.query_image_manager.image
        kp1 = self.train_image_manager.keypoints
        kp2 = self.query_image_manager.keypoints

        angle_hist = histogram_filter.angle_histogram
        length_hist = histogram_filter.length_histogram

        angle_th = histogram_filter.angle_threshold
        length_th = histogram_filter.length_threshold

        initial_matches_img = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, None, flags=2)
        final_matches_img = cv2.drawMatches(img1, kp1, img2, kp2, best_matches, None, flags=2)

        fig = Figure()
        canvas = FigureCanvas(figure=fig)
        ax = fig.gca()
        n_bins = min(int(np.pi/10*len(angle_hist.bin_centres)/np.ptp(angle_hist.bin_centres)), 50)
        histogram_span = [-np.pi/20, np.pi/20]
        gaussian_samples = np.linspace(histogram_span[0], histogram_span[1], n_bins)
        ax.hist(angle_hist.data, bins=n_bins, range=histogram_span, color='b')
        hist_fit_angle = gauss(gaussian_samples, *angle_hist.fitted_gauss_coefficients)
        ax.bar([angle_hist.fitted_gauss_coefficients[1] - angle_th * angle_hist.fitted_gauss_coefficients[2],
                angle_hist.fitted_gauss_coefficients[1]] + angle_th / 2 * angle_hist.fitted_gauss_coefficients[2],
               np.max(angle_hist.histogram), angle_th * angle_hist.fitted_gauss_coefficients[2], alpha=0.4, color='r')
        ax.plot(gaussian_samples, hist_fit_angle, color='g')
        initial_histogram_img = self.render_and_crop_canvas(ax, canvas)

        fig_2 = Figure()
        canvas = FigureCanvas(figure=fig_2)
        ax = fig_2.gca()
        ax.hist(length_hist.data, bins=length_hist.bins, color='b')
        hist_fit_length = gauss(length_hist.bin_centres, *length_hist.fitted_gauss_coefficients)
        ax.bar([length_hist.fitted_gauss_coefficients[1] - length_th * length_hist.fitted_gauss_coefficients[2],
                length_hist.fitted_gauss_coefficients[1]] + length_th / 2 * length_hist.fitted_gauss_coefficients[2],
               np.max(length_hist.histogram), length_th * length_hist.fitted_gauss_coefficients[2], alpha=0.4,
               color='r')
        ax.plot(length_hist.bin_centres, hist_fit_length, color='g')
        final_histogram_img = self.render_and_crop_canvas(ax, canvas)

        matches_img = np.append(initial_matches_img, final_matches_img, axis=0)
        final_img = self.resize_image_aspect_ratio(
            np.append(initial_histogram_img, final_histogram_img, axis=0), new_height=matches_img.shape[0])
        final_img = np.append(matches_img, final_img, axis=1)
        self.histogram_filter_publisher.publish(self.image_bridge.cv2_to_compressed_imgmsg(final_img))

    def plot_point_correspondences(self, train_pts, query_pts, proximity_mask):
        """
        Plots the point correspondences specified by the two input vectors in train and query images respectively

        :param train_pts:
        :param query_pts:
        :param proximity_mask:
        """
        img1 = self.train_image_manager.image
        img2 = self.query_image_manager.image

        fig = Figure()
        canvas = FigureCanvas(figure=fig)
        appended_pixels = img2.shape[0:2]
        ax = fig.gca()

        ax.imshow(np.append(img1, img2, axis=1))
        ax.imshow(np.append(proximity_mask, proximity_mask, axis=1), alpha=0.25)
        for i in range(0, len(train_pts)):
            try:
                ax.plot([train_pts[i][0], query_pts[i][0] + appended_pixels[1]], [train_pts[i][1], query_pts[i][1]],
                        'o-', markerfacecolor='none')
            except Exception as e:
                rospy.logerr(e)

        img3 = self.render_and_crop_canvas(ax, canvas)

        # plt.figure()
        # plt.imshow(img3)
        # plt.show()

        self.match_publisher.publish(self.image_bridge.cv2_to_compressed_imgmsg(img3))

    def plot_displacements_from_distance_mask(self, match_distance_filter):

        fig = Figure()
        canvas = FigureCanvas(figure=fig)
        ax = fig.gca()
        ax.imshow(self.query_image_manager.image)
        ax.imshow(match_distance_filter.proximity_mask, alpha=0.25)

        for i in range(0, len(match_distance_filter.close_query_points)):
            point_0 = match_distance_filter.close_train_points[i]
            point_t = match_distance_filter.close_query_points[i]

            ax.plot([point_t[0], point_0[0]], [point_t[1], point_0[1]], 'b-', markerfacecolor='none')

        for i in range(0, len(match_distance_filter.distant_query_points)):
            point_0 = match_distance_filter.distant_train_points[i]
            point_t = match_distance_filter.distant_query_points[i]

            ax.plot([point_t[0], point_0[0]], [point_t[1], point_0[1]], 'r-', markerfacecolor='none')

        img = self.render_and_crop_canvas(ax, canvas)

        # plt.figure()
        # plt.imshow(img)
        # plt.show()

        self.match_publisher.publish(self.image_bridge.cv2_to_compressed_imgmsg(img))

    def plot_query_bounding_box(self, bounding_box):
        """
        Plots the position of query image in train image based on the found matches

        :param bounding_box: bounding box of query image in train image
        :type bounding_box: 2x2 array: row 1 = top left corner, row 2 = lower right corner
        """

        bounding_box = bounding_box.astype(int)

        plt.figure(figsize=(20, 16))
        plt.subplot2grid((1, 4), (0, 0), colspan=3)
        img3 = cv2.rectangle(self.train_image_manager.image,
                             tuple(bounding_box[1, :]), tuple(bounding_box[0, :]), 1, thickness=3)
        plt.imshow(img3)
        plt.subplot2grid((1, 4), (0, 3), colspan=1)
        plt.imshow(self.query_image_manager.image)
        plt.show()

    def plot_ransac_homography(self, matches, h_matrix, matches_mask):
        """
        Plots the result of the ransac match filtering

        :param matches: vector of original matches
        :type matches: ndarray (nx1)
        :param h_matrix: homography matrix
        :type h_matrix: array
        :param matches_mask: matches used for final ransac
        :type matches_mask: logic ndarray (nx1)
        """

        rospy.logwarn(h_matrix)

        if h_matrix is not None:

            img1 = cv2.cvtColor(self.train_image_manager.image, cv2.COLOR_BGR2GRAY)
            img2 = cv2.cvtColor(self.query_image_manager.image, cv2.COLOR_BGR2GRAY)
            kp1 = self.train_image_manager.keypoints
            kp2 = self.query_image_manager.keypoints

            h, w = img1.shape
            pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts, h_matrix)

            img2 = cv2.polylines(img2, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)

            draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                               singlePointColor=None,
                               matchesMask=matches_mask,  # draw only inliners
                               flags=2)

            img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches, None, **draw_params)

            self.ransac_publisher.publish(self.image_bridge.cv2_to_compressed_imgmsg(img3))

    @staticmethod
    def render_and_crop_canvas(ax, canvas):
        """
        Render canvas as RGB image, and crop the white part around the canvas axis

        :param ax: figure axis object
        :param canvas: figure canvas object
        :return: the RBG rendered and cropped image
        """

        canvas.draw()
        img = np.fromstring(canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img = img.reshape(canvas.get_width_height()[::-1] + (3,))
        subplot_bb = np.round(ax.bbox.extents)
        img = img[int(subplot_bb[1]+5):int(subplot_bb[3]+5), int(subplot_bb[0]):int(subplot_bb[2])]
        return img

    @staticmethod
    def resize_image_aspect_ratio(image, new_width=None, new_height=None):
        height, width = image.shape[:2]

        if new_width is not None and new_height is None:
            r = new_width / width
            new_height = int(height * r)
        elif new_width is None and new_height is not None:
            r = new_height / height
            new_width = int(width * r)

        new_image = cv2.resize(image, (new_width, new_height))
        return new_image
