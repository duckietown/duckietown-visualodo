import numpy as np
from scipy import stats


class MatchData:
    """
    Class for storing various HistogramManager objects and associated data
    """

    def __init__(self, match_vec, best_angle_histogram, best_length_histogram):
        """
        :param match_vec: array of n features of that were found a match
        :type match_vec: ndarray (nx1)

        :param best_angle_histogram: histogram object for the angle distribution
        :type best_angle_histogram: HistogramManager

        :param best_length_histogram: histogram object for the length distribution
        :type best_length_histogram: HistogramManager
        """

        self.good = match_vec
        self.angle_histogram = best_angle_histogram
        self.length_histogram = best_length_histogram

    def get_data(self):
        return [self.angle_histogram, self.length_histogram, self.good]

    def filter_data_by_histogram(self):
        """
        Keep the matches that fulfill the gaussian distribution fitting of both histograms

        :return: the list of filtered match pairs
        :rtype: ndarray (nx2)
        """

        # Identify which matches are outliers in either of the two distributions
        m_matrix = ((self.length_histogram.outliers == True) | (self.angle_histogram.outliers == True))
        n_final_matches = np.count_nonzero(m_matrix == False)

        # Initialize vector with filtered matches
        filtered_match = [None] * n_final_matches

        # Proceed to filter out outliers
        i = 0
        for match_index, match_object in enumerate(self.good):

            if self.angle_histogram.outliers[match_index] == False and \
               self.length_histogram.outliers[match_index] == False:

                filtered_match[i] = match_object
                i += 1

        return filtered_match


class HistogramManager:
    """
    Class for generated histogram analysis from data and extra histogram-associated utilities
    """

    def __init__(self, data_vector, n_bins):
        """
        :param data_vector: Array of data for histogram analysis
        :param n_bins: number of desired bins for the histogram
        """

        self.data = data_vector
        self.n_bins = n_bins
        self.histogram, self.bins = np.histogram(data_vector, n_bins)
        self.bin_centres = (self.bins[:-1] + self.bins[1:]) / 2

        self.fitted_gauss_coefficients = []
        self.outliers = []
        self.area_under_curve = 0
        self.fano_factor = 0
        self.success = False

    def fit_gaussian(self, threshold):
        """
        Fits a gaussian function to the data distribution and removes the elements that are beyond the threshold

        :param threshold: Elimination threshold in terms of standard deviations. Elements outside threshold * standard
        deviation will be removed in both sides of the distribution
        :type threshold: float
        """

        # Initial guess for the fitting coefficients (A, mu and sigma)
        gauss_fit = [np.max(self.histogram), np.mean(self.data), np.std(self.data)]

        try:
            # Fit gaussian distribution to data
            m, s = stats.norm.fit(self.data)

            self.outliers = abs(self.data - m) > threshold * s
            self.fitted_gauss_coefficients = [max(self.histogram), m, s]

        except Exception as fit_exception:
            print("Curve could not be fitted to angle distribution: ", fit_exception)
            self.outliers = False * self.data

        # Area under the curve of the Angle distribution (to be maximized)
        self.area_under_curve = np.count_nonzero(
            (abs(self.bins[0:-1] - gauss_fit[1]) < threshold * gauss_fit[2]) == True)

        # Intra-class fano-factor (to be minimized)
        aux = np.trim_zeros((abs(self.bins - gauss_fit[1]) < threshold * gauss_fit[2]) * self.bins) - gauss_fit[1]
        self.fano_factor = sum(aux ** 2 / (len(aux) * gauss_fit[1]))
        self.success = True

