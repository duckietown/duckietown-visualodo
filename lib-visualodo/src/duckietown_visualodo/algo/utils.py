import numpy as np
import math
import cv2
import tf.transformations


# Gaussian function model to filter out unwanted matches during SURF registration
def gauss(val, *p):
    """
    1D gaussian function g(x)

    :param val: x input value(s) for the gaussian function
    :param p: parameter vector including [amplitude, mean, variance] describing the function
    :return: the gaussian function described by parameters 'p' for values 'val'
    """

    amp, mu, sigma = p  # Amplitude, mean, variance
    return amp * np.exp(-(val - mu) ** 2 / (2. * sigma ** 2))


# Softplus rectifier to penalize even more disperse distributions
def rectifier(val):
    """
    Softplus-based rectifier function. Its output tends to 0 below 'soft_th', increases rapidly between 'soft_th' and
    'hard_th', and outputs a negative value asymptotically tending to -0.001 above 'hard_th'

       output
         ^
         |  /
         | /
    ======'---,======= -> 'val'
         |   /
         |

    :param val: x input value(s) for the rectifier
    :return: the corresponding values of the rectifier function for the inputs described by values 'val'
    """

    hard_th = 5
    soft_th = 3
    if val < hard_th:
        return np.log(1 + np.exp(val + np.log(np.exp(1) - 1) - soft_th))+0.001
    else:
        return -np.log(1 + np.exp(-(val + np.log(np.exp(1) - 1) - soft_th)))-0.001


def calc_eq_point(x_p, y_p, x_map, y_map, def_x, def_y):
    """
    Interpolate point from the deformation vectors. Interpolation is calculated as a weighted average of ALL
    the obtained matches, in terms of the distance to the point (inversely proportional). A measure of
    confidence between 0 and 1 is also returned, where 1 means 100% confident

    :param x_p: x coordinate of point to be matched
    :param y_p: y coordinate of point to be matched
    :param x_map: map vector of x points between images
    :param y_map: map vector of y points between images
    :param def_x: x deformation vector between images
    :param def_y: y deformation vector between images
    :return: new point (x,y) coordinates and confidence measure
    """
    norm = np.divide(1, np.power(np.power(x_map - x_p, 2) + np.power(y_map - y_p, 2), 1) + 1)
    conf = sum(norm)
    norm = norm / conf

    new_x = np.sum(np.multiply(norm, def_x))
    new_y = np.sum(np.multiply(norm, def_y))

    return new_x, new_y, conf


def second_largest(numbers):
    """
    Obtain second largest value in input iterable object
    :param numbers: iterable object
    :return: second largest value in input object
    """

    count = 0
    m1 = m2 = float('-inf')
    for n in numbers:
        count += 1
        if n > m2:
            if n >= m1:
                m1, m2 = n, m1
            else:
                m2 = n
    return m2 if count >= 2 else None


def knn_match_filter(knn_matches, knn_weight):
    """
    Filters the matches of the knn algorithm according to the specified weight.

    :param knn_matches: list of knn matches
    :rtype knn_matches: list
    :param knn_weight: how larger must be the highest match with respect to the second to be considered a good match
    :rtype knn_weight: float

    :return: the matches of the original list that fulfill the above criteria
    """

    # Some matching algorithms do not return a list type. Ensure list type output
    to_list = False
    if type(knn_matches[0]) is not list:
        to_list = True

    match_in_range = []

    for knn_match in knn_matches:
        if to_list:
            knn_match = [knn_match]
        node_distance_list = list(node.distance for node in knn_match)

        # Apply ratio test to eliminate False positives.
        if max(node_distance_list) > second_largest(node_distance_list) * knn_weight:
            good_match_index = node_distance_list.index(max(node_distance_list))
            match_in_range.append(knn_match[good_match_index])

    return match_in_range


def is_rotation_matrix(r_mat):
    """
    Checks if a matrix is a valid rotation matrix (i.e. its determinant is 1)

    :param r_mat: 3x3 candidate rotation matrix
    :return: whether or not the rotation matrix is valid
    :rtype: bool
    """
    r_mat_transpose = np.transpose(r_mat)
    identity_candidate = np.dot(r_mat_transpose, r_mat)
    n = np.linalg.norm(np.identity(3, dtype=r_mat.dtype) - identity_candidate)
    if n < 1e-6:
        return True
    return False


def rotation_matrix_to_euler_angles(r_mat):
    """
    Converts a rotation matrix into euler angles if rotation is not singular

    :param r_mat: 3x3 rotation matrix
    :return: 1x3 vector of euler angles
    """
    assert (is_rotation_matrix(r_mat))

    sy = math.sqrt(r_mat[0, 0] * r_mat[0, 0] + r_mat[1, 0] * r_mat[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(r_mat[2, 1], r_mat[2, 2])
        y = math.atan2(-r_mat[2, 0], sy)
        z = math.atan2(r_mat[1, 0], r_mat[0, 0])
    else:
        x = math.atan2(-r_mat[1, 2], r_mat[1, 1])
        y = math.atan2(-r_mat[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def qv_multiply(q1, v1):
    """
    Rotate vector v1 by quaternion q1

    :param q1: rotation quaternion
    :param v1: position vector
    :return: v1 rotated by q1
    """
    q2 = list(np.squeeze(v1))
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2),
        tf.transformations.quaternion_conjugate(q1)
    )[:3]


def create_circular_mask(h, w, center=None, radius=None):
    """
    Creates a boolean mask of sizes hxw with a circle described by its center and radius

    :param h: height of mask
    :type h: float
    :param w: width of mask
    :type w: float
    :param center: center of circle
    :type center: (2,) tuple
    :param radius: radius of circle
    :type radius: float
    :return: A boolean mask containing a circle of 'True' values described by its center and radius
    :rtype: boolean ndarray (wxh)
    """

    if center is None:
        # use the middle of the image
        center = [int(w/2), int(h/2)]
    if radius is None:
        # use the smallest distance between the center and image walls
        radius = min(center[0], center[1], w-center[0], h-center[1])

    Y, X = np.ogrid[:h, :w]
    dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)

    mask = dist_from_center <= radius
    return mask


def create_exponential_mask(h, w, center):

    y_points, x_points = np.ogrid[h:0:-1, :w]
    coeffs = [w / (2 * h ** 2), 0, 0]
    y_points_poly = np.concatenate((y_points ** 2, y_points, np.ones((int(h), 1))), axis=1)
    y_points = np.expand_dims(np.matmul(y_points_poly, coeffs), axis=1)
    return abs(x_points - center[0]) - y_points >= 0


def create_geometric_mask(h, w, mask_params):
    """
    Creates a boolean mask for splitting an image in two sectors

    :param h: height of mask
    :type h: float
    :param w: width of mask
    :type w: float
    :param mask_params: array of parameters for creating the mask
    :type mask_params: list
    :return: A boolean mask of sizes wxh
    :rtype: boolean ndarray (wxh)
    """
    proximity_r = \
        (((mask_params[0] - mask_params[1]) * h) ** 2 + (0.5 * w) ** 2) / (-2 * (mask_params[0] - mask_params[1]) * h)
    proximity_mask = \
        create_circular_mask(h, w, center=(w / 2, (1 - mask_params[1]) * h + proximity_r), radius=proximity_r)
    proximity_mask_2 = create_exponential_mask(h, w, (w / 2, 0))
    proximity_mask = ~(~proximity_mask + ~proximity_mask_2)

    proximity_mask_2 = create_circular_mask(h, w, center=(w / 2, (1 - mask_params[1]) * h), radius=mask_params[2] * w)
    proximity_mask = ~proximity_mask_2 + proximity_mask
    return proximity_mask


def camera_inverse_projection(points, camera_k, camera_d, shrink_factors):
    """
    Performs the camera inverse projection assuming a given image downsapling factor

    :param points: list of n 2-D points listed along axis 0
    :type points: ndarray (nx2)
    :param camera_k: camera intrinsic calibration matrix
    :type camera_k: ndarray(3x3)
    :param camera_d: camera deformation matrix
    :type camera_d: ndarray(4x1) or (5x1)
    :param shrink_factors: ratio of down-sampling, in the range (0,1]
    :type shrink_factors: tuple (2,1)
    :return: list of original points
    :rtype: ndarray (nx2)
    """

    points = recover_downsampling(points, shrink_factors)
    rectified_query_points = np.squeeze(cv2.undistortPoints(np.expand_dims(points, axis=0), camera_k, camera_d))
    return rectified_query_points


def recover_downsampling(points, shrink_factors):
    """
    Recovers the original coordinates of image pixels after down-sampling

    :param points: list of n vectors of dimension m arranged along axis 1
    :type points: ndarray (nxm)
    :param shrink_factors: ratio of down-sampling, in the range (0,1]
    :type shrink_factors: tuple (2,1)
    :return: The recovered pixel coordinates
    :rtype: ndarray (nxm)
    """

    correction_vec = [[1 / shrink_factors[1], 0], [0, 1 / shrink_factors[0]]]
    points = np.matmul(np.asarray(points), correction_vec)
    return points


def normalize_points(points):
    """
    Normalizes a list of vectors such as they have modulus 1

    :param points: list of n vectors of dimension m arranged along axis 1
    :type points: ndarray (nxm)
    :return: normalized list of vectors
    """

    normalization_vec = np.sqrt(np.sum(points ** 2, axis=1))
    normalization_vec = np.matmul(np.expand_dims(normalization_vec, axis=1), np.ones((1, 2)))
    return np.multiply(points, 1/normalization_vec)
