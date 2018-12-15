import cv2


class ImageManager:

    def find_keypoints(self, detector):
        """
        Finds the keypoints of stored image given a cv2 detector

        :param detector: cv2 feature detector
        """
        self.keypoints, self.descriptors = detector.detectAndCompute(self.image, None)

    def downsample(self, shrink_x, shrink_y):
        """
        Downsamples an image given the x and y ratios

        :param shrink_x: shrink factor for x axis
        :param shrink_y: shrink factor for y axis
        """
        self.image = cv2.resize(self.image, (0, 0), fx=shrink_x, fy=shrink_y)
        self.height *= shrink_x
        self.width *= shrink_y

    def load_image(self, image, gray_scale):
        """
        Load an image

        :param image: image
        :param gray_scale: whether image should be converted to grayscale
        :type gray_scale: bool
        :type image: ndarray
        """

        self.image = image
        if gray_scale:
            self.image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            self.height, self.width = self.image.shape
        else:
            self.height, self.width, self.channels = self.image.shape

    def read_image(self, image_path):
        """
        :param image_path: path to image
        :type image_path: str

        Tries to read an image in color mode, or in gray-scale if not possible
        """

        self.image_path = image_path

        try:
            self.image = cv2.imread(self.image_path, cv2.IMREAD_COLOR)  # Query image

            # Save height, width and channels
            self.height, self.width, self.channels = self.image.shape

        except Exception as e:
            self.image = cv2.imread(self.image_path)
            self.height, self.width = self.image.shape
            print("Image is in gray scale: ", e)

    def __init__(self):
        self.image_path = []
        self.height = 0
        self.width = 0
        self.channels = 1
        self.image = []

        self.keypoints = []
        self.descriptors = []
