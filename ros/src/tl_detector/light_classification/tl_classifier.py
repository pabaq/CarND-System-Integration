from styx_msgs.msg import TrafficLight

import cv2
import numpy as np

from collections import deque
from utilities import *

# Size of light color queue used as buffer for noisy classifications
BUFFER_SIZE = 2
# Color intensity threshold
INTENSITY_THRES = 1e5
# Light state convertion
LIGHT_STATES = {0: TrafficLight.RED,
                1: TrafficLight.YELLOW,
                2: TrafficLight.GREEN}


class TLClassifier(object):
    def __init__(self):
        # Initialize the light state to be unknown
        self.light_state = TrafficLight.UNKNOWN
        # Initialize the buffer used for the traffic light classification
        self.color_intensity_buffer = deque(maxlen=BUFFER_SIZE)
        # Initialize the logger
        self.logger = Logger()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        self.logger.reset()

        # Get traffic light color intensities determined from the current image
        color_intensities = self.get_color_intensities_from(image)
        # Append them to the buffer
        self.color_intensity_buffer.append(color_intensities)
        # Sum up the intesities of each color in the buffer
        buffered_color_intensities = np.sum(self.color_intensity_buffer, axis=0)
        if np.any(buffered_color_intensities > INTENSITY_THRES):
            # The color with highest intensity represents traffic light's color
            state_id = int(np.argmax(buffered_color_intensities))
            light_state = LIGHT_STATES[state_id]
        else:
            light_state = TrafficLight.UNKNOWN

        self.logger.info('color intensity (r|g|b):  %8.0f%8.0f%8.0f',
                         *color_intensities)
        self.logger.info('buffer intensity (r|g|b): %8.0f%8.0f%8.0f',
                         *buffered_color_intensities)

        return light_state

    def reset_buffer(self):
        """ Reset the traffic light color intensity buffer.

        This usually happens when there are no more traffic lights nearby, so
        that the buffer has a clean start for the next detected traffic light.
        """
        self.color_intensity_buffer = deque(maxlen=BUFFER_SIZE)

    @staticmethod
    def get_color_intensities_from(image):
        """
        Apply thesholds to the image and determine the intensity of each color.

        Up to now the classification is performed by color thresholding in the
        hsv color space. Three masks are defined to isolate the red, yellow
        and green areas in the image. The summation of the active pixels in each
        mask represent the intensities of each color.

        In the future this classification approach may be replaced by
        Tensorflow's Object Detection API.

        Args:
            image:
                the current camera image in BGR color space

        Returns:
            list of color intensities [red, yellow, green] determined from the
             image
        """

        # Convert the image into the HSV color space
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Define the mask for each color
        red_mask_1 = cv2.inRange(hsv_img,
                                 (0, 20, 120),
                                 (5, 255, 255))
        red_mask_2 = cv2.inRange(hsv_img,
                                 (150, 20, 120),
                                 (180, 255, 255))
        red_mask = cv2.bitwise_or(red_mask_1, red_mask_2).astype(float)
        ylw_mask = cv2.inRange(hsv_img,
                               (20, 40, 200),
                               (40, 255, 255)).astype(float)
        grn_mask = cv2.inRange(hsv_img,
                               (40, 100, 150),
                               (80, 255, 255)).astype(float)
        # Compute the color weights by summing up the active pixels in the masks
        color_intensities = [red_mask.sum(), ylw_mask.sum(), grn_mask.sum()]
        return color_intensities
