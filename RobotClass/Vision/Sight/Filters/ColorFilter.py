#import needed libraries
import numpy as np
import cv2

#Class to filter colors from camera images
class ColorFilter():


    def __init__(self):
        self.lower_bound_filter_range = None
        self.upper_bound_filter_range = None




    def set_filter(self, hue: int, precision: int):
        '''Sets the color filter range based on the specified hue and precision.

        This method adjusts the color filter to focus on a specific hue with a certain precision, allowing for
        accurate color detection. The hue parameter determines the central color of interest, while the precision
        parameter controls the width of the color range to include.'''

        #Check to ensure the precision is not too large
        if (precision > 90 or precision < 0) or (hue < 0 or hue >= 180):
            raise(ValueError("Invalid input for either hue or precision"))
        #Set the filter for hue ranges wrapping from 0 -> 180
        if hue - precision < 0:
            self.lower_bound_filter_range = (np.uint8([(hue-precision)+180, 170, 136]), np.uint8([180, 255, 255]))
            self.upper_bound_filter_range = (np.uint8([0, 170, 136]), np.uint8([hue+precision, 255, 255]))
        #Set the filter for hue ranges wrapping from 180 -> 0
        elif hue + precision >= 180:
            self.lower_bound_filter_range = (np.uint8([hue-precision, 170, 136]), np.uint8([180, 255, 255]))
            self.upper_bound_filter_range = (np.uint8([0, 170, 136]), np.uint8([(hue+precision)-180, 255, 255]))
        #Set the filter for hue ranges when there is no wrapping involved
        else:
            self.lower_bound_filter_range = (np.uint8([hue-precision, 170, 136]), np.uint8([hue, 255, 255]))
            self.upper_bound_filter_range = (np.uint8([hue, 170, 136]), np.uint8([hue+precision, 255, 255]))




    def get_color_filter_mask(self, image: np.ndarray) -> np.ndarray:
        '''Obtains a filter mask based on the specified color filter.

        This method applies the configured color filter range to the input image, generating a binary mask
        that highlights the areas in the image matching the specified color criteria.'''


        #Blur the image to get rid of noise (small background objects)
        image = cv2.blur(image, (16,16))
        #Convert the image to an HSV format to more easily analyze the colors
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #Get the mask for the lower bound filter range
        mask = cv2.inRange(image, *self.lower_bound_filter_range)
        #Then update the mask to account for the upper bound filter range
        mask = cv2.bitwise_or(mask, cv2.inRange(image, *self.upper_bound_filter_range))
        #Last, return the mask
        return mask




    def filter_image(self, image: np.ndarray) -> np.ndarray:
        '''Filter the input image based on the color filter settings.

        This method applies the color filter mask to the input image, retaining only the areas that match
        the specified color criteria.'''

        #First, get the color filter mask
        mask = self.get_image_filter_mask(image)
        #Then apply the mask to the image
        image = cv2.bitwise_and(image, image, mask=mask)
        #Last, return the filtered image
        return image
