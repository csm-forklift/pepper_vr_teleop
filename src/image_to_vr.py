#!/usr/bin/env python
""" Shifts an image so it will be in focus on the VR headset.

Reads in an image from a ROS topic and adjusts it to work with the VR left and
right eye images. The images must be shifted so that they overlap in the
headset's view.

ROS Node Description
====================
Parameters
----------
~image_topic : str, default: '/pepper_interface/camera/front/image_raw'
    The topic which is publishing the image to be sent to the VR headset.
~left_image_topic : str, default: '/image_left'
    The topic that publishes the left image for the VR headset.
~right_image_topic : str, default: '/image_right'
    The topic that publishes the right image for the vR headset.

Published Topics
----------------
<~left_image_topic> : sensor_msgs/Image
    The left image for the VR headset.
<~right_image_topic> : sensor_msgs/Image
    The right image for the VR headset.

Subscribed Topics
-----------------
<~image_topic> : sensor_msgs/Image
    The image to shift then publish to the VR headset.
"""

# Python
import numpy as np

# ROS
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageToVR():
    def __init__(self):
        #======================================================================#
        # ROS Setup
        #======================================================================#
        rospy.init_node("image_to_vr")

        #===== Parameters =====#
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)
        self.bridge = CvBridge() # for image processing

        self.image_topic = rospy.get_param('~image_topic', '/pepper_interface/camera/front/image_raw')
        self.left_image_topic = rospy.get_param('~left_image_topic', '/image_left')
        self.right_image_topic = rospy.get_param('~right_image_topic', '/image_right')

        print("[{0}]: Reading image from '{1}', publishing left to '{2}', publishing right to '{3}'".format(rospy.get_name(), self.image_topic, self.left_image_topic, self.right_image_topic))

        #--- Image Shifting Parameter ---#
        # The image comes into focus at about 0.05, then it starts to get too
        # far away at about 0.2. So about 0.13 is a decent safe spot.
        self.image_shift = 0.13 # percentage as a fraction

        #===== Publishers =====#
        self.left_image_pub = rospy.Publisher(self.left_image_topic, Image, queue_size=1)
        self.right_image_pub = rospy.Publisher(self.right_image_topic, Image, queue_size=1)

        #===== Subscribers =====#
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.imageCallback, queue_size=1)

    def shiftImage(self, image, shift_percent):
        """ Shifts an image horizontally.

        Takes a cv2 image and shifts it by the number of pixels contained in
        'shift_percent' multiplied by the image width. A positive number shifts
        the image to the right and a negative number shifts it to the left. A
        new image is returned. The original is not altered.

        Parameters
        ----------
        image : cv2 image
            A cv2 image which is represented as a numpy ndarray.
        shift_percent : float
            The percentage of the image width to shift the image. This
            percentage is represented as a decimal such that 100% = 1.0.

        Returns
        -------
        cv2 image
            The shifted image with new pixels filled with black.
        """
        dimensions = image.shape
        height = dimensions[0]
        width = dimensions[1]
        shift = shift_percent * width
        T = np.float32([[1, 0, shift], [0, 1, 0]])

        return cv2.warpAffine(image, T, (width, height))

    #==========================================================================#
    # Callback Function
    #==========================================================================#
    def imageCallback(self, msg):
        """ Reads the image, shifts, and republishes.

        The VR display in the headset has the images directly in front of each
        eye. This makes them not overlap in the user's view, so they are always
        out of focus. In order to get the images in focus, the right image must
        be shifted to the left and the left image must be shifted to the right.
        """
        #===== Shift and Republish to Each Eye =====#
        #----- Convert to CV image -----#
        encoding = msg.encoding
        cv_image = self.bridge.imgmsg_to_cv2(msg, encoding)

        #----- Shift Left Image -----#
        cv_left_image = self.shiftImage(cv_image, self.image_shift)

        #----- Shift Right Image -----#
        cv_right_image = self.shiftImage(cv_image, -self.image_shift)

        #----- Convert to ROS messages -----#
        left_image_msg = self.bridge.cv2_to_imgmsg(cv_left_image, encoding)
        right_image_msg = self.bridge.cv2_to_imgmsg(cv_right_image, encoding)

        #----- Publish -----#
        self.left_image_pub.publish(left_image_msg)
        self.right_image_pub.publish(right_image_msg)


if __name__ == '__main__':
    try:
        image_to_vr = ImageToVR()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
