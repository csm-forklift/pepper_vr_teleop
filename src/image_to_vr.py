#!/usr/bin/env python
'''
Reads in an image from a ROS topic and adjusts it to work with the VR left and
right eye images. The images must be shifted so that they overlap in the
headset's view.
'''


import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageToVR():
    def __init__(self):
        #===== Start ROS Node =====#
        rospy.init_node("image_to_vr")
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        # Read Parameters
        self.image_topic = rospy.get_param('~image_topic', '/pepper_interface/camera/front/image_raw')
        self.left_image_topic = rospy.get_param('~left_image_topic', '/image_left')
        self.right_image_topic = rospy.get_param('~right_image_topic', '/image_right')

        print("[{0}]: Reading image from '{1}', publishing left to '{2}', publishing right to '{3}'".format(rospy.get_name(), self.image_topic, self.left_image_topic, self.right_image_topic))

        #--- Image Shifting Parameter ---#
        # The image comes into focus as about 0.05, then it starts to get too
        # far away at about 0.2. So ~0.13 is a decent safe spot.
        self.image_shift = 0.13 # percentage as a fraction

        # Create Publishers
        self.left_image_pub = rospy.Publisher(self.left_image_topic, Image, queue_size=1)
        self.right_image_pub = rospy.Publisher(self.right_image_topic, Image, queue_size=1)

        # Create Subscribers
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.imageCallback, queue_size=1)

    def shiftImage(self, image, shift_percent):
        '''
        Takes a cv2 image and shifts it by the number of pixel contained in
        'shift_percent' of the image width.
        A positive number shifts the image to the right and a negative number
        shifts to the left.
        A new image is returned. The original is not altered.
        '''
        dimensions = image.shape
        height = dimensions[0]
        width = dimensions[1]
        shift = shift_percent * width
        T = np.float32([[1, 0, shift], [0, 1, 0]])

        return cv2.warpAffine(image, T, (width, height))

    def imageCallback(self, msg):
        #--- Convert to CV image
        encoding = msg.encoding
        cv_image = self.bridge.imgmsg_to_cv2(msg, encoding)

        #--- Shift Left Image
        cv_left_image = self.shiftImage(cv_image, self.image_shift)

        #--- Shift Right Image
        cv_right_image = self.shiftImage(cv_image, -self.image_shift)

        #--- Convert to ROS messages
        left_image_msg = self.bridge.cv2_to_imgmsg(cv_left_image, encoding)
        right_image_msg = self.bridge.cv2_to_imgmsg(cv_right_image, encoding)

        #--- Publish
        self.left_image_pub.publish(left_image_msg)
        self.right_image_pub.publish(right_image_msg)


if __name__ == '__main__':
    try:
        image_to_vr = ImageToVR()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
