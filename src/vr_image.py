#!/usr/bin/env python
'''
This node reads in an image from a camera and publishes it to the image topics
for the VR headset.
'''

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class VR_Image():
    def __init__(self):
        # Start Node
        rospy.init_node("vr_image")
        self.frequency = 90
        self.rate = rospy.Rate(self.frequency)

        # Create Publishers
        self.image_left_pub = rospy.Publisher('/image_left', Image, queue_size=1)
        self.image_right_pub = rospy.Publisher('/image_right', Image, queue_size=1)

        # Create subscribers
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.imageCallback, queue_size=1)

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def imageCallback(self, msg):
        '''
        Read the camera's image and publish to the left and right eyes of the
        headset.
        '''
        self.image_left_pub.publish(msg)
        self.image_right_pub.publish(msg)

if __name__ == "__main__":
    try:
        vr_image = VR_Image()
        vr_image.spin()
    except rospy.ROSInterruptException:
        pass
