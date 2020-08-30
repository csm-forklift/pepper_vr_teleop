#!/usr/bin/env python
"""
ROS node that subscribes to a camera feed and then sends the images to an
Android application through TCP. The Android app is called "Pepper Image
Stream".
"""

from libtcp import TCPClient

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageStream(object):
    def __init__(self):
        #======================================================================#
        # Initialize ROS
        #======================================================================#
        rospy.init_node('image_stream')
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        #===== Parameters =====#
        self.server_ip = rospy.get_param('~server_ip', '127.0.0.1')
        self.server_port = rospy.get_param('~server_port', 65432)
        self.image_type = rospy.get_param('~image_type', 'jpeg')
        if (self.image_type not in TCPClient.get_available_image_types()):
            rospy.logwarn('[{0}]: image_type \'{1}\' does not match the available types: {2}. Setting value to default: \'jpeg\'.'.format(rospy.get_name(), self.image_type, TCPClient.get_available_image_types()))
        if (self.image_type == 'bmp'):
            rospy.loginfo('[{0}]: Using \'bmp\' image type does not allow any compression. The \'compression\' parameter will be ignored.'.format(rospy.get_name()))
        self.compression = rospy.get_param('~compression', 50)

        # Saturate between 0 and 100
        self.compression = min(self.compression, 100)
        self.compression = max(self.compression, 0)

        # Image topic
        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_color')

        # Print parameters for operator debugging
        rospy.loginfo('[{0}]: Image stream connection information:\n\tServer Address: {1}:{2}\n\tImage type: {3}, Compression: {4}%\n\tSubscribing to: {5}\n'.format(rospy.get_name(), self.server_ip, self.server_port, self.image_type, self.compression, self.image_topic))

        #===== TCP Client =====#
        self.client = TCPClient(self.server_ip, self.server_port, self.image_type, self.compression)

        #===== Subscribers =====#
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.imageCallback, queue_size=1)

    def spin(self):
        while not rospy.is_shutdown():
            success = self.client.spin_once()
            if not success:
                break
            self.rate.sleep()

    def imageCallback(self, msg):
        # Convert ROS Image msg to opencv format
        image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)

        # Send to client
        self.client.set_new_image(image)

if __name__ == '__main__':
    try:
        image_stream = ImageStream()
        image_stream.spin()
    except rospy.ROSInterruptException:
        pass
