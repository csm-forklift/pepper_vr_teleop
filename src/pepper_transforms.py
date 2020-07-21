'''
Module containing two objects used for the VR Controller Inverse Kinematics.

Transform:

PepperModel:

'''

import numpy as np
import rospy
import tf

class Transform():
    def __init__(self, position, quaternion, parent, child):
        '''
        Parameters
        ----------
        position: list of float, size: 3
            [x,y,z] position of transform origin
        quaternion: list of float, size: 4
            [x,y,z,w] quaternion representing frame rotation
        parent: str
            name of the parent frame
        child: str
            name of the child frame
        '''
        self.position = position
        self.quaternion = quaternion
        self.parent = parent
        self.child = child

    def broadcast(self, broadcaster):
        '''
        Publishes the transform to the '/tf' topic.

        Parameters
        ----------
        broadcaster: tf.TransformBroadcaster
            broadcaster object used to publish the transform
        '''
        try:
            broadcaster.sendTransform(self.position, self.quaternion, rospy.Time.now(), self.child, self.parent)
        except tf.Exception as err:
            rospy.logwarn('[{0}]: {1}'.format(rospy.get_name(), err))

    def listen(self, listener, wait_time=1.0):
        '''
        Listens for the transform on '/tf'.

        Parameters
        ----------
        listener: tf.TransformListener
            listener object used to read the position and orientation
        wait_time: float
            time in seconds to wait for the transform
        '''
        try:
            listener.waitForTransform(self.parent, self.child, rospy.Time(), rospy.Duration.from_sec(wait_time))
            self.position, self.quaternion = listener.lookupTransform(self.parent, self.child, rospy.Time())
        except tf.Exception as err:
            rospy.logwarn('[{0}]: {1}'.format(rospy.get_name(), err))

    def matrix(self):
        '''
        Converts the position/quaterion pair into a 4x4 transformation matrix

        Returns
        -------
        T: 4x4 numpy array
            the transformation matrix defined by the position and quaternion
        '''
        T = tf.transformations.quaternion_matrix(self.quaternion)
        T[0:3,3] = self.position
        return T
