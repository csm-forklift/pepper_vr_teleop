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


class PepperModel():
    def __init__(self, origin_angles=None):
        '''
        Defines Pepper's links and joints. Determines the origin position of
        each hand.

        Parameters
        ----------
        origin_angles: dict of floats, optional
            Joint angles used for the origin pose. Default values are 0. Keys:
            ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw'. 'LElbowRoll',
            'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw',
            'RElbowRoll', 'RWristYaw']
        '''
        #======================================================================#
        # Pepper Description
        #======================================================================#
        #===== Joint Angles =====#
        self.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        self.joint_names_left = self.joint_names[0:5]
        self.joint_names_right = self.joint_names[5:10]
        # dictionary containing the current joint configuration
        self.current_angles = {}
        for key in self.joint_names:
            self.current_angles[key] = 0.0

        #===== Link and Joint Specs =====#
        # Offsets
        # All values are in meters, tables of link offsets is found at:
        #    doc.aldebaran.com/2-5/family/pepper_technical/links_pep.html
        # The 'torso' link is called 'base_link_V' in this code

        # Left Arm
        self.base_link_V_to_LShoulder = [-0.057, 0.14974, 0.08682]
        self.LShoulder_to_LElbow = [0.1812, 0.015, 0.00013]
        # The website has a Y offset of 0.0236, but the simulation model has 0
        # The elbow joint has a slight incline which gives a Z offset in the 0
        # position, but when considering the Wrist frame in the Elbow frame,
        # this is just a straightline distance along the X axis. The length is
        # found from the distance formula using the X and Z lengths from the
        # documentation.
        self.LElbow_to_LWrist = [0.1517, 0.0, 0.0]
        self.LWrist_to_LHand = [0.0695, 0.0, -0.03030]

        # Right Arm
        self.base_link_V_to_RShoulder = [-0.057, -0.14974, 0.08682]
        self.RShoulder_to_RElbow = [0.1812, -0.015, 0.00013]
        self.RElbow_to_RWrist = [0.1517, 0.0, 0.0]
        self.RWrist_to_RHand = [0.0695, 0.0, -0.03030]

        #===== Frame Transforms =====#
        self.LShoulder = Transform(self.base_link_V_to_LShoulder, [0, 0, 0, 1], 'base_link_V', 'LShoulder_V')
        self.LElbow = Transform(self.LShoulder_to_LElbow, [0, 0, 0, 1], 'LShoulder_V', 'LElbow_V')
        self.LWrist = Transform(self.LElbow_to_LWrist, [0, 0, 0, 1], 'LElbow_V', 'LWrist_V')
        self.LHand = Transform(self.LWrist_to_LHand, [0, 0, 0, 1], 'LWrist_V', 'LHand_V')
        self.RShoulder = Transform(self.base_link_V_to_RShoulder, [0, 0, 0, 1], 'base_link_V', 'RShoulder_V')
        self.RElbow = Transform(self.RShoulder_to_RElbow, [0, 0, 0, 1], 'RShoulder_V', 'RElbow_V')
        self.RWrist = Transform(self.RElbow_to_RWrist, [0, 0, 0, 1], 'RElbow_V', 'RWrist_V')
        self.RHand = Transform(self.RWrist_to_RHand, [0, 0, 0, 1], 'RWrist_V', 'RHand_V')

        #======================================================================#
        # Set Hand Origins
        #======================================================================#
        #===== Set Initial Pose =====#
        # If the user provides a dictionary containing joint angles, set that as
        # the initial pose used to get the hand origin positions
        if origin_angles is not None:
            for key in origin_angles.keys()
                self.current_angles[key] = origin_angles[key]
        else:
            #--- Default Origin Angles ---#
            self.current_angles['LShoulderPitch'] = np.pi/2
            self.current_angles['RShoulderPitch'] = np.pi/2

        #===== Set Origins =====#
        self.updateTransforms()
        left_transform = self.getLeftHandTransform()
        right_transform = self.getRightHandTransform()
        self.left_hand_origin = left_transform[0:3,3]
        self.right_hand_origin = right_transform[0:3,3]

    def broadcastTransforms(self, broadcaster):
        '''
        Broadcasts the transforms on the '/tf' topic.

        Parameters
        ----------
        broadcaster: tf.TransformBroadcaster
            Broadcaster object used to publish the transforms.
        '''
        # Left Arm
        self.LShoulder.broadcast(broadcaster)
        self.LElbow.broadcast(broadcaster)
        self.LWrist.broadcast(broadcaster)
        self.LHand.broadcast(broadcaster)

        # Right Arm
        self.RShoulder.broadcast(broadcaster)
        self.RElbow.broadcast(broadcaster)
        self.RWrist.broadcast(broadcaster)
        self.RHand.broadcast(broadcaster)

    def setAnglesLeft(self, joint_angles_left):
        '''
        Updates the current angles for the left arm.

        Parameters
        ----------
        joint_angles_left: list of float
            The desired joint angles for the left arm in the following order:
            ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
            'LWristYaw']
        '''
        for i in range(len(self.joint_names_left)):
            self.current_angles[self.joint_names_left[i]] = joint_angles_left[i]

    def setAnglesRight(self, joint_angles_right):
        '''
        Updates the current angles for the right arm.

        Parameters
        ----------
        joint_angles_right: list of float
            The desired joint angles for the right arm in the following order:
            ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll',
            'RWristYaw']
        '''
        for i in range(len(self.joint_names_right)):
            self.current_angles[self.joint_names_right[i]] = joint_angles_right[i]

    def getAngles(self):
        '''
        Returns the current joint angle configuration as a list of floats.

        Returns
        -------
        The current_angles values as a list
        '''
        return [self.current_angles[key] for key in self.joint_names]

    def updateTransformsLeft(self):
        '''
        Updates the left arm transforms using the current angle configuration.
        '''
        self.LShoulder.quaternion = tf.transformations.quaternion_from_euler(0, self.current_angles['LShoulderPitch'], self.current_angles['LShoulderRoll'], 'rxyz')
        self.LElbow.quaternion = tf.transformations.quaternion_from_euler(self.current_angles['LElbowYaw'], 0, self.current_angles['LElbowRoll'], 'rxyz')
        self.LWrist.quaternion = tf.transformations.quaternion_from_euler(self.current_angles['LWristYaw'], 0, 0, 'rxyz')

    def updateTransformsRight(self):
        '''
        Updates the right arm transforms using the current angle configuration.
        '''
        self.RShoulder.quaternion = tf.transformations.quaternion_from_euler(0, self.current_angles['RShoulderPitch'], self.current_angles['RShoulderRoll'], 'rxyz')
        self.RElbow.quaternion = tf.transformations.quaternion_from_euler(self.current_angles['RElbowYaw'], 0, self.current_angles['RElbowRoll'], 'rxyz')
        self.RWrist.quaternion = tf.transformations.quaternion_from_euler(self.current_angles['RWristYaw'], 0, 0, 'rxyz')

    def setTransformsLeft(self, joint_angles_left):
        '''
        Updates the current angles for the left arm, then updates the
        transforms.

        Parameters
        ----------
        joint_angles_left: list of float
            The desired joint angles for the left arm in the following order:
            ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
            'LWristYaw']
        '''
        self.setAnglesLeft(joint_angles_left)
        self.updateTransformsLeft()

    def setTransformsRight(self, joint_angles_right):
        '''
        Updates the current angles for the right arm, then updates the
        transforms.

        Parameters
        ----------
        joint_angles_right: list of float
            The desired joint angles for the right arm in the following order:
            ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll',
            'RWristYaw']
        '''
        self.setAnglesRight(joint_angles_right)
        self.updateTransformsRight()

    def setAngles(self, joint_angles):
        '''
        Updates the current angles for both arms.

        Parameters
        ----------
        joint_angles: list of float
            The desired joint angles for both arms in the following order:
            ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
            'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw',
            'RElbowRoll', 'RWristYaw']
        '''
        self.setAnglesLeft(joint_angles[0:5])
        self.setAnglesRight(joint_angles[5:10])

    def updateTransforms(self):
        '''
        Updates all transforms using the current angle configuration.
        '''
        self.updateTransformsLeft()
        self.updateTransformsRight()

    def setTransforms(self, joint_angles):
        '''
        Updates the current angles for both arms, then updates the transforms.

        Parameters
        ----------
        joint_angles: list of float
            The desired joint angles for both arms in the following order:
            ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
            'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw',
            'RElbowRoll', 'RWristYaw']
        '''
        self.setTransformsLeft(joint_angles[0:5])
        self.setTransformsRight(joint_angles[5:10])

    def getLeftHandTransform(self):
        '''
        Calculates the left hand frame ('LHand_V') transform in the
        'base_link_V' frame. Returns it as a 4x4 numpy array.

        Returns
        -------
        base_T_lhand: 4x4 numpy array
            The transformation matrix representing the pose of 'LHand_V' in the
            'base_link_V' frame.
        '''
        base_T_lshoulder = self.LShoulder.matrix()
        lshoulder_T_lelbow = self.LElbow.matrix()
        lelbow_T_lwrist = self.LWrist.matrix()
        lwrist_T_lhand = self.LHand.matrix()
        base_T_lhand = base_T_lshoulder.dot(lshoulder_T_lelbow.dot(lelbow_T_lwrist.dot(lwrist_T_lhand)))
        return base_T_lhand

    def getRightHandTransform(self):
        '''
        Calculates the right hand frame ('RHand_V') transform in the
        'base_link_V' frame. Returns it as a 4x4 numpy array.

        Returns
        -------
        base_T_rhand: 4x4 numpy array
            The transformation matrix representing the pose of 'RHand_V' in the
            'base_link_V' frame.
        '''
        base_T_rshoulder = self.RShoulder.matrix()
        rshoulder_T_relbow = self.RElbow.matrix()
        relbow_T_rwrist = self.RWrist.matrix()
        rwrist_T_rhand = self.RHand.matrix()
        base_T_rhand = base_T_rshoulder.dot(rshoulder_T_relbow.dot(relbow_T_rwrist.dot(rwrist_T_rhand)))
        return base_T_rhand
