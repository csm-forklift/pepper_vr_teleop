""" Module containing classes used to generate a forward kinematics model of
Pepper.

The 'vr_hand_controllers' node requires a model of Pepper in order to perform
inverse kinematics to determine the optimal joint configuration to produce a
desired hand orientation. The classes in this module were designed to facilitate
this action. The PepperModel class is able to perform the forward kinematics
given a joint configuration. The 'vr_hand_controllers' node solves the inverse
kinematics problem by iteratively performing the forward kinematics until the
input joint configuration produces the desired output hand pose. This is done
using a scipy optimization solver.

Classes
-------
Transform : object
    Class containing transformation data.
PepperModel : object
    Defines links and joints for a kinematic Pepper model.
"""

# Python
import numpy as np

# ROS
import rospy
import tf


class Transform(object):
    """ Class containing transformation data.

    Class that stores the position and orientation values for a transformation.
    Also contains methods for broadcasting and listening to transforms on the
    ROS TF tree.

    Attributes
    ----------
    position : list of float
        [x,y,z] position of transform origin.
    quaternion : list of float
        [x,y,z,w] quaternion representing frame rotation.
    parent : str
        Name of the parent frame.
    child : str
        Name of the child frame.

    Methods
    -------
    broadcast(broadcaster)
        Broadcasts this transform on the ROS TF tree.
    listen(listener, wait_time=1.0)
        Listens for this transform from 'parent' to 'child' on the ROS TF tree.
    canTransform(listener)
        Checks if a transform from 'parent' to 'child' exists on the ROS TF
        tree.
    matrix()
        Returns this transform as a 4x4 transformation matrix.

    Parameters
    ----------
    See 'Attributes'.
    """
    def __init__(self, position, quaternion, parent, child):
        self.position = position
        self.quaternion = quaternion
        self.parent = parent
        self.child = child

    def broadcast(self, broadcaster):
        """ Publishes the transform to the '/tf' topic.

        Parameters
        ----------
        broadcaster : tf.TransformBroadcaster
            Broadcaster object used to publish the transform.
        """
        try:
            broadcaster.sendTransform(self.position, self.quaternion, rospy.Time.now(), self.child, self.parent)
        except tf.Exception as err:
            rospy.logwarn('[{0}]: {1}'.format(rospy.get_name(), err))

    def listen(self, listener, wait_time=1.0):
        """ Listens for the transform on '/tf'.

        Parameters
        ----------
        listener : tf.TransformListener
            Listener object used to read the position and orientation.
        wait_time : float
            Time in seconds to wait for the transform.
        """
        try:
            listener.waitForTransform(self.parent, self.child, rospy.Time(), rospy.Duration.from_sec(wait_time))
            self.position, self.quaternion = listener.lookupTransform(self.parent, self.child, rospy.Time())
        except tf.Exception as err:
            rospy.logwarn('[{0}]: {1}'.format(rospy.get_name(), err))

    def canTransform(self, listener):
        """ Checks if the transform exists in the TF tree.

        Parameters
        ----------
        listener : tf.TransformListener
            Listener object used to read the position and orientation.
        """
        return listener.canTransform(self.parent, self.child, rospy.Time())

    def matrix(self):
        """ Converts position/quaterion pair into a 4x4 transformation matrix.

        Returns
        -------
        T : 4x4 numpy array
            The transformation matrix defined by the position and quaternion.
        """
        T = tf.transformations.quaternion_matrix(self.quaternion)
        T[0:3,3] = self.position
        return T


class PepperModel(object):
    """ Defines links and joints for a kinematic Pepper model.

    Attributes
    ----------
    joint_names : list of str
        List of joint names.
    joint_names_side : list of str
        Dictionary containing the joint names for separated for each side.
    current_angles : dict
        Dictionary containing the joint names and current angle value.
    base_link_V_to_Shoulder : dict of list
        Dictionary of shoulder positions for each side.
    Shoulder_to_Elbow : dict of list
        Dictionary of elbow positions for each side.
    Elbow_to_Wrist : dict of list
        Dictionary of wrist positions for each side.
    Wrist_to_Hand : dict of list
        Dictionary of hand positions for each side.
    Shoulder : dict of Transform
        Transforms for each side from base_link to shoulder.
    Elbow : dict of Transform
        Transforms for each side from shoulder to elbow.
    Wrist : dict of Transform
        Transforms for each side from elbow to wrist.
    Hand : dict of Transform
        Transforms for each side from wrist to hand.
    hand_origin : dict of array
        Dictionary of positions for each hand.

    Methods
    -------
    checkKeys(keys)
        Checks a list of keys for valid joint names.
    checkSide(name, side)
        Validates the 'side' parameter.
    broadcastTransforms(broadcaster)
        Broadcasts the transforms on the '/tf' topic.
    getAngles()
        Returns the current joint angle configuration as a list of floats.
    setAngles(joint_angles)
        Updates current angles contained in 'joint_angles' parameter.
    _setCurrentAngles(joint_angles)
        Updates current angles contained in 'joint_angles' parameter.
    updateTransforms(side=None)
        Updates arm transforms using the current angle configuration.
    setTransforms(joint_values, side)
        Updates the current angles for the arm, then updates the transforms.
    getHandTransform(side)
        Calculates the hand transform for a given side.

    Parameters
    ----------
    origin_angles: dict of floats, optional
        Joint angles used for the origin pose. Default values are 0. Keys:
        ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw'. 'LElbowRoll',
        'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw',
        'RElbowRoll', 'RWristYaw']
    """
    def __init__(self, origin_angles=None):
        #======================================================================#
        # Pepper Description
        #======================================================================#
        #===== Joint Angles =====#
        self.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        self.joint_names_side = {
            'L': self.joint_names[0:5],
            'R': self.joint_names[5:10]
        }
        # dictionary containing the current joint configuration
        self.current_angles = {}
        for key in self.joint_names:
            self.current_angles[key] = 0.0

        #===== Link and Joint Specs =====#
        # Offsets
        # All values are in meters, tables of link offsets is found at:
        #    doc.aldebaran.com/2-5/family/pepper_technical/links_pep.html
        # The 'torso' link is called 'base_link_V' in this code
        self.base_link_V_to_Shoulder = {
            'L': [-0.057, 0.14974, 0.08682],
            'R': [-0.057, -0.14974, 0.08682]
        }
        self.Shoulder_to_Elbow = {
            'L': [0.1812, 0.015, 0.00013],
            'R': [0.1812, -0.015, 0.00013]
        }
        # The website has a Y offset of 0.0236 for the elbow to wrist transform
        # but the simulation model has 0 The elbow joint has a slight incline
        # which gives a Z offset in the 0 position, but when considering the
        # Wrist frame in the Elbow frame, this is just a straightline distance
        # along the X axis. The length is found from the distance formula using
        # the X and Z lengths from the documentation.
        self.Elbow_to_Wrist = {
            'L': [0.1517, 0.0, 0.0],
            'R': [0.1517, 0.0, 0.0]
        }
        self.Wrist_to_Hand = {
            'L': [0.0695, 0.0, -0.03030],
            'R': [0.0695, 0.0, -0.03030]
        }

        #===== Frame Transforms =====#
        self.Shoulder = {
            'L': Transform(self.base_link_V_to_Shoulder['L'], [0., 0., 0., 1.], 'base_link_V', 'LShoulder_V'),
            'R': Transform(self.base_link_V_to_Shoulder['R'], [0., 0., 0., 1.], 'base_link_V', 'RShoulder_V')
        }
        self.Elbow = {
            'L': Transform(self.Shoulder_to_Elbow['L'], [0., 0., 0., 1.], 'LShoulder_V', 'LElbow_V'),
            'R': Transform(self.Shoulder_to_Elbow['R'], [0., 0., 0., 1.], 'RShoulder_V', 'RElbow_V')
        }
        self.Wrist = {
            'L': Transform(self.Elbow_to_Wrist['L'], [0., 0., 0., 1.], 'LElbow_V', 'LWrist_V'),
            'R': Transform(self.Elbow_to_Wrist['R'], [0., 0., 0., 1.], 'RElbow_V', 'RWrist_V')
        }
        self.Hand = {
            'L': Transform(self.Wrist_to_Hand['L'], [0., 0., 0., 1.], 'LWrist_V', 'LHand_V'),
            'R': Transform(self.Wrist_to_Hand['R'], [0., 0., 0., 1.], 'RWrist_V', 'RHand_V')
        }

        #======================================================================#
        # Set Hand Origins
        #======================================================================#
        #===== Set Initial Pose =====#
        # If the user provides a dictionary containing joint angles, set that as
        # the initial pose used to get the hand origin positions
        if origin_angles is not None:
            for key in origin_angles.keys():
                self.current_angles[key] = origin_angles[key]
        else:
            #--- Default Origin Angles ---#
            self.current_angles['LShoulderPitch'] = np.pi/2.
            self.current_angles['RShoulderPitch'] = np.pi/2.

        #===== Set Origins =====#
        self.updateTransforms()
        left_transform = self.getHandTransform('L')
        right_transform = self.getHandTransform('R')
        self.hand_origin = {
            'L': left_transform[0:3,3],
            'R': right_transform[0:3,3]
        }

    #==========================================================================#
    # Utility Functions
    #==========================================================================#
    def checkKeys(self, keys):
        """ Checks a list of keys for valid joint names.

        Iterates through the list and removes any joint names that are not
        valid. If an invalid name is provided, that value is removed from the
        list. A list of only valid joint names is returned so that it can be
        subsequently passed to an angle or transform function.

        Parameters
        ----------
        keys : list of str
            A list of the joint names.

        Returns
        -------
        list of str
            The list of keys with invalid joint names removed.
        """
        invalid_keys = []
        for key in keys:
            if key not in self.joint_names:
                keys.remove(key)
                invalid_keys.append(key)

        if (len(invalid_keys) > 0):
            print('[{0}]: Joint name error. The values \'{1}\' are not valid joint names.\nValid joint names: {2}'.format(self.__class__.__name__, invalid_keys, self.joint_names))

        return keys

    def checkSide(self, name, side):
        """ Validates the 'side' parameter.

        Checks to see if 'side' is either 'L' or 'R'. If not, an error is
        printed containing the 'name' parameter to help identify which function
        is checking their parameter.

        Parameters
        ----------
        name : str
            The name to display in the error.
        side : {'L', 'R'}
            A letter indicating the side. It should be 'L' for Left or 'R' for
            Right. If not one of these values, an error is displayed and 'L' is
            returned as default.

        Returns
        -------
        char
            Simply returns 'side' if it is valid or 'L' if not.
        """
        if side not in ['L', 'R']:
            print('[{0}]: Invalid side \'{1}\'. Should be {2}'.format(name, side, ['L', 'R']))
            return 'L'
        else:
            return side

    def broadcastTransforms(self, broadcaster):
        """ Broadcasts the transforms on the '/tf' topic.

        Parameters
        ----------
        broadcaster : tf.TransformBroadcaster
            Broadcaster object used to publish the transforms.
        """
        for side in ['L', 'R']:
            self.Shoulder[side].broadcast(broadcaster)
            self.Elbow[side].broadcast(broadcaster)
            self.Wrist[side].broadcast(broadcaster)
            self.Hand[side].broadcast(broadcaster)

    def getAngles(self):
        """ Returns the current joint angle configuration as a list of floats.

        Returns
        -------
        list of float
            The current_angles values.
        """
        return [self.current_angles[key] for key in self.joint_names]

    def setAngles(self, joint_angles):
        """ Updates current angles contained in 'joint_angles' parameter.

        Reads the 'joint_angles' dictionary and assigns the value to each of the
        joint angles contained in the keys. If a key is not a valid joint name,
        that value is skipped.

        Parameters
        ----------
        joint_angles : dict
            The desired joint angles to set. Each joint angle must have a valid
            joint name as the key in order to be set. Example, joint_angles =
            {'LShoulderPitch': 0.0, 'LShoulderRoll': 0.1, 'LElbowYaw': 0.2,
            'LElbowRoll': 0.3, 'LWristYaw': 0.4, 'RShoulderPitch': 0.5,
            'RShoulderRoll': 0.6, 'RElbowYaw': 0.7, 'RElbowRoll': 0.8,
            'RWristYaw': 0.9}
        """
        # Validate joint names
        valid_joints = self.checkKeys(joint_angles.keys())

        # Set current angles
        for joint in valid_joint:
            self.current_angles[joint] = joint_angles[joint]

    def _setCurrentAngles(self, joint_angles):
        """ Updates current angles contained in 'joint_angles' parameter.

        Reads the 'joint_angles' dictionary and assigns the value to each of the
        joint angles contained in the keys. This is a private member function
        that does not check the angles and assumes the user is using the correct
        values. This is used for increased speed.

        Parameters
        ----------
        joint_angles : dict
            The desired joint angles to set. Each joint angle must have a valid
            joint name as the key in order to be set. Example, joint_angles =
            {'LShoulderPitch': 0.0, 'LShoulderRoll': 0.1, 'LElbowYaw': 0.2,
            'LElbowRoll': 0.3, 'LWristYaw': 0.4, 'RShoulderPitch': 0.5,
            'RShoulderRoll': 0.6, 'RElbowYaw': 0.7, 'RElbowRoll': 0.8,
            'RWristYaw': 0.9}
        """
        # Set current angles
        for joint in joint_angles.keys()
            self.current_angles[joint] = joint_angles[joint]

    def updateTransforms(self, side=None):
        """ Updates arm transforms using the current angle configuration.

        Parameters
        ----------
        side : {None, 'L', 'R'}
            The side to update the transforms for, 'L' = left, 'R' = right. If
            'side' is None, both sides are updated.
        """
        if (side != 'R'):
            self.Shoulder['L'].quaternion = tf.transformations.quaternion_from_euler(0., self.current_angles['LShoulderPitch'], self.current_angles['LShoulderRoll'], 'rxyz')
            self.Elbow['L'].quaternion = tf.transformations.quaternion_from_euler(self.current_angles['LElbowYaw'], 0., self.current_angles['LElbowRoll'], 'rxyz')
            self.Wrist['L'].quaternion = tf.transformations.quaternion_from_euler(self.current_angles['LWristYaw'], 0., 0., 'rxyz')
        if (side != 'L'):
            self.Shoulder['R'].quaternion = tf.transformations.quaternion_from_euler(0, self.current_angles['RShoulderPitch'], self.current_angles['RShoulderRoll'], 'rxyz')
            self.Elbow['R'].quaternion = tf.transformations.quaternion_from_euler(self.current_angles['RElbowYaw'], 0., self.current_angles['RElbowRoll'], 'rxyz')
            self.Wrist['R'].quaternion = tf.transformations.quaternion_from_euler(self.current_angles['RWristYaw'], 0., 0., 'rxyz')

    def setTransforms(self, joint_values, side=None):
        """ Updates the current angles for the arm, then updates the transforms.

        Parameters
        ----------
        joint_values : list of float
            The desired joint angles for the arm in the following order:
            ['ShoulderPitch', 'ShoulderRoll', 'ElbowYaw', 'ElbowRoll',
            'WristYaw']
        side : {'L', 'R'}
            The side to set the transforms for. 'L' = left, 'R' = right, if
            'None', both sides are set.
        """
        if size is None:
            # Validate parameters
            if (len(joint_values != 10)):
                raise ValueError('Invalid number of joint values: \'{0}\'. Must be 10'.format(len(joint_values)))

            # Set the transfroms
            self.setAngles(dict(zip(self.joint_names, joint_values)))
            self.updateTransforms()
        else:
            # Validate parameters
            if (len(joint_values) != 5):
                raise ValueError('Invalid number of joint values: \'{0}\'. Must be 5'.format(len(joint_values)))
            side = self.checkSide(self.__class__.__name__ + '.setTransforms()', side)

            # Set the transforms
            self.setAngles(dict(zip(self.joint_names_side[side], joint_values)))
            self.updateTransforms(side)

    def getHandTransform(self, side):
        """ Calculates the hand transform for a given side.

        Calculates the hand frame ('LHand_V' or 'RHand_V') transform in the
        'base_link_V' frame. Returns it as a 4x4 numpy array. If 'side' is 'L',
        the returned value is for the left hand. For any other value the right
        hand is returned.

        Parameters
        ----------
        side : {'L', 'R'}
            The side for which to obtain the hand pose.

        Returns
        -------
        base_T_hand: 4x4 numpy array
            The transformation matrix representing the pose of 'LHand_V' or
            'RHand_V' in the 'base_link_V' frame.
        """
        side = self.checkSide(self.__class__.__name__ + '.getHandTransform()', side)

        base_T_shoulder = self.Shoulder[side].matrix()
        shoulder_T_elbow = self.Elbow[side].matrix()
        elbow_T_wrist = self.Wrist[side].matrix()
        wrist_T_hand = self.Hand[side].matrix()
        base_T_hand = base_T_shoulder.dot(shoulder_T_elbow.dot(elbow_T_wrist.dot(wrist_T_hand)))
        return base_T_hand
