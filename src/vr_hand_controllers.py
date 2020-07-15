#!/usr/bin/env python
'''
Uses the Vive VR joystick controllers to set the joint angles. It uses the
controller orientation and distance between the two controllers to determine
where Pepper's hands should be placed.
'''

import math

import rospy
import tf


class Transform():
    def __init__(self, position, quaternion, parent, child):
        '''
        position: 3 element list of [x,y,z] position of transform origin
        quaternion: 4 element list of [x,y,z,w] quaternion representing frame
                    rotation
        parent: name of the parent frame
        child: name of the child frame
        '''
        if (not len(position) == 3):
            raise TypeError('Transform.position must be a 3 element list')
        if (not len(quaternion) == 4):
            raise TypeError('Transform.quaternion must be a 4 element list')
        # TODO: should check that the values are valid floats/integers and scale the quaternion to a magnitude of 1 if it is not.
        self.position = position
        self.quaternion = quaternion
        self.parent = parent
        self.child = child

class VRController():
    def __init__(self):
        #===== Start ROS =====#
        rospy.init_node('vr_hand_controllers')
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        #----- Pepper Description
        self.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        self.angle_setpoints = {}
        for key in self.joint_names:
            self.angle_setpoints[key] = 0.0
        # Offsets
        # All values are in meters, tables of link offsets is found at:
        #    doc.aldebaran.com/2-5/family/pepper_technical/links_pep.html
        # The 'torso' link is called 'base_link_V' in this code
        # Left Arm
        self.base_link_V_to_LShoulder = [-0.057, 0.14974, 0.08682]
        self.LShoulder_to_LElbow = [0.1812, 0.015, 0.00013]
        # The website has a Y offset of 0.0236, but the simulation model has 0
        # THe elbow joint has a slight incline which gives a Z offset in the 0 position, but when considering the Wrist frame in the Elbow frame, this is just a straightline distance along the X axis. The length is found from the distance formula using the X and Z lengths from the documentation.
        self.LElbow_to_LWrist = [0.1517, 0.0, 0.0]
        self.LWrist_to_LHand = [0.0695, 0.0, -0.03030]
        # Right Arm
        self.base_link_V_to_RShoulder = [-0.057, -0.14974, 0.08682]
        self.RShoulder_to_RElbow = [0.1812, -0.015, 0.00013]
        self.RElbow_to_RWrist = [0.1517, 0.0, 0.0]
        self.RWrist_to_RHand = [0.0695, 0.0, -0.03030]
        # The fixed frame that will be connected to "base_link"
        self.fixed_frame = rospy.get_param('~fixed_frame', 'world')
        self.base_link = Transform([0, 0, 0.82], [0, 0, 0, 1], self.fixed_frame, 'base_link_V')
        self.LShoulder = Transform(self.base_link_V_to_LShoulder, [0, 0, 0, 1], 'base_link_V', 'LShoulder_V')
        self.LElbow = Transform(self.LShoulder_to_LElbow, [0, 0, 0, 1], 'LShoulder_V', 'LElbow_V')
        self.LWrist = Transform(self.LElbow_to_LWrist, [0, 0, 0, 1], 'LElbow_V', 'LWrist_V')
        self.LHand = Transform(self.LWrist_to_LHand, [0, 0, 0, 1], 'LWrist_V', 'LHand_V')
        self.RShoulder = Transform(self.base_link_V_to_RShoulder, [0, 0, 0, 1], 'base_link_V', 'RShoulder_V')
        self.RElbow = Transform(self.RShoulder_to_RElbow, [0, 0, 0, 1], 'RShoulder_V', 'RElbow_V')
        self.RWrist = Transform(self.RElbow_to_RWrist, [0, 0, 0, 1], 'RElbow_V', 'RWrist_V')
        self.RHand = Transform(self.RWrist_to_RHand, [0, 0, 0, 1], 'RWrist_V', 'RHand_V')

        #----- TF for Listening and Broadcasting
        #--- Rotated World Transform
        # Check if 'headset_control' has set a yaw offset parameter
        # If not set one.
        self.yaw_offset = rospy.get_param('/headset_control/yaw_offset', None)
        if self.yaw_offset is None:
            rospy.loginfo('[{0}]: No yaw_offset parameter found at "/headset_control/yaw_offset". Using value set at "{0}/yaw_offset"'.format(rospy.get_name()))
            self.yaw_offset = rospy.get_param('~yaw_offset', 0.0)
        rospy.loginfo('[{0}]: yaw_offset: {1}'.format(rospy.get_name(), self.yaw_offset))
        # This rotation is applied to the controllers to rotate them about the world frame to be oriented such that the operator's forward position is directly along the X axis of the world. This new frame is published as 'world_rotated'
        self.world_rotated_position = [0, 0, 0]
        self.z_offset_quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw_offset, 'rxyz')
        self.world_rotated = Transform(self.world_rotated_position, self.z_offset_quaternion, self.fixed_frame, 'world_rotated')

        #--- Controller Transforms
        # Name of left controller frame
        self.left_name = rospy.get_param('~left_name', 'left_controller')
        # Name of right controller frame
        self.right_name = rospy.get_param('~right_name', 'right_controller')
        self.left_controller = Transform([0, 0, 0], [0, 0, 0, 1], 'world_rotated', 'LHand_C')
        self.right_controller = Transform([0, 0, 0], [0, 0, 0, 1], 'world_rotated', 'RHand_C')
        # Multiply the controller rotation by this rotation to adjust it to the
        # correct frame.
        # The initial orientation for the controllers are x: down, y: right, z:
        # backwards. Needed orientation is x: forward, y: left, z: up.
        self.controller_rotation = tf.transformations.quaternion_from_euler(0, -math.pi/2, math.pi, 'rxyz')

        self.tfListener = tf.TransformListener()
        self.tfBroadcaster = tf.TransformBroadcaster()

        # Publishers

        # Subscribers

        # DEBUG: Test pose for controllers
        self.left_test_pose = Transform([0.8, -0.3, 1.5], [0.771698, 0.3556943, -0.5155794, 0.1101889], self.fixed_frame, self.left_name)
        self.right_test_pose = Transform([0.7, 0.3, 1.2], [-0.7427013, 0.3067963, 0.5663613, 0.1830457], self.fixed_frame, self.right_name)

    def spin(self):
        # DEBUG: set joint angles for testing
        self.angle_setpoints['LShoulderPitch'] = 1.0
        self.angle_setpoints['LShoulderRoll'] = 1.0
        self.angle_setpoints['LElbowYaw'] = -0.5
        self.angle_setpoints['LElbowRoll'] = -math.pi/2
        self.angle_setpoints['LWristYaw'] = 0
        self.angle_setpoints['RShoulderPitch'] = 0
        self.angle_setpoints['RShoulderRoll'] = 0
        self.angle_setpoints['RElbowYaw'] = 0
        self.angle_setpoints['RElbowRoll'] = 0
        self.angle_setpoints['RWristYaw'] = -0.5
        self.calculateTransforms()
        while not rospy.is_shutdown():
            self.readTransforms()
            # DEBUG: publish transforms for visualization
            self.publishTransforms()

            self.sendTransform(self.world_rotated)
            self.rate.sleep()

    def readTransforms(self):
        '''
        Reads the current states of the controllers.
        '''
        # Read the controller transforms
        # DEBUG: Test pose values
        left_position = self.left_test_pose.position
        left_quaternion = self.left_test_pose.quaternion
        right_position = self.right_test_pose.position
        right_quaternion = self.right_test_pose.quaternion

        # Convert to proper orientation (X: forward, Y: left, Z: up)
        self.left_controller.position = left_position
        self.left_controller.quaternion = tf.transformations.quaternion_multiply(left_quaternion, self.controller_rotation)
        self.right_controller.position = right_position
        self.right_controller.quaternion = tf.transformations.quaternion_multiply(right_quaternion, self.controller_rotation)

    def calculateTransforms(self):
        '''
        Calculates the transforms for Pepper's joints given a set of joint
        angles.
        '''
        #----- Left Arm -----#
        self.LShoulder.quaternion = tf.transformations.quaternion_from_euler(0, self.angle_setpoints['LShoulderPitch'], self.angle_setpoints['LShoulderRoll'], 'rxyz')
        self.LElbow.quaternion = tf.transformations.quaternion_from_euler(self.angle_setpoints['LElbowYaw'], 0, self.angle_setpoints['LElbowRoll'], 'rxyz')
        self.LWrist.quaternion = tf.transformations.quaternion_from_euler(self.angle_setpoints['LWristYaw'], 0, 0, 'rxyz')

        #----- Right Arm -----#
        self.RShoulder.quaternion = tf.transformations.quaternion_from_euler(0, self.angle_setpoints['RShoulderPitch'], self.angle_setpoints['RShoulderRoll'], 'rxyz')
        self.RElbow.quaternion = tf.transformations.quaternion_from_euler(self.angle_setpoints['RElbowYaw'], 0, self.angle_setpoints['RElbowRoll'], 'rxyz')
        self.RWrist.quaternion = tf.transformations.quaternion_from_euler(self.angle_setpoints['RWristYaw'], 0, 0, 'rxyz')

    def publishTransforms(self):
        '''
        Pulishes the transforms for Pepper's joints and through
        the /tf topic. This is used for debugging purposes.
        '''
        self.sendTransform(self.base_link)

        #----- Left Arm -----#
        self.sendTransform(self.LShoulder)
        self.sendTransform(self.LElbow)
        self.sendTransform(self.LWrist)
        self.sendTransform(self.LHand)

        #----- Right Arm -----#
        self.sendTransform(self.RShoulder)
        self.sendTransform(self.RElbow)
        self.sendTransform(self.RWrist)
        self.sendTransform(self.RHand)

        #----- Controllers -----#
        self.sendTransform(self.left_controller)
        self.sendTransform(self.right_controller)

        # DEBUG: test poses
        self.sendTransform(self.left_test_pose)
        self.sendTransform(self.right_test_pose)

    def sendTransform(self, transform):
        '''
        Uses the tf Broadcaster to send a transform contained in a "Transform"
        object

        Args
        ----
        transform: (Transform), the transform to publish

        Returns
        -------
        None
        '''
        self.tfBroadcaster.sendTransform(transform.position, transform.quaternion, rospy.Time.now(), transform.child, transform.parent)

if __name__ == '__main__':
    try:
        vr_controller = VRController()
        vr_controller.spin()
    except rospy.ROSInterruptException:
        pass
