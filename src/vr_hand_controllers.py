#!/usr/bin/env python
'''
Uses the Vive VR joystick controllers to set the joint angles. It uses the
controller orientation and distance between the two controllers to determine
where Pepper's hands should be placed.
'''

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
        # Offsets
        # All values are in meters, tables of link offsets is found at:
        #    doc.aldebaran.com/2-5/family/pepper_technical/links_pep.html
        # The 'torso' link is called 'base_link_V' in this code
        # Left Arm
        self.base_link_V_to_LShoulder = [-0.057, 0.14974, 0.08682]
        self.LShoulder_to_LElbow = [0.1812, 0.015, 0.00013]
        # The website has a Y offset of 0.0236, but the simulation model has 0
        self.LElbow_to_LWrist = [0.150, 0.0, 0.02284]
        self.LWrist_to_LHand = [0.0695, 0.0, -0.03030]
        # Right Arm
        self.base_link_V_to_RShoulder = [-0.057, -0.14974, 0.08682]
        self.RShoulder_to_RElbow = [0.1812, -0.015, 0.00013]
        self.RElbow_to_RWrist = [0.150, 0.0, 0.02284]
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
        self.tfListener = tf.TransformListener()
        self.tfBroadcaster = tf.TransformBroadcaster()

        # Publishers

        # Subscribers

    def spin(self):
        while not rospy.is_shutdown():
            # DEBUG: publish transforms for visualization
            self.publishTransforms()
            self.rate.sleep()

    def readTransforms(self):
        '''
        Reads the current states of the controllers.
        '''
        pass

    def calculateTransforms(self, joint_angles):
        '''
        Calculates the transforms for Pepper's joints given a set of joint
        angles.
        '''
        pass

    def publishTransforms(self):
        '''
        Pulishes the transforms for Pepper's joints and through
        the /tf topic. This is used for debugging purposes.
        '''
        self.tfBroadcaster.sendTransform(self.base_link.position, self.base_link.quaternion, rospy.Time.now(), self.base_link.child, self.base_link.parent)
        # Left Arm
        self.tfBroadcaster.sendTransform(self.LShoulder.position, self.LShoulder.quaternion, rospy.Time.now(), self.LShoulder.child, self.LShoulder.parent)
        self.tfBroadcaster.sendTransform(self.LElbow.position, self.LElbow.quaternion, rospy.Time.now(), self.LElbow.child, self.LElbow.parent)
        self.tfBroadcaster.sendTransform(self.LWrist.position, self.LWrist.quaternion, rospy.Time.now(), self.LWrist.child, self.LWrist.parent)
        self.tfBroadcaster.sendTransform(self.LHand.position, self.LHand.quaternion, rospy.Time.now(), self.LHand.child, self.LHand.parent)
        # Right Arm
        self.tfBroadcaster.sendTransform(self.RShoulder.position, self.RShoulder.quaternion, rospy.Time.now(), self.RShoulder.child, self.RShoulder.parent)
        self.tfBroadcaster.sendTransform(self.RElbow.position, self.RElbow.quaternion, rospy.Time.now(), self.RElbow.child, self.RElbow.parent)
        self.tfBroadcaster.sendTransform(self.RWrist.position, self.RWrist.quaternion, rospy.Time.now(), self.RWrist.child, self.RWrist.parent)
        self.tfBroadcaster.sendTransform(self.RHand.position, self.RHand.quaternion, rospy.Time.now(), self.RHand.child, self.RHand.parent)

if __name__ == '__main__':
    try:
        vr_controller = VRController()
        vr_controller.spin()
    except rospy.ROSInterruptException:
        pass
