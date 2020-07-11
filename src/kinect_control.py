#!/usr/bin/env python

'''
ROS Node that reads the transforms published by 'convert_transforms' and
calculates the joint angles for Pepper, then publishes them.
'''
# TODO:
# make a parameter for "base_link" and change all the instances to that
# variable.

import math
import numpy as np

import rospy
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import JointState
import tf


class KinectControl:
    def __init__(self):
        #===== Initialize ROS Objects =====#
        rospy.init_node('kinect_control')

        # Joint Names
        self.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']

        # Set Motion Parameters
        # Scale factor
        self.fraction_max_arm_speed = rospy.get_param('~speed_fraction', 0.1)

        # Publishing frequency
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        # Publishers
        self.joint_angles_msg = JointAnglesWithSpeed()
        self.joint_angles_msg.joint_names = self.joint_names
        self.joint_angles_msg.speed = self.fraction_max_arm_speed
        self.joint_angles_pub = rospy.Publisher('/pepper_interface/joint_angles', JointAnglesWithSpeed, queue_size=3)

        # TransformListener
        self.tfListener = tf.TransformListener()

        # TransformBroadcaster
        self.tfBroadcaster = tf.TransformBroadcaster()

        #===== Derivation Variables =====#
        # used to calculate the setpoints from the transforms
        self.shoulder_base_link = {'L': [0, 0, 0], 'R': [0, 0, 0]}
        self.shoulder_rotation = {'L': [0, 0, 0, 1], 'R': [0, 0, 0, 1]}
        self.elbow_base_link = {'L': [0, 0, 0], 'R': [0, 0, 0]}
        self.elbow_shoulder_link = {'L': [0, 0, 0], 'R': [0, 0, 0]}
        self.elbow_rotated = {'L': [0, 0, 0], 'R': [0, 0, 0]}
        self.hand_base_link = {'L': [0, 0, 0], 'R': [0, 0, 0]}
        self.hand_shoulder_link = {'L': [0, 0, 0], 'R': [0, 0, 0]}
        self.hand_rotated = {'L': [0, 0, 0], 'R': [0, 0, 0]}
        self.theta = {'L': 0, 'R': 0}

        #===== Angle Setpoints =====#
        # Current
        self.angle_setpoints = {}
        for key in self.joint_names:
            self.angle_setpoints[key] = 0.0

        # Previous
        self.angle_setpoints_previous = {}
        for key in self.joint_names:
            self.angle_setpoints_previous[key] = self.angle_setpoints[key]
        self.current_time = rospy.get_time()

        #===== Filtering Parameters =====#
        # These variables are used for filtering the angle changes when the
        # shoulder and elbow are near singularities and for detecting and
        # smoothing out large jumps when the Kinect loses tracking.
        # EWMA = exponentially weighted moving average
        self.shoulder_pitch_EWMA = {'L': 0, 'R': 0}
        self.elbow_yaw_EWMA = {'L': 0, 'R': 0}
        self.shoulder_learning_rate = {'L': 1, 'R': 1}
        self.elbow_learning_rate = {'L': 1, 'R': 1}
        self.max_learning_rate = 1
        self.min_learning_rate = 0.1
        self.singularity = {
            'LShoulderRoll': np.pi/2,
            'RShoulderRoll': -np.pi/2,
            'LElbowRoll': 0,
            'RElbowRoll': 0
        }
        # The cutoff is the angle where the learning factor becomes 1 and all
        # new angles are used as the setpoint without any averaging.
        self.cutoff = {
            'LShoulderRoll': 3*np.pi/8,
            'RShoulderRoll': -3*np.pi/8,
            'LElbowRoll': -np.pi/8,
            'RElbowRoll': np.pi/8
        }

        # Calculate the linear regression parameters
        self.a = {}
        self.b = {}
        for key in self.singularity.keys():
            self.a[key] = (self.min_learning_rate - self.max_learning_rate) / (self.singularity[key] - self.cutoff[key])
            self.b[key] = self.max_learning_rate - self.a[key]*self.cutoff[key]
        #================================#

        #===== Large jump detection parameters =====#
        # Angle change to be considered a large jump
        self.large_angle = 3*np.pi/4 # rad
        # Length of averaging window
        # (should be determined based on publishing frequency and how long you want the pose to be stable before going back to tracking)
        self.window_length = 40 # runs at 30Hz, so 30 ~ 1sec
        self.averaging_window = {}
        for key in self.joint_names:
            self.averaging_window[key] = self.window_length*[0]
        # Distance the current pose angle must be from the average before tracking again
        # (all joint angles must be within this threshold)
        self.angle_threshold = np.pi/10 # rad

        # Boolean indicating whether the system has experienced a large jump or if it should continue tracking
        self.large_jump = False
        #===========================================#

        #===== Define Max Angle Change =====#
        # This is the maximum change allowed per loop in order to smooth out
        # large jumps in the setpoint.
        self.max_angle_change = rospy.get_param('~max_angle_change', np.pi/6)
        #===================================#

    def spin(self):
        while not rospy.is_shutdown():
            try:
                self.readTransforms()
                self.calculateAngles()
                self.publishCommand()
            except tf.Exception as err:
                print "[{0}]: error reading transforms: {1}".format(rospy.get_name(), err)
            self.rate.sleep()

    def readTransforms(self):
        '''
        Grabs the current positions of the shoulders, elbows, and hands
        '''
        for side in  ['L', 'R']:
            self.tfListener.waitForTransform('base_link_K', side + 'Shoulder_K', rospy.Time(), rospy.Duration.from_sec(1.0))
            self.shoulder_base_link[side], self.shoulder_rotation[side] = self.tfListener.lookupTransform('base_link_K', side + 'Shoulder_K', rospy.Time())
            self.elbow_base_link[side], _ = self.tfListener.lookupTransform('base_link_K', side + 'Elbow_K', rospy.Time())
            self.hand_base_link[side], _ = self.tfListener.lookupTransform('base_link_K', side + 'Hand_K', rospy.Time())

    def calculateAngles(self):
        '''
        Calculates the angle setpoints based on current joint positions.
        '''
        for side in ['L', 'R']:
            #--- ShoulderPitch
            self.angle_setpoints[side + 'ShoulderPitch'] = -math.atan2(self.elbow_base_link[side][2] - self.shoulder_base_link[side][2], self.elbow_base_link[side][0] - self.shoulder_base_link[side][0])

            #--- ShoulderRoll
            self.angle_setpoints[side + 'ShoulderRoll'] = math.atan2(self.elbow_base_link[side][1] - self.shoulder_base_link[side][1],
                math.sqrt((self.elbow_base_link[side][0] - self.shoulder_base_link[side][0])**2 + (self.elbow_base_link[side][2] - self.shoulder_base_link[side][2])**2))

            #--- ElbowYaw
            # get shoulder transform as matrix
            # Use the calculated pitch and roll (which is actually "yaw") rather than the transform from tfListener.
            self.shoulder_rotation[side] = tf.transformations.quaternion_from_euler(0, self.angle_setpoints[side + 'ShoulderPitch'], self.angle_setpoints[side + 'ShoulderRoll'], 'rxyz')
            shoulder_transform = self.tfListener.fromTranslationRotation(self.shoulder_base_link[side], self.shoulder_rotation[side])
            shoulder_T_base_link = np.linalg.inv(shoulder_transform)

            # DEBUG: for visual debugging, transform is not used
            #self.tfBroadcaster.sendTransform(shoulder_transform[0:3,3], tf.transformations.quaternion_from_matrix(shoulder_transform), rospy.Time.now(), side + 'ShoulderNew_K', 'base_link_K')

            # Update the hand and elbow points to obtain their values in the rotated shoulder frame
            self.elbow_shoulder_link[side] = shoulder_T_base_link.dot(np.append(self.elbow_base_link[side], 1))
            self.hand_shoulder_link[side] = shoulder_T_base_link.dot(np.append(self.hand_base_link[side], 1))

            self.theta[side] = math.atan2(self.hand_shoulder_link[side][2] - self.elbow_shoulder_link[side][2], self.hand_shoulder_link[side][1] - self.elbow_shoulder_link[side][1])
            if side == 'L':
                self.angle_setpoints[side + 'ElbowYaw'] = -math.pi + self.theta[side]
            else:
                self.angle_setpoints[side + 'ElbowYaw'] = self.theta[side]

            #--- ElbowRoll
            # apply Yaw rotation
            R = tf.transformations.euler_matrix(self.theta[side], 0, 0, 'rxyz')
            shoulder_rotated = shoulder_transform.dot(R)
            shoulder_rotated_T_base_link = np.linalg.inv(shoulder_rotated)

            # DEBUG: for visual debugging, transform is not used
            #self.tfBroadcaster.sendTransform(shoulder_rotated[0:3,3], tf.transformations.quaternion_from_matrix(shoulder_rotated), rospy.Time.now(), side + 'ShoulderNewRot_K', 'base_link_K')

            # find new elbow and hand points
            self.elbow_rotated[side] = shoulder_rotated_T_base_link.dot(np.append(self.elbow_base_link[side], 1))
            self.hand_rotated[side] = shoulder_rotated_T_base_link.dot(np.append(self.hand_base_link[side], 1))
            if side == 'L':
                self.angle_setpoints[side + 'ElbowRoll'] = -math.atan2(self.hand_rotated[side][1] - self.elbow_rotated[side][1], self.hand_rotated[side][0] - self.elbow_rotated[side][0])
            else:
                self.angle_setpoints[side + 'ElbowRoll'] = math.atan2(self.hand_rotated[side][1] - self.elbow_rotated[side][1], self.hand_rotated[side][0] - self.elbow_rotated[side][0])

            # DEBUG: print out angles in degrees for troubleshooting
            #print("#----- Side: %s -----#\n\tshoulder_pitch: %f\n\tshoulder_roll: %f\n\telbow_yaw: %f\n\telbow_roll: %f" % (side, self.angle_setpoints[side + 'ShoulderPitch']*(180/math.pi), self.angle_setpoints[side + 'ShoulderRoll']*(180/math.pi), self.angle_setpoints[side + 'ElbowYaw']*(180/math.pi), self.angle_setpoints[side + 'ElbowRoll']*(180/math.pi)))

        # #===== Filter angle when close to a singularity =====#
        # for side in ['L', 'R']:
        #     # Calculate the learning rates (the 'roll' angles determine the
        #     # singularity, then the shoulder pitch and elbow yaw are the angles
        #     # that must be averaged)
        #     self.shoulder_learning_rate[side] = self.calculateLearningRate(self.angle_setpoints[side + 'ShoulderRoll'], side + 'ShoulderRoll')
        #     self.elbow_learning_rate[side] = self.calculateLearningRate(self.angle_setpoints[side + 'ElbowRoll'], side + 'ElbowRoll')
        #
        #     # Calculate the EWMA
        #     self.shoulder_pitch_EWMA[side] = self.shoulder_learning_rate[side] * self.angle_setpoints[side + 'ShoulderPitch'] + (1 - self.shoulder_learning_rate[side]) * self.angle_setpoints_previous[side + 'ShoulderPitch']
        #     self.elbow_yaw_EWMA[side] = self.elbow_learning_rate[side] * self.angle_setpoints[side + 'ElbowYaw'] + (1 - self.elbow_learning_rate[side]) * self.angle_setpoints_previous[side + 'ElbowYaw']
        #
        #     self.angle_setpoints[side + 'ShoulderPitch'] = self.shoulder_pitch_EWMA[side]
        #     self.angle_setpoints[side + 'ElbowYaw'] = self.elbow_yaw_EWMA[side]
        # #====================================================#

        # #===== Detect large jumps =====#
        # # Add new point to the averaging window
        # for key in self.joint_names:
        #     self.averaging_window[key].pop(0)
        #     self.averaging_window[key].append(self.angle_setpoints[key])
        #
        # # Activate "large jump" mode where you must wait until the average settles down before tracking again
        # if self.large_jump:
        #     # Calculate the average
        #     averages = {}
        #     for key in self.joint_names:
        #         averages[key] = sum(self.averaging_window[key])/len(self.averaging_window[key])
        #
        #     # Find if any angles are outside the threshold
        #     all_angles_within_threshold = True
        #     for key in self.joint_names:
        #         if (abs(averages[key] - self.angle_setpoints[key]) > self.angle_threshold):
        #             all_angles_within_threshold = False
        #             break
        #
        #     # Define setpoints
        #     if all_angles_within_threshold:
        #         self.large_jump = False
        #         # Leave the current setpoints unchanged
        #     else:
        #         # Pose has not stabilized, keep setpoints at previous angles
        #         for key in self.joint_names:
        #             self.angle_setpoints[key] = self.angle_setpoints_previous[key]
        # # Normal tracking operation
        # else:
        #     for key in self.joint_names:
        #         if (abs(self.angle_setpoints[key] - self.angle_setpoints_previous[key]) > self.large_angle):
        #             self.large_jump = True
        #             for key in self.joint_names:
        #                 self.angle_setpoints[key] = self.angle_setpoints_previous[key]
        #             break
        # #==============================#

        # #===== Saturate Large Angle Jumps at a Maximum Change =====#
        # for key in self.joint_names:
        #     angle_difference = self.angle_setpoints[key] - self.angle_setpoints_previous[key]
        #     if abs(angle_difference) > self.max_angle_change:
        #         self.angle_setpoints[key] = self.angle_setpoints_previous[key] + math.copysign(self.max_angle_change, angle_difference)
        # #==========================================================#

    def publishCommand(self):
        '''
        Publishes the joint angle commands for Pepper.
        '''
        self.joint_angles_msg.joint_angles = [self.angle_setpoints[key] for key in self.joint_angles_msg.joint_names]
        self.joint_angles_pub.publish(self.joint_angles_msg)

        # Update previous setpoint
        for key in self.joint_names:
            self.angle_setpoints_previous[key] = self.angle_setpoints[key]

    def calculateLearningRate(self, angle, joint_name):
        '''
        Calculate the learning rate using a simple linear regression found wiih
        the minimum learning rate assigned when the joint is at a singularity
        and increases to the maximum learning rate at the desired cutoff angle.
        '''
        # Calculate the learning rate
        learning_rate = self.a[joint_name] * angle + self.b[joint_name]
        learning_rate = min(self.max_learning_rate, learning_rate)
        learning_rate = max(self.min_learning_rate, learning_rate)
        return learning_rate


if __name__ == "__main__":
    try:
        kinect_control = KinectControl()
        kinect_control.spin()
    except rospy.ROSInterruptException:
        pass
