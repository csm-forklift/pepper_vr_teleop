#!/usr/bin/env python

'''
ROS Node that reads the transforms published by 'convert_transforms' and
calculates the joint angles for Pepper, then publishes them.

Uses the /pepper_robot/pose/joint_angles topic to publish commands
This topic uses a naoqi_bridge_msgs/JointAnglesWithSpeed message.
'''
# TODO:
# make a parameter for "base_link" and change all the instances to that
# variable.

import math
import numpy as np

import rospy
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf


class KinectControl:
    def __init__(self):
        #===== Initialize ROS Objects =====#
        rospy.init_node("kinect_control")

        # Joint Names
        self.joint_names = ['HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
        self.head_names = self.joint_names[0:2]
        self.l_arm_names = self.joint_names[2:6]
        self.r_arm_names = self.joint_names[6:10]

        # Set Motion Parameters
        # Scale factor
        self.fractionMaxArmSpeed = 0.1

        # Publishing frequency
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        # Publishers
        # Head
        self.joint_angles_pub = rospy.Publisher('/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=3)
        self.joint_angles_msg = JointAnglesWithSpeed()
        self.joint_angles_msg.header.seq = 0
        self.joint_angles_msg.speed = self.fractionMaxArmSpeed

        # TransformListener
        self.tfListener = tf.TransformListener()

        # TransformBroadcaster
        self.tfBroadcaster = tf.TransformBroadcaster()

        #===== Derivation Variables =====#
        # used to calculate the setpoints from the transforms
        self.l_shoulder_base_link = [0, 0, 0]
        self.l_shoulder_rotation = [0, 0, 0, 1]
        self.r_shoulder_base_link = [0, 0, 0]
        self.r_shoulder_rotation = [0, 0, 0, 1]
        self.l_elbow_base_link = [0, 0, 0]
        self.r_elbow_base_link = [0, 0, 0]
        self.l_elbow_shoulder_link = [0, 0, 0]
        self.r_elbow_shoulder_link = [0, 0, 0]
        self.l_elbow_rotated = [0, 0, 0]
        self.r_elbow_rotated = [0, 0, 0]
        self.l_hand_base_link = [0, 0, 0]
        self.r_hand_base_link = [0, 0, 0]
        self.l_hand_shoulder_link = [0, 0, 0]
        self.r_hand_shoulder_link = [0, 0, 0]
        self.l_hand_rotated = [0, 0, 0]
        self.r_hand_rotated = [0, 0, 0]
        self.l_theta = 0
        self.l_phi = 0
        self.r_theta = 0
        self.r_phi = 0

        #===== Angle Setpoints =====#
        # Current
        self.angle_setpoints = {}
        for key in self.l_arm_names:
            self.angle_setpoints[key] = 0.0
        for key in self.r_arm_names:
            self.angle_setpoints[key] = 0.0

        # Previous
        self.angle_setpoints_previous = {}
        for key in self.l_arm_names:
            self.angle_setpoints_previous[key] = self.angle_setpoints[key]
        for key in self.r_arm_names:
            self.angle_setpoints_previous[key] = self.angle_setpoints[key]
        self.current_time = rospy.get_time()

        #===== Filtering Parameters =====#
        # These variables are used for filtering the angle changes when the
        # shoulder and elbow are near singularities and for detecting and
        # smoothing out large jumps when the Kinect loses tracking.
        # EWMA = exponentially weighted moving average
        self.l_shoulder_pitch_EWMA = 0
        self.r_shoulder_pitch_EWMA = 0
        self.l_elbow_yaw_EWMA = 0
        self.r_elbow_yaw_EWMA = 0
        self.l_shoulder_learning_rate = 1
        self.r_shoulder_learning_rate = 1
        self.l_elbow_learning_rate = 1
        self.r_elbow_learning_rate = 1
        self.max_learning_rate = 1
        self.min_learning_rate = 0.1
        self.singularity = {
            'LShoulderRoll': np.pi/2,
            'RShoulderRoll': -np.pi/2,
            'LElbowRoll': 0,
            'RElbowRoll': 0
        }
        self.cutoff = {
            'LShoulderRoll': np.pi/4,
            'RShoulderRoll': -np.pi/4,
            'LElbowRoll': -np.pi/4,
            'RElbowRoll': np.pi/4
        }

        # Calculate the linear regression parameters
        self.a = {}
        self.b = {}
        for key in self.singularity.keys():
            self.a[key] = (self.min_learning_rate - self.max_learning_rate) / (self.singularity[key] - self.cutoff[key])
            self.b[key] = ((self.min_learning_rate - self.max_learning_rate)*(self.cutoff[key])) / (self.singularity[key] - self.cutoff[key])
        #================================#

        #===== Large jump detection parameters =====#
        # Angle change to be considered a large jump
        self.large_angle = np.pi/2 # rad
        # Length of averaging window
        # (should be determined based on publishing frequency and how long you want the pose to be stable before going back to tracking)
        self.window_length = 60 # runs at 30Hz, so 30 ~ 1sec
        self.averaging_window = {}
        for key in self.l_arm_names:
            self.averaging_window[key] = self.window_length*[0]
        for key in self.r_arm_names:
            self.averaging_window[key] = self.window_length*[0]
        # Distance the current pose angle must be from the average before tracking again
        # (all joint angles must be within this threshold)
        self.angle_threshold = np.pi/10 # rad

        # Boolean indicating whether the system has experienced a large jump or if it should continue tracking
        self.large_jump = False
        #===========================================#

    def spin(self):
        while not rospy.is_shutdown():
            try:
                self.readTransforms()
                self.calculateAngles()
                self.publishSetPoints()
            except tf.Exception as err:
                print "[{0}]: error reading transforms: {1}".format(rospy.get_name(), err)
            self.rate.sleep()

    def readTransforms(self):
        '''
        Grabs the current positions of the shoulders, elbows, and hands
        '''
        # FIXME: update the hand frames once you add that part to "convert_transforms". Should be "RHand_K" and "LHand_K"
        #----- Right Arm Joints -----#
        self.tfListener.waitForTransform('base_link_K', 'RShoulder_K', rospy.Time(), rospy.Duration.from_sec(1.0))
        self.r_shoulder_base_link, self.r_shoulder_rotation = self.tfListener.lookupTransform('base_link_K', 'RShoulder_K', rospy.Time())
        self.r_elbow_base_link, rotation = self.tfListener.lookupTransform('base_link_K', 'RElbow_K', rospy.Time())
        self.r_hand_base_link, rotation = self.tfListener.lookupTransform('base_link_K', 'RHand_K', rospy.Time())

        #----- Left Arm Joints -----#
        self.l_shoulder_base_link, self.l_shoulder_rotation = self.tfListener.lookupTransform('base_link_K', 'LShoulder_K', rospy.Time())
        self.l_elbow_base_link, rotation = self.tfListener.lookupTransform('base_link_K', 'LElbow_K', rospy.Time())
        self.l_hand_base_link, rotation = self.tfListener.lookupTransform('base_link_K', 'LHand_K', rospy.Time())

    def calculateAngles(self):
        '''
        Calculates the angle setpoints based on current joint positions.
        '''
        #===== Left Arm =====#
        #--- LShoulderPitch
        self.angle_setpoints['LShoulderPitch'] = -math.atan2(self.l_elbow_base_link[2] - self.l_shoulder_base_link[2], self.l_elbow_base_link[0] - self.l_shoulder_base_link[0])

        #--- LShoulderRoll
        self.angle_setpoints['LShoulderRoll'] = math.atan2(self.l_elbow_base_link[1] - self.l_shoulder_base_link[1], math.sqrt((self.l_elbow_base_link[0] - self.l_shoulder_base_link[0])**2 + (self.l_elbow_base_link[2] - self.l_shoulder_base_link[2])**2))

        #--- LElbowYaw
        # get shoulder transform as matrix
        # Use the calculated pitch and roll (which is actually "yaw") rather than the transform from tfListener.
        self.l_shoulder_rotation = tf.transformations.quaternion_from_euler(0, self.angle_setpoints['LShoulderPitch'], self.angle_setpoints['LShoulderRoll'], 'rxyz')
        shoulder_transform = self.tfListener.fromTranslationRotation(self.l_shoulder_base_link, self.l_shoulder_rotation)
        shoulder_T_base_link = np.linalg.inv(shoulder_transform)

        # DEBUG: for visual debugging, transform is not used
        #self.tfBroadcaster.sendTransform(shoulder_transform[0:3,3], tf.transformations.quaternion_from_matrix(shoulder_transform), rospy.Time.now(), 'LShoulderNew_K', 'base_link_K')

        # Update the hand and elbow points to obtain their values in the rotated shoulder frame
        self.l_elbow_shoulder_link = shoulder_T_base_link.dot(np.append(self.l_elbow_base_link, 1))
        self.l_hand_shoulder_link = shoulder_T_base_link.dot(np.append(self.l_hand_base_link, 1))

        self.l_theta = math.atan2(self.l_hand_shoulder_link[2] - self.l_elbow_shoulder_link[2], self.l_hand_shoulder_link[1] - self.l_elbow_shoulder_link[1])
        self.angle_setpoints['LElbowYaw'] = -math.pi + self.l_theta

        #--- LElbowRoll
        # apply Yaw rotation
        R = tf.transformations.euler_matrix(self.l_theta, 0, 0, 'rxyz')
        shoulder_rotated = shoulder_transform.dot(R)
        shoulder_rotated_T_base_link = np.linalg.inv(shoulder_rotated)

        # DEBUG: for visual debugging, transform is not used
        #self.tfBroadcaster.sendTransform(shoulder_rotated[0:3,3], tf.transformations.quaternion_from_matrix(shoulder_rotated), rospy.Time.now(), 'LShoulderNewRot_K', 'base_link_K')

        # find new elbow and hand points
        self.l_elbow_rotated = shoulder_rotated_T_base_link.dot(np.append(self.l_elbow_base_link, 1))
        self.l_hand_rotated = shoulder_rotated_T_base_link.dot(np.append(self.l_hand_base_link, 1))
        self.l_phi = math.atan2(self.l_hand_rotated[1] - self.l_elbow_rotated[1], self.l_hand_rotated[0] - self.l_elbow_rotated[0])
        self.angle_setpoints['LElbowRoll'] = -self.l_phi

        # DEBUG: print out angles in degrees for troubleshooting
        #print("Left Angles:\n\tshoulder_pitch: %f\n\tshoulder_roll: %f\n\telbow_yaw: %f\n\telbow_roll: %f" % (self.angle_setpoints['LShoulderPitch']*(180/math.pi), self.angle_setpoints['LShoulderRoll']*(180/math.pi), self.angle_setpoints['LElbowYaw']*(180/math.pi), self.angle_setpoints['LElbowRoll']*(180/math.pi)))

        #===== Right Arm =====#
        #--- RShoulderPitch
        self.angle_setpoints['RShoulderPitch'] = -math.atan2(self.r_elbow_base_link[2] - self.r_shoulder_base_link[2], self.r_elbow_base_link[0] - self.r_shoulder_base_link[0])

        #--- RShoulderRoll
        self.angle_setpoints['RShoulderRoll'] = math.atan2(self.r_elbow_base_link[1] - self.r_shoulder_base_link[1], math.sqrt((self.r_elbow_base_link[0] - self.r_shoulder_base_link[0])**2 + (self.r_elbow_base_link[2] - self.r_shoulder_base_link[2])**2))

        #--- RElbowYaw
        # get shoulder transform as matrix
        # Use the calculated pitch and roll (which is actually "yaw") rather than the transform from tfListener.
        self.r_shoulder_rotation = tf.transformations.quaternion_from_euler(0, self.angle_setpoints['RShoulderPitch'], self.angle_setpoints['RShoulderRoll'], 'rxyz')
        shoulder_transform = self.tfListener.fromTranslationRotation(self.r_shoulder_base_link, self.r_shoulder_rotation)
        shoulder_T_base_link = np.linalg.inv(shoulder_transform)

        # DEBUG: for visual debugging, transform is not used
        #self.tfBroadcaster.sendTransform(shoulder_transform[0:3,3], tf.transformations.quaternion_from_matrix(shoulder_transform), rospy.Time.now(), 'RShoulderNew_K', 'base_link_K')

        # Update the hand and elbow points to obtain their values in the rotated shoulder frame
        self.r_elbow_shoulder_link = shoulder_T_base_link.dot(np.append(self.r_elbow_base_link, 1))
        self.r_hand_shoulder_link = shoulder_T_base_link.dot(np.append(self.r_hand_base_link, 1))

        self.r_theta = math.atan2(self.r_hand_shoulder_link[2] - self.r_elbow_shoulder_link[2], self.r_hand_shoulder_link[1] - self.r_elbow_shoulder_link[1])
        self.angle_setpoints['RElbowYaw'] = self.r_theta

        #--- RElbowRoll
        # apply Yaw rotation
        R = tf.transformations.euler_matrix(self.r_theta, 0, 0, 'rxyz')
        shoulder_rotated = shoulder_transform.dot(R)
        shoulder_rotated_T_base_link = np.linalg.inv(shoulder_rotated)

        # DEBUG: for visual debugging, transform is not used
        #self.tfBroadcaster.sendTransform(shoulder_rotated[0:3,3], tf.transformations.quaternion_from_matrix(shoulder_rotated), rospy.Time.now(), 'RShoulderNewRot_K', 'base_link_K')

        # find new elbow and hand points
        self.r_elbow_rotated = shoulder_rotated_T_base_link.dot(np.append(self.r_elbow_base_link, 1))
        self.r_hand_rotated = shoulder_rotated_T_base_link.dot(np.append(self.r_hand_base_link, 1))
        self.r_phi = math.atan2(self.r_hand_rotated[1] - self.r_elbow_rotated[1], self.r_hand_rotated[0] - self.r_elbow_rotated[0])
        self.angle_setpoints['RElbowRoll'] = self.r_phi

        # DEBUG: print out angles in degrees for troubleshooting
        #print("Right Angles:\n\tshoulder_pitch: %f\n\tshoulder_roll: %f\n\telbow_yaw: %f\n\telbow_roll: %f" % (self.angle_setpoints['RShoulderPitch']*(180/math.pi), self.angle_setpoints['RShoulderRoll']*(180/math.pi), self.angle_setpoints['RElbowYaw']*(180/math.pi), self.angle_setpoints['RElbowRoll']*(180/math.pi)))

        #===== Filter angle when close to a singularity =====#
        # Calculate the learning rates
        # (the 'roll' angles determine the singularity, then the shoulder pitch
        # and elbow yaw are the angles that must be averaged)
        self.l_shoulder_learning_rate = self.calculateLearningRate(self.angle_setpoints['LShoulderRoll'], 'LShoulderRoll')
        self.r_shoulder_learning_rate = self.calculateLearningRate(self.angle_setpoints['RShoulderRoll'], 'RShoulderRoll')
        self.l_elbow_learning_rate = self.calculateLearningRate(self.angle_setpoints['LElbowRoll'], 'LElbowRoll')
        self.r_elbow_learning_rate = self.calculateLearningRate(self.angle_setpoints['RElbowRoll'], 'RElbowRoll')

        # Calculate the EWMA
        self.l_shoulder_pitch_EWMA = self.l_shoulder_learning_rate * self.angle_setpoints['LShoulderPitch'] + (1 - self.l_shoulder_learning_rate) * self.angle_setpoints_previous['LShoulderPitch']
        self.r_shoulder_pitch_EWMA = self.r_shoulder_learning_rate * self.angle_setpoints['RShoulderPitch'] + (1 - self.r_shoulder_learning_rate) * self.angle_setpoints_previous['RShoulderPitch']
        self.l_elbow_yaw_EWMA = self.l_elbow_learning_rate * self.angle_setpoints['LElbowYaw'] + (1 - self.l_elbow_learning_rate) * self.angle_setpoints_previous['LElbowYaw']
        self.r_elbow_yaw_EWMA = self.r_elbow_learning_rate * self.angle_setpoints['RElbowYaw'] + (1 - self.r_elbow_learning_rate) * self.angle_setpoints_previous['RElbowYaw']

        self.angle_setpoints['LShoulderPitch'] = self.l_shoulder_pitch_EWMA
        self.angle_setpoints['RShoulderPitch'] = self.r_shoulder_pitch_EWMA
        self.angle_setpoints['LElbowYaw'] = self.l_elbow_yaw_EWMA
        self.angle_setpoints['RElbowYaw'] = self.r_elbow_yaw_EWMA
        #====================================================#

        #===== Detect large jumps =====#
        # Add new point to the averaging window
        for key in self.l_arm_names + self.r_arm_names:
            self.averaging_window[key].pop(0)
            self.averaging_window[key].append(self.angle_setpoints[key])

        # Activate "large jump" mode where you must wait until the average settles down before tracking again
        if self.large_jump:
            # Calculate the average
            averages = {}
            for key in self.l_arm_names + self.r_arm_names:
                averages[key] = sum(self.averaging_window[key])/len(self.averaging_window[key])

            # Find if any angles are outside the threshold
            all_angles_within_threshold = True
            for key in self.l_arm_names + self.r_arm_names:
                if (abs(averages[key] - self.angle_setpoints[key]) > self.angle_threshold):
                    all_angles_within_threshold = False
                    break

            # Define setpoints
            if all_angles_within_threshold:
                self.large_jump = False
                # Leave the current setpoints unchanged
            else:
                # Pose has not stabilized, keep setpoints at previous angles
                for key in self.l_arm_names + self.r_arm_names:
                    self.angle_setpoints[key] = self.angle_setpoints_previous[key]
        # Normal tracking operation
        else:
            for key in self.l_arm_names + self.r_arm_names:
                if (abs(self.angle_setpoints[key] - self.angle_setpoints_previous[key]) > self.large_angle):
                    self.large_jump = True
                    for key in self.l_arm_names + self.r_arm_names:
                        self.angle_setpoints[key] = self.angle_setpoints_previous[key]
                    break
        #==============================#

    def publishSetPoints(self):
        '''
        Uses the setpoints to generate the joint angles message and then
        publishes it to Pepper.
        '''
        # Update message
        self.joint_angles_msg.joint_names = self.l_arm_names + self.r_arm_names
        self.joint_angles_msg.joint_angles = [self.angle_setpoints[key] for key in self.l_arm_names + self.r_arm_names]

        # Publish
        self.joint_angles_pub.publish(self.joint_angles_msg)

        # Update previous setpoint
        for key in self.l_arm_names + self.r_arm_names:
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
