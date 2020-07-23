#!/usr/bin/env python
'''
Uses the Vive VR joystick controllers to set the joint angles. It uses the
controller orientation and distance between the two controllers to determine
where Pepper's hands should be placed.
'''

import math
from scipy.optimize import minimize
from pepper_transforms import Transform, PepperModel
import numpy as np

import rospy
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import Joy
import tf


class VRController():
    def __init__(self):
        #======================================================================#
        # ROS Setup
        #======================================================================#
        #===== Start Node =====#
        rospy.init_node('vr_hand_controllers')
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        #===== Parameters =====#
        # Ratio of Pepper's arm to human arm
        self.arm_ratio = rospy.get_param('~arm_ratio', 1.0)
        rospy.loginfo('[{0}]: Arm ratio: {1}'.format(rospy.get_name(), self.arm_ratio))
        # Name of left controller frame from vive_ros
        self.left_name = rospy.get_param('~left_name', 'controller_LHR_FFF73D47')
        # Name of right controller frame from vive_ros
        self.right_name = rospy.get_param('~right_name', 'controller_LHR_FFFAFF45')
        # The user must calibrate the yaw offset so that the X axis points in
        # the foward direction. This is the fixed frame to which the
        # 'world_rotated' frame will be attached, but with the X axis poining
        # forward.
        # NOTE: It is assumed this frame is level with the ground.
        self.fixed_frame = rospy.get_param('~fixed_frame', 'world')
        # Sets the joint speed of the arms
        self.fraction_max_arm_speed = rospy.get_param('~speed_fraction', 0.1)
        self.calibration_time = rospy.get_param('~calibration_time', 3.0)
        self.yaw_offset = rospy.get_param('/headset_control/yaw_offset', None)
        if self.yaw_offset is None:
            rospy.loginfo('[{0}]: No yaw_offset parameter found at "/headset_control/yaw_offset". Using value set at "{0}/yaw_offset"'.format(rospy.get_name()))
            self.yaw_offset = rospy.get_param('~yaw_offset', None)
        else:
            # THe headset_control yaw_offset is in the opposite direction as
            # desired here, so the value must be flipped.
            self.yaw_offset *= -1
        # Error weights used in the optimization
        self.position_weight = rospy.get_param('~position_weight', 1.0)
        self.orientation_weight = rospy.get_param('~orientation_weight', 1.0)

        #=====- TF Listener and Broadcaster =====#
        self.tfListener = tf.TransformListener()
        self.tfBroadcaster = tf.TransformBroadcaster()

        #===== Publishers =====#
        self.joint_angles_msg = JointAnglesWithSpeed()
        self.joint_angles_msg.joint_names = self.joint_names
        self.joint_angles_msg.speed = self.fraction_max_arm_speed
        self.joint_angles_pub = rospy.Publisher('/pepper_interface/joint_angles', JointAnglesWithSpeed, queue_size=3)

        #===== Subscribers =====#
        self.left_controller_sub = rospy.Subscriber('/vive/' + self.left_name + '/joy', Joy, self.leftCallback, queue_size=1)
        self.right_controller_sub = rospy.Subscriber('/vive/' + self.right_name + '/joy', Joy, self.rightCallback, queue_size=1)

        #======================================================================#
        # Controller Setup and Calibration
        #======================================================================#
        #===== Controller Transforms =====#
        self.left_controller = Transform([0., 0., 0.], [0., 0., 0., 1.], 'world_rotated', self.left_name)
        self.right_controller = Transform([0., 0., 0.], [0., 0., 0., 1.], 'world_rotated', self.right_name)
        self.left_controller_rotated = Transform([0., 0., 0.], [0., 0., 0., 1.], 'world_rotated', 'LHand_C')
        self.right_controller_rotated = Transform([0., 0., 0.], [0., 0., 0., 1.], 'world_rotated', 'RHand_C')
        # Multiply the controller rotation by this rotation to adjust it to the
        # correct frame.
        # The initial orientation for the controllers are x: right, y: up, z:
        # backwards. Needed orientation is x: forward, y: left, z: up.
        self.controller_rotation = tf.transformations.quaternion_from_euler(0., math.pi/2, -math.pi/2, 'rzyx')

        # DEBUG: Test pose for controllers
        self.left_test_pose = Transform([0.7, 0.3, 1.2], [-0.7427013, 0.3067963, 0.5663613, 0.1830457], 'world_rotated', 'LHand_C')
        self.right_test_pose = Transform([0.8, -0.3, 1.5], [0.771698, 0.3556943, -0.5155794, 0.1101889], 'world_rotated', 'RHand_C')

        #===== Yaw_offset calibration =====#
        # This rotation is applied to the controllers to rotate them about the
        # world frame to be oriented such that the operator's forward position
        # is directly along the X axis of the world. This new frame is published
        # as 'world_rotated'
        self.world_rotated_position = [0., 0., 0.]
        self.world_rotated = Transform(self.world_rotated_position, [0., 0., 0., 1.], self.fixed_frame, 'world_rotated')

        self.running_calibration = False

        # If the 'headset_control' node has not set the yaw_offset parameter and
        # it is not provided as a parameter for this node, then it must be
        # calibrated here.
        if self.yaw_offset is None:
            self.yaw_offset = 0
            rospy.loginfo('[{0}]: No yaw_offset parameter found at {0}/yaw_offset. You will need to perform a calibration.'.format(rospy.get_name()))
            print('\n{0}\n{1}{2}\n{0}'.format(60*'=', 16*' ', 'Orientation Calibration'))
            print('Point both controllers towards the front direction. Press the side button on the LEFT controller and hold for {0} seconds.'.format(self.calibration_time))

            # Wait for user to press the side button and for calibration to
            # complete
            self.orientation_calibration = False
            while not self.orientation_calibration:
                self.rate.sleep()

        # Publish the new 'world_rotated' frame
        self.world_rotated.broadcast(self.tfBroadcaster)

        #===== Hand Origin Calibration =====#
        # Controller Origin
        self.left_controller_origin = [0, 0, 0]
        self.right_controller_origin = [0, 0, 0]
        print('\n{0}\n{1}{2}\n{0}'.format(60*'=', 16*' ', 'Position Calibration'))
        print('Point arms straight towards the ground. Press the side button on the RIGHT controller and hold position for {0} seconds.'.format(self.calibration_time))

        # Wait for user to press the side button and for calibration to complete
        self.position_calibration = False
        while not self.position_calibration:
            self.rate.sleep()

        #======================================================================#
        # Create Pepper Model for Inverse Kinematics
        #======================================================================#
        # Set Joints
        self.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        self.joint_names_left = self.joint_names[0:5]
        self.joint_names_right = self.joint_names[5:10]
        self.angle_setpoints = {}
        for key in self.joint_names:
            self.angle_setpoints[key] = 0.0

        # Set bounds for optimization
        self.bounds_left = [(-2.0857, 2.0857), (0.0087, 1.5620), (-2.0857, 2.0857),  (-1.5620, -0.0087), (-1.8239, 1.8239)]
        self.bounds_right = [(-2.0857, 2.0857), (-1.5620, -0.0087), (-2.0857, 2.0857), (0.0087, 1.5620), (-1.8239, 1.8239)]

        # Pepper Model Object
        self.pepper_model = PepperModel()

    def spin(self):
        while not rospy.is_shutdown():
            # Publish the rotated fixed frame
            self.world_rotated.broadcast(self.tfBroadcaster)

            # Update the controller transforms
            self.readTransforms()

            # Find joint angles using optimization
            self.calculateJointAngles()

            # Publish joint angles to Pepper
            self.publishJointAnglesCommand()

            # DEBUG: publish transforms for visualization
            self.broadcastRotatedTransforms()
            self.pepper_model.broadcastTransform(self.tfBroadcaster)

            self.rate.sleep()

    def calculateJointAngles(self):
        '''
        Calculates the joint angles for the desired position using the
        optimizer.
        '''
        #===== Perform inverse kinematics optimization for left arm =====#
        # Calculate desired setpoint
        left_controller_relative = np.array(self.left_controller_relative.position) - np.array(self.left_controller_origin)
        left_pepper_setpoint = (self.arm_ratio*left_controller_relative) + self.pepper_model.left_hand_origin

        # Set initial condition (current joint configuration)
        x0_left = [self.angle_setpoints[key] for key in self.joint_names_left]

        # Run optimization
        res_left = minimize(self.objective, x0_left, args=(left_pepper_setpoint, 'L'), method='SLSQP', bounds=self.bounds_left)

        #===== Perform inverse kinematics optimization for right arm =====#
        # Calculate desired setpoint
        right_controller_relative = np.array(self.right_controller_relative.position) - np.array(self.right_controller_origin)
        right_pepper_setpoint = (self.arm_ratio*right_controller_relative) + self.pepper_model.right_hand_origin

        # Set initial condition (current joint configuration)
        x0_right = [self.angle_setpoints[key] for key in self.joint_names_right]

        # Run optimization
        res_right = minimize(self.objective, x0_right, args=(right_pepper_setpoint, 'R'), method='SLSQP', bounds=self.bounds_right)

        # Set joint angles
        for i in range(len(self.joint_names_left)):
            self.angle_setpoints[self.joint_names_left[i]] = res_left.x[i]
        for i in range(len(self.joint_names_right)):
            self.angle_setpoints[self.joint_names_right[i]] = res_right.x[i]

    def objective(self, x, setpoint, hand):
        '''
        Objective function for the optimization.

        Parameters
        ----------
        x: list of float
            Array of joint angles in the following order [ShoulderPitch,
            ShoulderRoll, ElbowYaw, ElbowRoll, WristYaw].
        setpoint: numpy array
            The desired [x,y,z] relative position for Pepper's hand.
        hand: {'L', 'R'}
            The side to perform the optimization on.

        Returns
        -------
        cost: float
            The cost of the optimization objective, a weighted sum of errors.
        '''
        #===== Check Inputs =====#
        if not (len(x) == 5):
            raise ValueError('The objective input must be a numpy array of length 5')
        if hand not in ['L', 'R']:
            raise ValueError('Invalid hand type: {0}, must be "L" or "R"'.format(hand))

        position_error = 0.0
        orientation_error = 0.0

        #===== Left Arm =====#
        if hand == 'L':
            # Update hand position
            self.pepper_model.setTransformsLeft(x)
            left_transform = self.pepper_model.getLeftHandTransform()

            # Calculate position error
            left_position = left_transform[0:3,3]
            left_error = setpoint - left_position
            position_error = np.sum(left_error**2)

            # Calculate orientation error
        #===== Right Arm =====#
        else:
            # Update hand position
            self.pepper_model.setTransformsRight(x)
            right_transform = self.pepper_model.getRightHandTransform()

            # Calculate position error
            right_position = right_transform[0:3,3]
            right_error = setpoint - right_position
            position_error = np.sum(right_error**2)

            # Calculate orientation error

        cost = self.position_weight*position_error + self.orientation_weight*orientation_error

        return cost

    def readTransforms(self):
        '''
        Reads the current states of the controllers.
        '''
        if self.left_controller.canTransform():
            # Read the controller transforms
            self.left_controller.listen()
            self.right_controller.listen()

            # Convert to proper orientation (X: forward, Y: left, Z: up)
            self.left_controller_rotated.position = self.left_controller.position
            self.left_controller_rotated.quaternion = tf.transformations.quaternion_multiply(self.left_controller.quaternion, self.controller_rotation)
            self.right_controller_rotated.position = self.right_controller.position
            self.right_controller_rotated.quaternion = tf.transformations.quaternion_multiply(self.right_controller.quaternion, self.controller_rotation)

        #======================================================================#
        # DEBUG: Test pose values
        #======================================================================#
        # self.left_controller.position = self.left_test_pose.position
        # self.left_controller.quaternion = self.left_test_pose.quaternion
        # self.right_controller.position = self.right_test_pose.position
        # self.right_controller.quaternion = self.right_test_pose.quaternion
        #======================================================================#
        # DEBUG: Test pose values
        #======================================================================#

    def broadcastRotatedTransforms(self):
        '''
        Broadcasts the left and right rotated controller frames.
        '''
        self.left_controller_rotated.broadcast(self.tfBroadcaster)
        self.right_controller_rotated.broadcast(self.tfBroadcaster)

    def publishJointAnglesCommand(self):
        '''
        Publishes the current setpoints to the joint angle command topic.
        '''
        self.joint_angles_msg.joint_angles = [self.angle_setpoints[key] for key in self.joint_angles_msg.joint_names]
        self.joint_angles_pub.publish(self.joint_angles_msg)

    def calibrateOrientation(self):
        '''
        Calibrates the 'yaw_offset' angle which indicates the front direction.
        '''
        # Guard to keep from running more than one instance of the calibration
        # functions # at a time
        self.running_calibration = True

        if (self.yaw_offset is None):
            self.yaw_offset = 0
        calibration_average = 0
        calibration_start = rospy.get_time()

        rospy.loginfo('[{0}]: Calibrating orientation for {1} seconds'.format(rospy.get_name(), self.calibration_time))

        #===== Calibration Loop =====#
        n = 0
        while (rospy.get_time() - calibration_start) < self.calibration_time:
            left_position = [0, 0, 0]
            left_quaternion = [0, 0, 0, 1]
            right_position = [0, 0, 0]
            right_quaternion = [0, 0, 0, 1]
            try:
                self.tfListener.waitForTransform(self.fixed_frame, self.left_name, rospy.Time(), rospy.Duration.from_sec(1.0))
                left_position, left_quaternion = self.tfListener.lookupTransform(self.fixed_frame, self.left_name, rospy.Time())
                self.tfListener.waitForTransform(self.fixed_frame, self.right_name, rospy.Time(), rospy.Duration.from_sec(1.0))
                right_position, right_quaternion = self.tfListener.lookupTransform(self.fixed_frame, self.right_name, rospy.Time())
            except tf.Exception as err:
                rospy.logwarn('[{0}]: TF Error: {1}'.format(rospy.get_name(), err))

            # Frames must be rotated so that the X axis is pointing forward
            # with Z axis pointing up
            left_quaternion = tf.transformations.quaternion_multiply(left_quaternion, self.controller_rotation)
            right_quaternion = tf.transformations.quaternion_multiply(right_quaternion, self.controller_rotation)

            left_yaw, left_pitch, left_roll = tf.transformations.euler_from_quaternion(left_quaternion, 'rzyx')
            right_yaw, right_pitch, right_roll = tf.transformations.euler_from_quaternion(right_quaternion, 'rzyx')
            calibration_average = (n*calibration_average + (left_yaw + right_yaw)/2)/(n+1)
            n += 1

        self.yaw_offset = calibration_average
        rospy.loginfo('[{0}]: yaw_offset: {1}'.format(rospy.get_name(), self.yaw_offset))
        self.world_rotated.quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw_offset, 'rxyz')
        self.orientation_calibration = True

        self.running_calibration = False

    def calibratePosition(self):
        '''
        Calibrates the origin positions for the left and right controllers.
        '''
        # Guard to keep from running more than one instance of the calibration
        # functions # at a time
        self.running_calibration = True

        left_controller_average = np.array([0.0, 0.0, 0.0])
        right_controller_average = np.array([0.0, 0.0, 0.0])

        rospy.loginfo('[{0}]: Calibrating positions for {1} seconds'.format(rospy.get_name(), self.calibration_time))
        calibration_start = rospy.get_time()

        #===== Calibration Loop =====#
        n = 0
        while (rospy.get_time() - calibration_start) < self.calibration_time:
            try:
                self.tfListener.waitForTransform('world_rotated', self.left_name, rospy.Time(), rospy.Duration.from_sec(1.0))
                self.left_controller_origin, _ = self.tfListener.lookupTransform('world_rotated', self.left_name, rospy.Time())
                self.tfListener.waitForTransform('world_rotated', self.right_name, rospy.Time(), rospy.Duration.from_sec(1.0))
                self.right_controller_origin, _ = self.tfListener.lookupTransform('world_rotated', self.right_name, rospy.Time())
            except tf.Exception as err:
                rospy.logwarn('[{0}]: TF Error: {1}'.format(rospy.get_name(), err))

            left_controller_average = (n*left_controller_average + self.left_controller_origin)/(n+1)
            right_controller_average = (n*right_controller_average + self.right_controller_origin   q)/(n+1)

            n += 1

        self.left_controller_origin = list(left_controller_average)
        self.right_controller_origin = list(right_controller_average)
        rospy.loginfo('[{0}]: Left controller origin: [{1}, {2}, {3}]'.format(rospy.get_name(), self.left_controller_origin[0], self.left_controller_origin[1], self.left_controller_origin[2]))
        rospy.loginfo('[{0}]: Right controller origin: [{1}, {2}, {3}]'.format(rospy.get_name(), self.right_controller_origin[0], self.right_controller_origin[1], self.right_controller_origin[2]))

        self.position_calibration = True
        self.running_calibration = False

    def leftCallback(self, msg):
        '''
        Reads the left joystick buttons.
        '''
        # Side button: perform orientation calibration
        if msg.buttons[3]:
            if not self.running_calibration:
                self.calibrateOrientation()

    def rightCallback(self, msg):
        '''
        Reads the right joystick buttons.
        '''
        if msg.buttons[3]:
            if not self.running_calibration:
                self.calibratePosition()

if __name__ == '__main__':
    try:
        vr_controller = VRController()
        vr_controller.spin()
    except rospy.ROSInterruptException:
        pass
