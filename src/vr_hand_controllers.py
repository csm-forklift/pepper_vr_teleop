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
        self.bounds_left = [(-2.0857, 2.0857), (0.0087, 1.5620), (-2.0857, 2.0857),  (-1.5620, -0.0087)]
        self.bounds_left_wrist = [(-1.8239, 1.8239)]
        self.bounds_right = [(-2.0857, 2.0857), (-1.5620, -0.0087), (-2.0857, 2.0857), (0.0087, 1.5620)]
        self.bounds_right_wrist = [(-1.8239, 1.8239)]

        # These are the standard poses used for starting the optimization
        self.standard_poses = np.array([
            [-self.bounds_left[0][0], 0.0, -np.pi/2.0, -np.pi/4.0, 0.0, -self.bounds_right[0][0], 0.0, np.pi/2.0, np.pi/4.0, 0.0],
            [np.pi/2.0, np.pi/2.0, -np.pi/2.0, -0.3, 0.0, np.pi/2.0, -np.pi/2.0, np.pi/2.0, 0.3, 0.0],
            [0.2, 0.2, -1.0, -0.4, 0.0, 0.2, -0.2, 1.0, 0.4, 0.0]
            ])

        # Pepper Model Object
        self.pepper_model = PepperModel()

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
        self.calibration_time = rospy.get_param('~calibration_time', 1.0)
        self.yaw_offset = rospy.get_param('/headset_control/yaw_offset', None)
        if self.yaw_offset is None:
            rospy.loginfo('[{0}]: No yaw_offset parameter found at "/headset_control/yaw_offset". Using value set at "{0}/yaw_offset"'.format(rospy.get_name()))
            self.yaw_offset = rospy.get_param('~yaw_offset', None)
        else:
            # THe headset_control yaw_offset is in the opposite direction as
            # desired here, so the value must be flipped.
            self.yaw_offset *= -1
        # Error weights used in the optimization
        self.position_weight = rospy.get_param('~position_weight', 100.0)
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
        self.left_controller_origin = [0., 0., 0.]
        self.right_controller_origin = [0., 0., 0.]
        print('\n{0}\n{1}{2}\n{0}'.format(60*'=', 16*' ', 'Position Calibration'))
        print('Point arms straight towards the ground. Press the side button on the RIGHT controller and hold position for {0} seconds.'.format(self.calibration_time))

        # Wait for user to press the side button and for calibration to complete
        self.position_calibration = False
        while not self.position_calibration:
            self.rate.sleep()

        #======================================================================#
        # Debugging Frames
        #======================================================================#
        # Transform to link Pepper's base_link to the world frame
        self.base_link = Transform([0., 0., 0.82], [0., 0., 0., 1.], 'world_rotated', 'base_link_V')

        # DEBUG: Test pose for controllers
        self.left_test_pose = Transform([0.7, 0.3, 1.2], [-0.7427013, 0.3067963, 0.5663613, 0.1830457], 'world_rotated', 'LHand_C')
        self.right_test_pose = Transform([0.8, -0.3, 1.5], [0.771698, 0.3556943, -0.5155794, 0.1101889], 'world_rotated', 'RHand_C')

        # Display the origin and setpoint for human and pepper
        self.pepper_left_origin = Transform(self.pepper_model.left_hand_origin, [0., 0., 0., 1.], 'base_link_V', 'LOrigin_P')
        self.pepper_right_origin = Transform(self.pepper_model.right_hand_origin, [0., 0., 0., 1.], 'base_link_V', 'ROrigin_P')
        self.pepper_left_setpoint = Transform([0., 0., 0.], [0., 0., 0., 1.], 'base_link_V', 'LSetpoint_P')
        self.pepper_right_setpoint = Transform([0., 0., 0.], [0., 0., 0., 1.], 'base_link_V', 'RSetpoint_P')
        self.human_left_origin = Transform(self.left_controller_origin, [0., 0., 0., 1.], 'world_rotated', 'LOrigin_H')
        self.human_right_origin = Transform(self.right_controller_origin, [0., 0., 0., 1.], 'world_rotated', 'ROrigin_H')

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
            self.broadcastDebugTransforms()
            self.pepper_model.broadcastTransforms(self.tfBroadcaster)

            self.rate.sleep()

    def speedTest(self):
        '''
        Runs the optimization multiple times and finds and average speed.
        '''
        n = 100
        x = np.zeros(n+1)
        x[0] = rospy.get_time()
        for i in range(1,n+1):
            self.left_controller_rotated.position = list(np.random.rand(3))
            self.right_controller_rotated.position = list(np.random.rand(3))
            self.calculateJointAngles()
            x[i] = rospy.get_time()

        sum_time = 0.0
        for i in range(1,n+1):
            sum_time = sum_time + (x[i] - x[i-1])
        avg_time = sum_time / n

        print('Ran optimization {0} times.\n Average speed: {1:0.6f} sec'.format(n, avg_time))

    def poseTest(self):
        '''
        Puts the robot model in a specificy pose to test what the joint angles
        will produce.
        '''
        # Set joint angles
        self.angle_setpoints['LShoulderPitch'] = 0.2
        self.angle_setpoints['LShoulderRoll'] = 0.2
        self.angle_setpoints['LElbowYaw'] = -1.0
        self.angle_setpoints['LElbowRoll'] = -0.4
        self.angle_setpoints['LWristYaw'] = 0.0
        self.angle_setpoints['RShoulderPitch'] = 0.2
        self.angle_setpoints['RShoulderRoll'] = -0.2
        self.angle_setpoints['RElbowYaw'] = 1.0
        self.angle_setpoints['RElbowRoll'] = 0.4
        self.angle_setpoints['RWristYaw'] = 0.0

        # Set Pepper model angles
        self.pepper_model.setTransforms([self.angle_setpoints[key] for key in self.joint_names])
        self.pepper_model.broadcastTransforms(self.tfBroadcaster)

    def calculateJointAngles(self):
        '''
        Calculates the joint angles for the desired position using the
        optimizer.
        '''
        #===== Optimization Paramters =====#
        opt_method = 'SLSQP'

        #self.initial_poses = np.append(self.standard_poses, [np.array([self.angle_setpoints[key] for key in self.joint_names])], axis=0)
        self.initial_poses = np.array([np.array([self.angle_setpoints[key] for key in self.joint_names])])
        num_poses = self.initial_poses.shape[0]
        left_costs = np.zeros(num_poses)
        left_angles = np.zeros([num_poses,4])
        left_successes = np.zeros(num_poses)
        right_costs = np.zeros(num_poses)
        right_angles = np.zeros([num_poses,4])
        right_successes = np.zeros(num_poses)

        #===== Perform inverse kinematics optimization for left arm =====#
        # Calculate desired setpoint
        left_controller_relative = np.array(self.left_controller_rotated.position) - np.array(self.left_controller_origin)
        left_pepper_setpoint = (self.arm_ratio*left_controller_relative) + self.pepper_model.left_hand_origin
        left_rotation = tf.transformations.quaternion_matrix(self.left_controller_rotated.quaternion)[0:3,0:3]

        # DEBUG: update setpoint transform
        self.pepper_left_setpoint.position = list(left_pepper_setpoint)

        #--- Perform Optimization over loop with different starting points
        # Use the 3 standard poses and the current pose.
        # Take the angles from the lowest cost.
        for i in range(num_poses):
            # Set initial condition (current joint configuration)
            x0_left = self.initial_poses[i,0:4]

            # Run optimization
            res_left = minimize(self.objective, x0_left, args=(left_pepper_setpoint, left_rotation[0:3,0], 'L'), method=opt_method, bounds=self.bounds_left)

            left_costs[i] = res_left.fun
            left_angles[i] = res_left.x
            left_successes[i] = res_left.success

        #===== Perform inverse kinematics optimization for right arm =====#
        # Calculate desired setpoint
        right_controller_relative = np.array(self.right_controller_rotated.position) - np.array(self.right_controller_origin)
        right_pepper_setpoint = (self.arm_ratio*right_controller_relative) + self.pepper_model.right_hand_origin
        right_rotation = tf.transformations.quaternion_matrix(self.right_controller_rotated.quaternion)[0:3,0:3]

        # DEBUG: update setpoint transform
        self.pepper_right_setpoint.position = list(right_pepper_setpoint)

        #--- Perform Optimization over loop with different starting points
        # Use the 3 standard poses and the current pose
        # Take the angles from the lowest cost
        for i in range(num_poses):
            # Set initial condition (current joint configuration)
            x0_right = self.initial_poses[i,5:9]

            # Run optimization
            res_right = minimize(self.objective, x0_right, args=(right_pepper_setpoint, right_rotation[0:3,0], 'R'), method=opt_method, bounds=self.bounds_right)

            right_costs[i] = res_right.fun
            right_angles[i] = res_right.x
            right_successes[i] = res_right.success

        #===== Evaluate the Solutions for Best Fit =====#
        # Find lowest cost that is successful
        left_index = 0
        right_index = 0
        for i in range(num_poses):
            left_index = np.argmin(left_costs)
            if (left_successes[left_index] == False):
                # Optimization invalid, delete row and find new option
                left_costs = np.delete(left_costs, left_index)
                left_angles = np.delete(left_angles, left_index, axis=0)
                left_successes = np.delete(left_successes, left_index)
            else:
                break

        for i in range(num_poses):
            right_index = np.argmin(right_costs)
            if (right_successes[right_index] == False):
                # Optimization invalid, delete row and find new option
                right_costs = np.delete(right_costs, right_index)
                right_angles = np.delete(right_angles, right_index, axis=0)
                right_successes = np.delete(right_successes, right_index)
            else:
                break

        #===== Set Joint Angles =====#
        # (if the array isze is 0 there were no successful optimzations, then
        # use the previous angles)
        if left_angles.shape[0] is not 0:
            for i in range(len(self.joint_names_left[0:-1])):
                self.angle_setpoints[self.joint_names_left[i]] = left_angles[left_index][i]
        else:
            rospy.logwarn('[{0}]: Left optimization failed.'.format(rospy.get_name()))
        if right_angles.shape[0] is not 0:
            for i in range(len(self.joint_names_right[0:-1])):
                self.angle_setpoints[self.joint_names_right[i]] = right_angles[right_index][i]
        else:
            rospy.logwarn('[{0}]: Right optimization failed.'.format(rospy.get_name()))

        # FIXME: vvv testing calculation values vvv
        # Update hand position
        self.pepper_model.setTransforms(np.array([self.angle_setpoints[key] for key in self.joint_names]))
        left_transform = self.pepper_model.getLeftHandTransform()

        # Calculate position error
        left_position = left_transform[0:3,3]
        left_error = left_pepper_setpoint - left_position
        left_position_error = np.sum(left_error**2)

        # Calculate orientation error
        left_x_axis = left_transform[0:3,0]
        left_controller_x_axis = left_rotation[0:3,0]

        left_orientation_error = np.arccos(left_x_axis.dot(left_controller_x_axis))/np.pi

        right_transform = self.pepper_model.getRightHandTransform()

        right_position = right_transform[0:3,3]
        right_error = right_pepper_setpoint - right_position
        right_position_error = np.sum(right_error**2)

        right_x_axis = right_transform[0:3,0]
        right_controller_x_axis = right_rotation[0:3,0]

        right_orientation_error = np.arccos(right_x_axis.dot(right_controller_x_axis))/np.pi

        # print('L Position: {0}\tL Orientation: {1}'.format(left_position_error, left_orientation_error))
        # print('R Position: {0}\tR Orientation: {1}'.format(right_position_error, right_orientation_error))
        # print('Current angles: {0}'.format([self.angle_setpoints[key] for key in self.joint_names]))
        # FIXME: ^^^ testing calculation values ^^^

    def objective(self, x, setpoint, controller_x_axis, hand):
        '''
        Objective function for the optimization.

        Parameters
        ----------
        x: list of float
            Array of joint angles in the following order [ShoulderPitch,
            ShoulderRoll, ElbowYaw, ElbowRoll].
        setpoint: numpy array
            The desired [x,y,z] relative position for Pepper's hand.
        controller_x_axis: numpy array
            (3,) numpy array containing the controller's X axis vector.
        hand: {'L', 'R'}
            The side to perform the optimization on.

        Returns
        -------
        cost: float
            The cost of the optimization objective, a weighted sum of errors.
        '''
        #===== Check Inputs =====#
        if not (len(x) == 4):
            raise ValueError('The objective input must be a numpy array of length 5')
        if hand not in ['L', 'R']:
            raise ValueError('Invalid hand type: {0}, must be "L" or "R"'.format(hand))

        position_error = 0.0
        orientation_error = 0.0

        #===== Left Arm =====#
        if hand == 'L':
            # Update hand position
            # Expand the joint angles to include the WristYaw as 0
            self.pepper_model.setTransformsLeft(np.append(x, 0.))
            left_transform = self.pepper_model.getLeftHandTransform()

            # Calculate position error
            left_position = left_transform[0:3,3]
            left_error = setpoint - left_position
            position_error = np.sum(left_error**2)

            # Calculate orientation error
            left_x_axis = left_transform[0:3,0]

            # Scaled by 1/pi, so being off by pi is equivalent to being off by 1
            # meter in position.
            try:
                if (left_x_axis.dot(controller_x_axis)) > 1.0:
                    print("Left angle: {0}".format(left_x_axis.dot(controller_x_axis)))
                orientation_error = np.arccos(left_x_axis.dot(controller_x_axis))/np.pi
            except RuntimeWarning:
                print('L Dot product: {0}'.format(left_x_axis.dot(controller_x_axis)))

        #===== Right Arm =====#
        else:
            # Update hand position
            # Expand the joint angles to include the WristYaw as 0
            self.pepper_model.setTransformsRight(np.append(x, 0.))
            right_transform = self.pepper_model.getRightHandTransform()

            # Calculate position error
            right_position = right_transform[0:3,3]
            right_error = setpoint - right_position
            position_error = np.sum(right_error**2)

            # Calculate orientation error
            right_x_axis = right_transform[0:3,0]

            # Scaled by 1/pi, so being off by pi is equivalent to being off by 1
            # meter in position
            try:
                if (right_x_axis.dot(controller_x_axis)) > 1.0:
                    print("Right angle: {0}".format(right_x_axis.dot(controller_x_axis)))
                orientation_error = np.arccos(right_x_axis.dot(controller_x_axis))/np.pi
            except RuntimeWarning:
                print('R Dot product: {0}'.format(right_x_axis.dot(controller_x_axis)))

        cost = self.position_weight*position_error + self.orientation_weight*orientation_error

        return cost

    def readTransforms(self):
        '''
        Reads the current states of the controllers.
        '''
        if self.left_controller.canTransform(self.tfListener):
            # Read the controller transforms
            self.left_controller.listen(self.tfListener)
            self.right_controller.listen(self.tfListener)

            # Convert to proper orientation (X: forward, Y: left, Z: up)
            self.left_controller_rotated.position = self.left_controller.position
            self.left_controller_rotated.quaternion = tf.transformations.quaternion_multiply(self.left_controller.quaternion, self.controller_rotation)
            self.right_controller_rotated.position = self.right_controller.position
            self.right_controller_rotated.quaternion = tf.transformations.quaternion_multiply(self.right_controller.quaternion, self.controller_rotation)

            # Update setpoint orientations to match controller's
            self.pepper_left_setpoint.quaternion = self.left_controller_rotated.quaternion
            self.pepper_right_setpoint.quaternion = self.right_controller_rotated.quaternion

        #======================================================================#
        # DEBUG: Test pose values
        #======================================================================#
        else:
            self.left_controller.position = self.left_test_pose.position
            self.left_controller.quaternion = self.left_test_pose.quaternion
            self.right_controller.position = self.right_test_pose.position
            self.right_controller.quaternion = self.right_test_pose.quaternion
        #======================================================================#
        # DEBUG: Test pose values
        #======================================================================#

    def broadcastRotatedTransforms(self):
        '''
        Broadcasts the left and right rotated controller frames.
        '''
        self.left_controller_rotated.broadcast(self.tfBroadcaster)
        self.right_controller_rotated.broadcast(self.tfBroadcaster)

    def broadcastDebugTransforms(self):
        '''
        Broadcasts the frames used for debugging with rviz.
        '''
        self.base_link.broadcast(self.tfBroadcaster)
        self.pepper_left_origin.broadcast(self.tfBroadcaster)
        self.pepper_right_origin.broadcast(self.tfBroadcaster)
        self.pepper_left_setpoint.broadcast(self.tfBroadcaster)
        self.pepper_right_setpoint.broadcast(self.tfBroadcaster)
        self.human_left_origin.broadcast(self.tfBroadcaster)
        self.human_right_origin.broadcast(self.tfBroadcaster)

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
            left_position = [0., 0., 0.]
            left_quaternion = [0., 0., 0., 1.]
            right_position = [0., 0., 0.]
            right_quaternion = [0., 0., 0., 1.]
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
            # 'world_rotated' frame needs to be broadcast
            self.world_rotated.broadcast(self.tfBroadcaster)

            # Get controller positions
            self.left_controller.listen(self.tfListener)
            self.right_controller.listen(self.tfListener)

            # Calculate average
            left_controller_average = (n*left_controller_average + self.left_controller.position)/(n+1)
            right_controller_average = (n*right_controller_average + self.right_controller.position)/(n+1)

            # Iterate on average
            n += 1

        self.left_controller_origin = list(left_controller_average)
        self.right_controller_origin = list(right_controller_average)
        rospy.loginfo('[{0}]: Left controller origin: [{1}, {2}, {3}]'.format(rospy.get_name(), self.left_controller_origin[0], self.left_controller_origin[1], self.left_controller_origin[2]))
        rospy.loginfo('[{0}]: Right controller origin: [{1}, {2}, {3}]'.format(rospy.get_name(), self.right_controller_origin[0], self.right_controller_origin[1], self.right_controller_origin[2]))

        # DEBUG: update debug transforms
        try:
            self.human_left_origin.position = self.left_controller_origin
            self.human_right_origin.position = self.right_controller_origin
        except AttributeError:
            # No value exists, skip assignment
            pass

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
