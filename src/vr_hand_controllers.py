#!/usr/bin/env python
'''
Uses the Vive VR joystick controllers to set the joint angles. It uses the
controller orientation and distance between the two controllers to determine
where Pepper's hands should be placed.
'''

import math
from scipy.optimize import minimize

import rospy
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import Joy
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

        self.arm_ratio = rospy.get_param('~arm_ratio', 1.0)
        rospy.loginfo('[{0}]: Arm ratio: {1}'.format(rospy.get_name(), self.arm_ratio))

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
        self.tfListener = tf.TransformListener()
        self.tfBroadcaster = tf.TransformBroadcaster()

        #--- Controller Transforms
        # Name of left controller frame
        self.left_name = rospy.get_param('~left_name', 'controller_LHR_FFF73D47')
        # Name of right controller frame
        self.right_name = rospy.get_param('~right_name', 'controller_LHR_FFFAFF45')
        self.left_controller = Transform([0, 0, 0], [0, 0, 0, 1], 'world_rotated', 'LHand_C')
        self.right_controller = Transform([0, 0, 0], [0, 0, 0, 1], 'world_rotated', 'RHand_C')
        # Multiply the controller rotation by this rotation to adjust it to the
        # correct frame.
        # The initial orientation for the controllers are x: right, y: up, z:
        # backwards. Needed orientation is x: forward, y: left, z: up.
        self.controller_rotation = tf.transformations.quaternion_from_euler(0, math.pi/2, -math.pi/2, 'rzyx')

        # DEBUG: Test pose for controllers
        self.left_test_pose = Transform([0.7, 0.3, 1.2], [-0.7427013, 0.3067963, 0.5663613, 0.1830457], 'world_rotated', 'LHand_C')
        self.right_test_pose = Transform([0.8, -0.3, 1.5], [0.771698, 0.3556943, -0.5155794, 0.1101889], 'world_rotated', 'RHand_C')

        # Publishers
        self.fraction_max_arm_speed = rospy.get_param('~speed_fraction', 0.1)
        self.joint_angles_msg = JointAnglesWithSpeed()
        self.joint_angles_msg.joint_names = self.joint_names
        self.joint_angles_msg.speed = self.fraction_max_arm_speed
        self.joint_angles_pub = rospy.Publisher('/pepper_interface/joint_angles', JointAnglesWithSpeed, queue_size=3)

        # Subscribers
        self.left_controller_sub = rospy.Subscriber('/vive/' + self.left_name + '/joy', Joy, self.leftCallback, queue_size=1)
        self.right_controller_sub = rospy.Subscriber('/vive/' + self.right_name + '/joy', Joy, self.rightCallback, queue_size=1)

        #--- Rotated World Transform (and yaw_offset calibration)
        # (The joystick subscribers should be set up before the calibration
        # sequence. The side buttons initiate the calibration)
        # Check if 'headset_control' has set a yaw offset parameter
        # If not set one.

        # This rotation is applied to the controllers to rotate them about the world frame to be oriented such that the operator's forward position is directly along the X axis of the world. This new frame is published as 'world_rotated'
        self.world_rotated_position = [0, 0, 0]
        self.world_rotated = Transform(self.world_rotated_position, [0, 0, 0, 1], self.fixed_frame, 'world_rotated')

        self.running_calibration = False
        self.calibration_time = rospy.get_param('~calibration_time', 3.0)
        self.yaw_offset = rospy.get_param('/headset_control/yaw_offset', None)
        if self.yaw_offset is None:
            rospy.loginfo('[{0}]: No yaw_offset parameter found at "/headset_control/yaw_offset". Using value set at "{0}/yaw_offset"'.format(rospy.get_name()))
            self.yaw_offset = rospy.get_param('~yaw_offset', None)
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

        # Update transforms
        self.sendTransform(self.world_rotated)
        self.publishTransforms()

        #--- Controller origin calibration
        # Pepper Origin
        self.left_pepper_origin = [0, 0, 0]
        self.right_pepper_origin = [0, 0, 0]
        self.left_pepper_setpoint = [0, 0, 0]
        self.right_pepper_setpoint = [0, 0, 0]
        for key in self.joint_names:
            self.angle_setpoints[key] = 0.0
        self.angle_setpoints['LShoulderPitch'] = math.pi/2
        self.angle_setpoints['RShoulderPitch'] = math.pi/2
        self.calculateTransforms()
        self.left_pepper_origin, _, self.right_pepper_origin, _ = self.calculateHandPoses()

        # Human Origin
        self.left_controller_origin = [0, 0, 0]
        self.right_controller_origin = [0, 0, 0]
        print('\n{0}\n{1}{2}\n{0}'.format(60*'=', 16*' ', 'Position Calibration'))
        print('Point arms straight towards the ground. Press the side button on the RIGHT controller and hold position for {0} seconds.'.format(self.calibration_time))

        # Wait for user to press the side button and for calibration to complete
        self.position_calibration = False
        while not self.position_calibration:
            self.rate.sleep()

    def spin(self):
        # DEBUG: set joint angles for testing
        self.angle_setpoints['LShoulderPitch'] = 0.0
        self.angle_setpoints['LShoulderRoll'] = 0.0
        self.angle_setpoints['LElbowYaw'] = 0.0
        self.angle_setpoints['LElbowRoll'] = -math.pi/2
        self.angle_setpoints['LWristYaw'] = 0
        self.angle_setpoints['RShoulderPitch'] = 0
        self.angle_setpoints['RShoulderRoll'] = 0
        self.angle_setpoints['RElbowYaw'] = 0
        self.angle_setpoints['RElbowRoll'] = 0
        self.angle_setpoints['RWristYaw'] = -0.5
        self.calculateTransforms()

        while not rospy.is_shutdown():
            self.sendTransform(self.world_rotated)
            self.readTransforms()
            self.calculateJointAngles()
            self.publishJointAnglesCommand()
            # DEBUG: publish transforms for visualization
            self.publishTransforms()

            self.rate.sleep()

    def testFunction(self):
        '''
        Debugging function
        '''
        x = 101*[0]
        x[0] = rospy.get_time()
        for i in range(1, 101):
            res = self.optimizationWrapper()
            x[i] = rospy.get_time()
        # Average time
        times = 0
        for i in range(1, 101):
            times += x[i] - x[i-1]
        avg_time = times/100
        print('Average time: {0}'.format(avg_time))

        for i in range(len(self.joint_names)):
            self.angle_setpoints[self.joint_names[i]] = res.x[i]
        self.calculateTransforms()

    def calculateJointAngles(self):
        '''
        Calculates the joint angles for the desired position using the
        optimizer.
        '''
        res = self.optimizationWrapper()
        for i in range(len(self.joint_names)):
            self.angle_setpoints[self.joint_names[i]] = res.x[i]

    def objective(self, x, left_setpoint, right_setpoint):
        '''
        Objective function for the optimization.

        Args
        ----
        x: ([10*float]), array of joint angles [LShoulderPitch, LShoulderRoll,
           LElbowYaw, LElbowRoll, LWristYaw, RShoulderPitch, RShoulderRoll,
           RElbowYaw, RElbowRoll, RWristYaw]
        args: (([x,y,z], [x,y,z])), tuple containing two 3 element lists. The
              first is the left hand setpoint and the second is the right hand
              setpoint.

        Returns
        -------
        cost: (float), the cost of the optimization objective
        '''
        # Set angles
        for i in range(len(self.joint_names)):
            self.angle_setpoints[self.joint_names[i]] = x[i]

        # Update the hand positions
        self.calculateTransforms()
        left_position, left_quaternion, right_position, right_quaternion = self.calculateHandPoses()

        # Calculate position error (distance squared)
        left_relative = [left_position[i] - self.left_pepper_origin[i] for i in range(3)]
        right_relative = [right_position[i] - self.right_pepper_origin[i] for i in range(3)]
        left_error = 0
        right_error = 0
        for i in range(3):
            left_error += (left_setpoint[i] - left_relative[i])**2
            right_error += (right_setpoint[i] - right_relative[i])**2
        cost = left_error + right_error

        return cost

    def optimizationWrapper(self):
        '''
        Sets a starting value for the optimization, runs the optimization, then
        publishes the result.

        Args
        ----
        None

        Returns
        -------
        res: (OptimizeResult), object containing,
             x: solution array
             success: boolean indicating if optimizer exited successfully
             message: string containing cause of termination
        '''
        # Determine the desired setpoint
        left_controller_relative = [self.left_controller.position[i] - self.left_controller_origin[i] for i in range(3)]
        right_controller_relative = [self.right_controller.position[i] - self.right_controller_origin[i] for i in range(3)]
        left_pepper_setpoint = [left_controller_relative[i]*self.arm_ratio for i in range(3)]
        right_pepper_setpoint = [right_controller_relative[i]*self.arm_ratio for i in range(3)]

        x0 = [self.angle_setpoints[key] for key in self.joint_names]
        bounds = [(-2.0857, 2.0857), (0.0087, 1.5620), (-2.0857, 2.0857),  (-1.5620, -0.0087), (-1.8239, 1.8239), (-2.0857, 2.0857), (-1.5620, -0.0087), (-2.0857, 2.0857), (0.0087, 1.5620), (-1.8239, 1.8239)]

        res = minimize(self.objective, x0, args=(left_pepper_setpoint, right_pepper_setpoint), method='SLSQP', bounds=bounds)

        return res

    def readTransforms(self):
        '''
        Reads the current states of the controllers.
        '''
        if self.tfListener.canTransform('world_rotated', self.left_name, rospy.Time()):
            # Read the controller transforms
            self.tfListener.waitForTransform('world_rotated', self.left_name, rospy.Time(), rospy.Duration.from_sec(1.0))
            left_position, left_quaternion = self.tfListener.lookupTransform('world_rotated', self.left_name, rospy.Time())
            self.tfListener.waitForTransform('world_rotated', self.right_name, rospy.Time(), rospy.Duration.from_sec(1.0))
            right_position, right_quaternion = self.tfListener.lookupTransform('world_rotated', self.right_name, rospy.Time())

            # Convert to proper orientation (X: forward, Y: left, Z: up)
            self.left_controller.position = left_position
            self.left_controller.quaternion = tf.transformations.quaternion_multiply(left_quaternion, self.controller_rotation)
            self.right_controller.position = right_position
            self.right_controller.quaternion = tf.transformations.quaternion_multiply(right_quaternion, self.controller_rotation)

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

    def calculateHandPoses(self):
        '''
        Calculates the pose of the hands in the base_link frame. Returns the
        position and orientation as a quaternion for each hand.

        Args
        ----
        None

        Returns
        -------
        left_hand_position: ([float, float, float]), (x,y,z) position of the
                            left hand in base_link frame
        left_hand_quaternion: ([float, float, float, float]), (x,y,z,w)
                              quaternion orientation of the left hand in
                              base_link frame
        right_hand_position: ([float, float, float]), (x,y,z) position of the
                             right hand in base_link frame
        right_hand_quaternion: ([float, float, float, float]), (x,y,z,w)
                               quaternion orientation of the right hand in
                               base_link frame
        '''
        # Left Hand
        base_T_lshoulder = self.tfListener.fromTranslationRotation(self.LShoulder.position, self.LShoulder.quaternion)
        lshoulder_T_lelbow = self.tfListener.fromTranslationRotation(self.LElbow.position, self.LElbow.quaternion)
        lelbow_T_lwrist = self.tfListener.fromTranslationRotation(self.LWrist.position, self.LWrist.quaternion)
        lwrist_T_lhand = self.tfListener.fromTranslationRotation(self.LHand.position, self.LHand.quaternion)

        # Right Hand
        base_T_rshoulder = self.tfListener.fromTranslationRotation(self.RShoulder.position, self.RShoulder.quaternion)
        rshoulder_T_relbow = self.tfListener.fromTranslationRotation(self.RElbow.position, self.RElbow.quaternion)
        relbow_T_rwrist = self.tfListener.fromTranslationRotation(self.RWrist.position, self.RWrist.quaternion)
        rwrist_T_rhand = self.tfListener.fromTranslationRotation(self.RHand.position, self.RHand.quaternion)

        base_T_lhand = base_T_lshoulder.dot(lshoulder_T_lelbow.dot(lelbow_T_lwrist.dot(lwrist_T_lhand)))
        base_T_rhand = base_T_rshoulder.dot(rshoulder_T_relbow.dot(relbow_T_rwrist.dot(rwrist_T_rhand)))

        left_hand_position = base_T_lhand[0:3,3]
        left_hand_quaternion = tf.transformations.quaternion_from_matrix(base_T_lhand)
        right_hand_position = base_T_rhand[0:3,3]
        right_hand_quaternion = tf.transformations.quaternion_from_matrix(base_T_rhand)

        return left_hand_position, left_hand_quaternion, right_hand_position, right_hand_quaternion

    def publishTransforms(self):
        '''
        Pulishes the transforms for Pepper's joints through
        the /tf topic. This is used for debugging purposes.
        '''
        #----- Fixed Frames -----#
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

        # # DEBUG: test poses
        # self.sendTransform(self.left_test_pose)
        # self.sendTransform(self.right_test_pose)

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

        left_controller_average = [0, 0, 0]
        right_controller_average = [0, 0, 0]

        rospy.loginfo('[{0}]: Calibrating positions for {1} seconds'.format(rospy.get_name(), self.calibration_time))
        calibration_start = rospy.get_time()

        n = 0
        while (rospy.get_time() - calibration_start) < self.calibration_time:
            try:
                self.tfListener.waitForTransform('world_rotated', self.left_name, rospy.Time(), rospy.Duration.from_sec(1.0))
                self.left_controller_origin, _ = self.tfListener.lookupTransform('world_rotated', self.left_name, rospy.Time())
                self.tfListener.waitForTransform('world_rotated', self.right_name, rospy.Time(), rospy.Duration.from_sec(1.0))
                self.right_controller_origin, _ = self.tfListener.lookupTransform('world_rotated', self.right_name, rospy.Time())
            except tf.Exception as err:
                rospy.logwarn('[{0}]: TF Error: {1}'.format(rospy.get_name(), err))

            left_controller_average[0] = (n*left_controller_average[0] + self.left_controller_origin[0])/(n+1)
            left_controller_average[1] = (n*left_controller_average[1] + self.left_controller_origin[1])/(n+1)
            left_controller_average[2] = (n*left_controller_average[2] + self.left_controller_origin[2])/(n+1)

            right_controller_average[0] = (n*right_controller_average[0] + self.right_controller_origin[0])/(n+1)
            right_controller_average[1] = (n*right_controller_average[1] + self.right_controller_origin[1])/(n+1)
            right_controller_average[2] = (n*right_controller_average[2] + self.right_controller_origin[2])/(n+1)

            n += 1

        self.left_controller_origin = left_controller_average
        self.right_controller_origin = right_controller_average
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
