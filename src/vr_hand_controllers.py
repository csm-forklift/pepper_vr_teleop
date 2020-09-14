#!/usr/bin/env python
""" Controls Pepper's arms using the VR controller positions.

Uses the Vive VR joystick controllers to set the joint angles. It uses the
controller orientation and distance between the two controllers and a calibrated
origin position to determine where Pepper's hands should be placed. The inverse
kinematics used to determine the joint angles is done numerically using a
solver.

ROS Node Description
====================
Parameters
----------
~arm_ratio : float, default: 1.0
    The ratio of Pepper's arm to human arm. 0.5 means Pepper's arm length is
    half of the human operator's.
~velocity_linear_max : float, default: 0.3
    The maximum linear velocity allowed to send to the base.
~velocity_angular_max : float, default: 0.4
    The maximum angular velocity allowed to send to the base.
~joystick_deadband : float, default: 0.2
    The deadband value for the joystick. The joystick signal must be greater
    than this value before a motion command will be sent.
~left_name : str, default: 'controller_LHR_FFF73D47'
    The ROS TF tree frame name of the left controller.
~right_name : str, default: 'controller_LHR_FFFAFF45'
    The ROS TF tree frame name of the right controller.
~fixed_frame : str, default: 'world'
    The name of the fixed frame for the VR system.
~speed_fraction : float, default: 0.1
    The fraction of the maximum speed at which to move the arm joints.
~calibration_time : float, default: 1.0
    The time in seconds over which to average values for the calibrations.
/headset_control/yaw_offset : float, default: None
    The yaw offset as taken from the 'headset_control' node. If not set, the
    value is 'None' and a yaw offset will be determined for this node.
~yaw_offset : float, default: None
    Grabs the yaw offset if set for this node. If not set, the value is 'None'
    and a calibration must be performed before beginning the main process loop.
~position_weight : float, default: 100.0
    The weight for the position error when calculating the cost for the
    optimization objective function. Only change this value if you really need
    to alter the behavior. The current position/orientation weight ratio has
    been tuned for best visual performance.
~orientation_weight : float, default: 1.0
    The weight for the orientation error when calculating the cost for the
    optimization objective function. Only change this value if you really need
    to alter the behavior. The current position/orientation weight ratio has
    been tuned for best visual performance.

Published Topics
----------------
/pepper_interface/joint_angles : naoqi_bridge_msgs/JointAnglesWithSpeed
    The joint angle command sent to Pepper.
/pepper_interface/cmd_vel : geometry_msgs/Twist
    The linear and angular velocity command sent to Pepper.
/pepper_interface/grasp//left : std_msgs/Float64
    The grasp fraction sent to Pepper's left hand.
/pepper_interface/grasp/right : std_msgs/Float64
    The grasp fraction sent to Pepper's right hand.

Subscribed Topics
-----------------
/vive/<~left_name>/joy : sensor_msgs/Joy
    The buttons and axes configuration of the left joystick.
/vive/<~right_name>/joy : sensor_msgs/Joy
    The buttons and axes configuration of the right joystick.
"""

# Python
import math
from scipy.optimize import minimize
from pepper_transforms import Transform, PepperModel
import numpy as np

# ROS
import rospy
from geometry_msgs.msg import Twist
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import tf


class VRController():
    def __init__(self):
        #======================================================================#
        # Create Pepper Model for Inverse Kinematics
        #======================================================================#
        # Set Joints
        self.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        self.joint_names_side = {
            'L': self.joint_names[0:5],
            'R': self.joint_names[5:10]
        }
        self.angle_setpoints = {}
        for key in self.joint_names:
            self.angle_setpoints[key] = 0.0

        # Set bounds for optimization
        self.bounds_arm = {
            'L': [(-2.0857, 2.0857), (0.0087, 1.5620), (-2.0857, 2.0857),  (-1.5620, -0.0087)],
            'R': [(-2.0857, 2.0857), (-1.5620, -0.0087), (-2.0857, 2.0857), (0.0087, 1.5620)]
        }
        self.bounds_wrist = {
            'L': [(-1.8239, 1.8239)],
            'R': [(-1.8239, 1.8239)]
        }

        # These are the standard poses used for starting the optimization
        self.standard_poses = np.array([
            [-self.bounds_arm['L'][0][0], 0.0, -np.pi/2.0, -np.pi/4.0, 0.0, -self.bounds_arm['R'][0][0], 0.0, np.pi/2.0, np.pi/4.0, 0.0],
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

        #===== Parameters =====#
        # Loop rate
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)
        # Ratio of Pepper's arm to human arm
        self.arm_ratio = rospy.get_param('~arm_ratio', 1.0)
        rospy.loginfo('[{0}]: Arm ratio: {1}'.format(rospy.get_name(), self.arm_ratio))
        self.velocity_linear_max = rospy.get_param('~velocity_linear_max', 0.3)
        self.velocity_angular_max = rospy.get_param('~velocity_angular_max', 0.4)
        rospy.loginfo('[{0}]: Max linear velocity: {1}, Max angular velocity: {2}'.format(rospy.get_name(), self.velocity_linear_max, self.velocity_angular_max))
        self.joystick_deadband = rospy.get_param('~joystick_deadband', 0.2)
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
        self.cmd_vel_msg = Twist()
        self.cmd_vel_pub = rospy.Publisher('/pepper_interface/cmd_vel', Twist, queue_size=3)
        self.left_hand_grasp_msg = Float64()
        self.left_hand_grasp_pub = rospy.Publisher('/pepper_interface/grasp/left', Float64, queue_size=3)
        self.right_hand_grasp_msg = Float64()
        self.right_hand_grasp_pub = rospy.Publisher('/pepper_interface/grasp/right', Float64, queue_size=3)

        #===== Subscribers =====#
        self.left_controller_sub = rospy.Subscriber('/vive/' + self.left_name + '/joy', Joy, self.leftCallback, queue_size=1)
        self.right_controller_sub = rospy.Subscriber('/vive/' + self.right_name + '/joy', Joy, self.rightCallback, queue_size=1)

        #======================================================================#
        # Controller Setup and Calibration
        #======================================================================#
        #===== Controller Transforms =====#
        self.controller = {
            'L': Transform([0., 0., 0.], [0., 0., 0., 1.], 'world_rotated', self.left_name),
            'R': Transform([0., 0., 0.], [0., 0., 0., 1.], 'world_rotated', self.right_name)
        }
        self.controller_rotated = {
            'L': Transform([0., 0., 0.], [0., 0., 0., 1.], 'world_rotated', 'LHand_C'),
            'R': Transform([0., 0., 0.], [0., 0., 0., 1.], 'world_rotated', 'RHand_C')
        }
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
        self.controller_origin = {
            'L': [0., 0., 0.],
            'R': [0., 0., 0.]
        }
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
        self.test_pose = {
            'L': Transform([0.7, 0.3, 1.2], [-0.7427013, 0.3067963, 0.5663613, 0.1830457], 'world_rotated', 'LHand_C'),
            'R': Transform([0.8, -0.3, 1.5], [0.771698, 0.3556943, -0.5155794, 0.1101889], 'world_rotated', 'RHand_C')
        }

        # Display the origin and setpoint for human and pepper
        self.pepper_origin = {
            'L': Transform(self.pepper_model.hand_origin['L'], [0., 0., 0., 1.], 'base_link_V', 'LOrigin_P'),
            'R': Transform(self.pepper_model.hand_origin['R'], [0., 0., 0., 1.], 'base_link_V', 'ROrigin_P')
        }
        self.pepper_setpoint = {
            'L': Transform([0., 0., 0.], [0., 0., 0., 1.], 'base_link_V', 'LSetpoint_P'),
            'R': Transform([0., 0., 0.], [0., 0., 0., 1.], 'base_link_V', 'RSetpoint_P')
        }
        self.human_origin = {
            'L': Transform(self.controller_origin['L'], [0., 0., 0., 1.], 'world_rotated', 'LOrigin_H'),
            'R': Transform(self.controller_origin['R'], [0., 0., 0., 1.], 'world_rotated', 'ROrigin_H')
        }

    #==========================================================================#
    # Main Process
    #==========================================================================#
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

    #==========================================================================#
    # Utility Functions
    #==========================================================================#
    # DEBUG: Debugging function, test optimization speed
    def speedTest(self):
        """ Runs the optimization multiple times and finds and average speed.
        """
        n = 100
        x = np.zeros(n+1)
        x[0] = rospy.get_time()
        for i in range(1,n+1):
            self.controller_rotated['L'].position = list(np.random.rand(3))
            self.controller_rotated['R'].position = list(np.random.rand(3))
            self.calculateJointAngles()
            x[i] = rospy.get_time()

        sum_time = 0.0
        for i in range(1,n+1):
            sum_time = sum_time + (x[i] - x[i-1])
        avg_time = sum_time / n

        print('Ran optimization {0} times.\n Average speed: {1:0.6f} sec'.format(n, avg_time))

    # DEBUG: Debugging function, set pose for visual debugging
    def poseTest(self):
        """ Puts the robot model in a specific pose to test what the joint
        angles will produce.
        """
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
        """ Calculates the joint angles for the desired position.

        This function performs the inverse kinematics using an optimization
        solver to determine the best joint configuration for Pepper to match the
        hand with the VR system controllers.
        """
        #===== Optimization Paramters =====#
        opt_method = 'SLSQP'

        self.angle_setpoints_values = np.array([self.angle_setpoints[key] for key in self.joint_names])
        self.initial_pose_arm = {
            'L': self.angle_setpoints_values[0:4],
            'R': self.angle_setpoints_values[5:9]
        }
        self.initial_pose_wrist = {
            'L': self.angle_setpoints_values[4],
            'R': self.angle_setpoints_values[9]
        }

        # Initialize result dictionaries
        res_arm = {}
        res_wrist = {}

        #===== Perform inverse kinematics optimization for arms =====#
        for side in ['L', 'R']:
            # Calculate desired setpoint
            controller_relative = np.array(self.controller_rotated[side].position) - np.array(self.controller_origin[side])
            pepper_setpoint_position = (self.arm_ratio*controller_relative) + self.pepper_model.hand_origin[side]
            rotation = tf.transformations.quaternion_matrix(self.controller_rotated[side].quaternion)[0:3,0:3]

            # DEBUG: update setpoint transform
            self.pepper_setpoint[side].position = list(pepper_setpoint_position)

            # Set initial condition (current joint configuration)
            x0 = self.initial_pose_arm[side]

            # Run optimization
            res_arm[side] = minimize(self.objective, x0, args=(pepper_setpoint_position, rotation[0:3,0], side), method=opt_method, bounds=self.bounds_arm[side])

        #===== Perform inverse kinematics optimization for wrists =====#
        for side in ['L', 'R']:
            rotation = tf.transformations.quaternion_matrix(self.controller_rotated[side].quaternion)[0:3,0:3]
            x0 = self.initial_pose_wrist[side]
            res_wrist[side] = minimize(self.objectiveWrist, x0, args=(res_arm[side].x, rotation[0:3,1], rotation[0:3,2], side), method=opt_method, bounds=self.bounds_wrist[side])

        #===== Set Joint Angles =====#
        for side in ['L', 'R']:
            #----- Arm -----#
            if res_arm[side].success is not False:
                for i in range(len(self.joint_names_side[side][0:-1])):
                    self.angle_setpoints[self.joint_names_side[side][i]] = res_arm[side].x[i]
            else:
                rospy.logwarn('[{0}]: {1} arm optimization failed.'.format(rospy.get_name(), side))

            #----- Wrist -----#
            if res_wrist[side].success is not False:
                self.angle_setpoints[side + 'WristYaw'] = res_wrist[side].x
            else:
                rospy.logwarn('[{0}]: {1} wrist optimization failed.'.format(rospy.get_name(), side))

        # FIXME: vvv testing calculation values vvv
        # Update hand position
        self.pepper_model.setTransforms(np.array([self.angle_setpoints[key] for key in self.joint_names]))
        left_transform = self.pepper_model.getHandTransform('L')

        # Calculate position error
        left_position = left_transform[0:3,3]
        left_error = np.array(self.pepper_setpoint['L'].position) - left_position
        left_position_error = np.sum(left_error**2)

        # Calculate orientation error
        left_x_axis = left_transform[0:3,0]
        left_controller_x_axis = tf.transformations.quaternion_matrix(self.controller_rotated['L'].quaternion)[0:3,0]

        left_orientation_error = np.arccos(left_x_axis.dot(left_controller_x_axis))/np.pi

        right_transform = self.pepper_model.getHandTransform('R')

        right_position = right_transform[0:3,3]
        right_error = np.array(self.pepper_setpoint['R'].position) - right_position
        right_position_error = np.sum(right_error**2)

        right_x_axis = right_transform[0:3,0]
        right_controller_x_axis = tf.transformations.quaternion_matrix(self.controller_rotated['R'].quaternion)[0:3,0]

        right_orientation_error = np.arccos(right_x_axis.dot(right_controller_x_axis))/np.pi

        # print('L Position: {0}\tL Orientation: {1}'.format(left_position_error, left_orientation_error))
        # print('R Position: {0}\tR Orientation: {1}'.format(right_position_error, right_orientation_error))
        # print('Current angles: {0}'.format([self.angle_setpoints[key] for key in self.joint_names]))
        # FIXME: ^^^ testing calculation values ^^^

    def objective(self, x, setpoint, controller_x_axis, hand):
        """ Objective function for the arm optimization.

        Calculates the weighted error between the controller's position and the
        model's hand position and the angle between the controller's X axis and
        the model's hand X axis.

        Parameters
        ----------
        x : list of float
            Array of joint angles in the following order [ShoulderPitch,
            ShoulderRoll, ElbowYaw, ElbowRoll].
        setpoint : numpy array
            The desired [x,y,z] relative position for Pepper's hand.
        controller_x_axis : numpy array
            (3,) numpy array containing the controller's X axis vector.
        hand : {'L', 'R'}
            The side to perform the optimization on. 'L' = left, 'R' = right

        Returns
        -------
        cost : float
            The cost of the optimization objective, a weighted sum of errors.
        """
        #===== Check Inputs =====#
        if not (len(x) == 4):
            raise ValueError('The objective input must be a numpy array of length 4')
        if hand not in ['L', 'R']:
            raise ValueError('Invalid hand type: {0}, must be "L" or "R"'.format(hand))

        position_error = 0.0
        orientation_error = 0.0

        #===== Calculate Cost =====#
        # Update hand position
        # Expand the joint angles to include the WristYaw as 0
        self.pepper_model.setTransforms(np.append(x, 0.), hand)
        hand_transform = self.pepper_model.getHandTransform(hand)

        # Calculate position error
        hand_position = hand_transform[0:3,3]
        hand_error = setpoint - hand_position
        position_error = np.sum(hand_error**2)

        # Calculate orientation error
        x_axis = hand_transform[0:3,0]

        # Scaled by 1/pi, so being off by pi is equivalent to being off by 1
        # meter in position.
        try:
            if (x_axis.dot(controller_x_axis)) > 1.0:
                print(hand + ' angle: {0}'.format(x_axis.dot(controller_x_axis)))
            orientation_error = np.arccos(x_axis.dot(controller_x_axis))/np.pi
        except RuntimeWarning:
            print(hand + ' Dot product: {0}'.format(x_axis.dot(controller_x_axis)))

        cost = self.position_weight*position_error + self.orientation_weight*orientation_error

        return cost

    def objectiveWrist(self, x, arm_angles, controller_y_axis, controller_z_axis, hand):
        """ Objective function for determining the wrist yaw angles.

        Calculates the weighted error between the Y and Z axes of the Pepper
        model and the controller's pose Y and Z axes. The error is the angle
        between the model and controller axes.

        Parameters
        ----------
        x : float
            The wrist yaw angle for the given hand.
        arm_angles : numpy array
            (4,) numpy array of angles for the four other joints in the
            following order: [ShoulderPitch, ShoulderRoll, ElbowYaw, ElbowRoll]
        controller_y_axis : numpy array
            (3,) numpy array containing the controller's Y axis vector.
        controller_z_axis : numpy array
            (3,) numpy array containing the controller's Z axis vector.
        hand : {'L', 'R'}
            The side to perform the optimization on. 'L' = left, 'R' = right

        Returns
        -------
        cost : float
            The error between the model hand axes and the controller's.
        """
        #===== Check Inputs =====#
        if hand not in ['L', 'R']:
            raise ValueError('Invalid hand type for objectiveWrist(): {0}, must be "L" or "R"'.format(hand))

        error_y = 0.0
        error_z = 0.0

        #===== Calculate Cost =====#
        # Update hand position
        self.pepper_model.setTransforms(np.append(arm_angles, x), hand)
        hand_transform = self.pepper_model.getHandTransform(hand)
        y_axis = hand_transform[0:3,1]
        z_axis = hand_transform[0:3,2]

        # Get Y axis error
        error_y = np.arccos(y_axis.dot(controller_y_axis))

        # Get Z axis error
        error_z = np.arccos(z_axis.dot(controller_z_axis))

        cost = error_y + error_z

        return cost

    def readTransforms(self):
        """ Reads the current states of the controllers. """
        for side in ['L', 'R']:
            if self.controller[side].canTransform(self.tfListener):
                # Read the controller transforms
                self.controller[side].listen(self.tfListener)

                # Convert to proper orientation (X: forward, Y: left, Z: up)
                self.controller_rotated[side].position = self.controller[side].position
                self.controller_rotated[side].quaternion = tf.transformations.quaternion_multiply(self.controller[side].quaternion, self.controller_rotation)

                # Update setpoint orientations to match controller's
                self.pepper_setpoint[side].quaternion = self.controller_rotated[side].quaternion

            #==================================================================#
            # DEBUG: Test pose values
            #==================================================================#
            else:
                self.controller[side].position = self.test_pose[side].position
                self.controller[side].quaternion = self.test_pose[side].quaternion
            #==================================================================#
            # DEBUG: Test pose values
            #==================================================================#

    def broadcastRotatedTransforms(self):
        """ Broadcasts the left and right rotated controller frames. """
        self.controller_rotated['L'].broadcast(self.tfBroadcaster)
        self.controller_rotated['R'].broadcast(self.tfBroadcaster)

    def broadcastDebugTransforms(self):
        """ Broadcasts the frames used for debugging with rviz. """
        self.base_link.broadcast(self.tfBroadcaster)
        for side in ['L', 'R']:
            self.pepper_origin[side].broadcast(self.tfBroadcaster)
            self.pepper_setpoint[side].broadcast(self.tfBroadcaster)
            self.human_origin[side].broadcast(self.tfBroadcaster)

    def publishJointAnglesCommand(self):
        """ Publishes the current setpoints to the joint angle command topic.
        """
        self.joint_angles_msg.joint_angles = [self.angle_setpoints[key] for key in self.joint_angles_msg.joint_names]
        self.joint_angles_pub.publish(self.joint_angles_msg)

    #==========================================================================#
    # Callback Functions
    #==========================================================================#
    def calibrateOrientation(self):
        """ Calibrates the 'yaw_offset' angle which indicates the front
        direction.

        Continuously loops through reading the controller orientations,
        calculating the yaw angle of the controller in the world fixed frame,
        and then averages the yaw angles over 'calibration_time' seconds. The
        left and right controller yaw angles are averaged together before adding
        that value to the running average. This yaw angle determines the 'front'
        direction and is used to created the 'world_rotated' frame.
        """
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
                # The transform must be from the fixed frame to the controller
                # rather than using the 'self.controller' Transform object
                # because it uses the 'world_rotated' frame.
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
        """ Calibrates the origin positions for the controllers.

        Continuously loops through reading the controller position and adding it
        to a running average for 'calibration_time' seconds. The final average
        is the origin position which is used to find the position error used in
        the inverse kinematics calculations.
        """
        # Guard to keep from running more than one instance of the calibration
        # functions # at a time
        self.running_calibration = True

        controller_average = {
            'L': np.array([0.0, 0.0, 0.0]),
            'R': np.array([0.0, 0.0, 0.0])
        }

        rospy.loginfo('[{0}]: Calibrating positions for {1} seconds'.format(rospy.get_name(), self.calibration_time))
        calibration_start = rospy.get_time()

        #===== Calibration Loop =====#
        n = 0
        while (rospy.get_time() - calibration_start) < self.calibration_time:
            # 'world_rotated' frame needs to be broadcast before listening for
            # controllers
            self.world_rotated.broadcast(self.tfBroadcaster)

            for side in ['L', 'R']:
                # Get controller positions
                self.controller[side].listen(self.tfListener)

                # Calculate average
                controller_average[side] = (n*controller_average[side] + self.controller[side].position)/(n+1)

                # Iterate on average
                n += 1

        #===== Set Origin =====#
        for side in ['L', 'R']:
            self.controller_origin[side] = list(controller_average[side])
            rospy.loginfo('[{0}]: {1} controller origin: {2}'.format(rospy.get_name(), side, self.controller_origin[side]))

            # DEBUG: update debug transform for viewing calibration pose
            try:
                self.human_origin[side].position = self.controller_origin[side]
            except AttributeError:
                # No value exists, skip assignment
                pass

        self.position_calibration = True
        self.running_calibration = False

    def leftCallback(self, msg):
        """ Reads the left joystick buttons. """
        # Side button: perform orientation calibration
        if msg.buttons[3]:
            if not self.running_calibration:
                self.calibrateOrientation()

        # Trigger button: perform hand grasp
        self.left_hand_grasp_msg.data = msg.axes[2]
        self.left_hand_grasp_pub.publish(self.left_hand_grasp_msg)

        # Joystick: controls translational velocities
        # (Y axis joystick values are flipped and must be negated)
        if abs(msg.axes[1]) > self.joystick_deadband:
            self.cmd_vel_msg.linear.x = self.velocity_linear_max*msg.axes[1]
        else:
            self.cmd_vel_msg.linear.x = 0.0
        if abs(msg.axes[0]) > self.joystick_deadband:
            self.cmd_vel_msg.linear.y = -self.velocity_linear_max*msg.axes[0]
        else:
            self.cmd_vel_msg.linear.y = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def rightCallback(self, msg):
        """ Reads the right joystick buttons. """
        # Side button: perform position calibration
        if msg.buttons[3]:
            if not self.running_calibration:
                self.calibratePosition()

        # Trigger button: perform hand grasp
        self.right_hand_grasp_msg.data = msg.axes[2]
        self.right_hand_grasp_pub.publish(self.right_hand_grasp_msg)

        # Joystick: controls rotational velocities
        # (Joystick axes are reversed so velocity must be negated)
        if abs(msg.axes[0]) > self.joystick_deadband:
            self.cmd_vel_msg.angular.z = -self.velocity_angular_max*msg.axes[0]
        else:
            self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

if __name__ == '__main__':
    try:
        vr_controller = VRController()
        vr_controller.spin()
    except rospy.ROSInterruptException:
        pass
