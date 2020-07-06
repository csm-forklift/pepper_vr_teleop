#!/usr/bin/env python
'''
Reads the torso (base_link) and shoulder transforms from the skeleton tracker.
Treats the Z axis of the torso like a joystick and sends X/Y velocity commands
based on the axis tilt. Also reads the shoulder positions and determines an
angular velocity command based on how much the operator has turned.
'''

import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import tf


class TorsoControl:
    def __init__(self):
        #===== Initialize ROS =====#
        rospy.init_node('torso_control')
        rospy.on_shutdown(self.shutdown)
        self.frequency = 30 # Hz
        self.rate = rospy.Rate(self.frequency)

        # Parameters
        self.calibration_time = rospy.get_param('~calibration_time', 3.0) # sec
        self.deadband_x = rospy.get_param('~deadband_x', 0.2)
        self.deadband_y = rospy.get_param('~deadband_y', 0.2)
        self.deadband_angle = rospy.get_param('~deadband_angle', math.pi/8)
        self.fixed_frame = rospy.get_param('~fixed_frame', 'camera_link') # name of the parent frame to 'joystick' frame
        self.joystick_frame = rospy.get_param('~joystick_frame', 'joystick')
        self.velocity_x_max = rospy.get_param('~velocity_x_max', 0.2) # m/s
        self.velocity_y_max = rospy.get_param('~velocity_y_max', 0.2) # m/s
        self.velocity_angular_max = rospy.get_param('~velocity_angular_max', math.pi/4) # rad/s
        self.velocity_angular_scale = 1
        self.max_user_tilt_angle = math.pi/4
        self.x_max_length = math.cos(math.pi/2 - self.max_user_tilt_angle)
        self.y_max_length = math.cos(math.pi/2 - self.max_user_tilt_angle)

        # Check to ensure the deadband does not exceed the max values
        if (self.x_max_length <= self.deadband_x):
            rospy.logerr("[{0}]: The deadband radius is larger than the maximum for the X dimension. Please decrease the deadband for x or increase the 'x_max_length' variable in the {0} node.".format(rospy.get_name()))
            rospy.signal_shutdown("[{0}]: Deadband radius error.".format(rospy.get_name()))
        if (self.y_max_length <= self.deadband_y):
            rospy.logerr("[{0}]: The deadband radius is larger than the maximum for the Y dimension. Please decrease the deadband radius or increase the 'y_max_length' variable in the {0} node.".format(rospy.get_name()))
            rospy.signal_shutdown("[{0}]: Deadband radius error.".format(rospy.get_name()))

        # Variables
        self.joystick_x = 0
        self.joystick_y = 0
        # Rotation from the joystick frame to the fixed frame
        self.joystick_quaternion = [0, 0, 0, 1]

        # TF Listener/Broadcaster
        self.tfListener = tf.TransformListener()
        self.tfBroadcaster = tf.TransformBroadcaster()

        # Publishers
        self.cmd_vel_msg = Twist()
        self.cmd_vel_pub = rospy.Publisher('/pepper_interface/cmd_vel', Twist, queue_size=3)

        # Subscribers
        self.calibration_sub = rospy.Subscriber('~calibrate', Empty, self.calibrateJoystickPose, queue_size=1)

    def spin(self):
        # Perform initial calibration
        print("\n{0}\n{1}{2}\n{0}".format(60*'=', 24*' ', 'Calibration'))
        print("Performing initial calibration.")
        print("Stand facing the desired direction with your back straight, press 'Enter', and hold for {0} seconds".format(self.calibration_time))
        raw_input("Press Enter:")
        if not self.calibrateJoystickPose(Empty()):
            rospy.logerr("[{0}]: Calibration failed. Make sure there is a transform from 'base_link_K' to '{1}' being published. Shutting down node...".format(rospy.get_name(), self.fixed_frame))
            rospy.signal_shutdown("[{0}]: Calibration failed. Cannot proceed without a valid joystick frame.".format(rospy.get_name()))

        while not rospy.is_shutdown():
            # Calculate velocities
            # If there are any errors, velocities are set to 0
            base_link, l_shoulder, r_shoulder = self.getTransforms()
            if base_link is None:
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.linear.y = 0
                self.cmd_vel_msg.angular.z = 0
            else:
                self.cmd_vel_msg.linear.x, self.cmd_vel_msg.linear.y = self.calculateLinearVelocity(base_link)
                self.cmd_vel_msg.angular.z = self.calculateAngularVelocity(l_shoulder, r_shoulder)

            # Publish velocity
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

            self.rate.sleep()

    def getTransforms(self):
        '''
        Looks up the transform for the base_link in the fixed frame. Uses
        base_link's position as the joystick frame's position and broadcasts the
        joystick transform. Finds the shoulder transforms in the joystick frame.
        Returns the orientation for the base_link and the positions for the left
        and right shoulders. These values are subsequently used to calculate the
        linear and angular velocities. If there are any errors, "base_link" is
        returned as "None".

        Args
        ----
        None

        Returns
        -------
        base_link:  4 element list, orientation of base_link in joystick frame
                    as quaternion [x, y, z, w].
        l_shoulder: 3 element list, position of LShoulder_K in joystick frame.
        r_shoulder: 3 element list, position of RShoulder_K in joystick frame.
        '''
        # Lookup base_link
        base_link_position = [0, 0, 0]
        base_link_rotation = [0, 0, 0, 1]
        try:
            self.tfListener.waitForTransform(self.fixed_frame, 'base_link_K', rospy.Time(), rospy.Duration.from_sec(1.0))
            base_link_position, base_link_rotation = self.tfListener.lookupTransform(self.fixed_frame, 'base_link_K', rospy.Time())
        except tf.Exception as err:
            rospy.loginfo("[{0}]: {1}".format(rospy.get_name(), err))
            return None, None, None

        # Publish joystick frame
        self.tfBroadcaster.sendTransform(base_link_position, self.joystick_quaternion, rospy.Time.now(), self.joystick_frame, self.fixed_frame)

        # Calculate base_link orientation in joystick frame
        # b = base_link, f = fixed_frame, j = joystick
        # f_R_b = rotation from base_link to fixed frame
        # f_R_j = rotation from joystick to fixed frame
        # j_R_b = (f_R_j))^-1 * f_R_b
        q_joystick_inverse = tf.transformations.quaternion_inverse(self.joystick_quaternion)
        base_link = tf.transformations.quaternion_multiply(q_joystick_inverse, base_link_rotation)

        # Lookup shoulders in joystick frame
        l_shoulder = [0, 0, 0]
        r_shoulder = [0, 0, 0]
        try:
            self.tfListener.waitForTransform(self.joystick_frame, 'LShoulder_K', rospy.Time(), rospy.Duration.from_sec(1.0))
            l_shoulder, _ = self.tfListener.lookupTransform(self.joystick_frame, 'LShoulder_K', rospy.Time())
            self.tfListener.waitForTransform(self.joystick_frame, 'RShoulder_K', rospy.Time(), rospy.Duration.from_sec(1.0))
            r_shoulder, _ = self.tfListener.lookupTransform(self.joystick_frame, 'RShoulder_K', rospy.Time())
        except tf.Exception as err:
            rospy.loginfo("[{0}]: {1}".format(rospy.get_name(), err))
            return None, None, None

        return base_link, l_shoulder, r_shoulder

    def calculateLinearVelocity(self, base_link_quaternion):
        '''
        Calculate the X and Y linear velocities of Pepper's base using the
        operator's torso as a joystick.

        Args
        ----
        base_link_quaternion: 4 element list, base_link orientation in joystick
                              frame.

        Returns
        -------
        velocity_x: linear velocity in the x direction
        velocity_y: linear velocity in the y direction
        '''
        # Initialize velocity
        velocity_x = 0.0
        velocity_y = 0.0

        # Convert the quaternion into a rotation matrix
        j_R_b = tf.transformations.quaternion_matrix(base_link_quaternion)

        # Extract the X and Y values of the Z axis
        self.joystick_x = j_R_b[0,2]
        self.joystick_y = j_R_b[1,2]

        # Check deadband
        if (abs(self.joystick_x) > self.deadband_x):
            # Saturate the joystick value
            self.joystick_x = min(self.joystick_x, self.x_max_length)
            self.joystick_x = max(self.joystick_x, -self.x_max_length)

            # Shift the joystick origin to the deadband position
            self.joystick_x = self.joystick_x - math.copysign(self.deadband_x, self.joystick_x)

            # Map to a range from 0 to 1
            self.joystick_x = self.joystick_x / (self.x_max_length - self.deadband_x)

            # Apply scaling
            velocity_x = self.velocity_x_max * self.joystick_x

        if (abs(self.joystick_y) > self.deadband_y):
            # Saturate the joystick value
            self.joystick_y = min(self.joystick_y, self.y_max_length)
            self.joystick_y = max(self.joystick_y, -self.y_max_length)

            # Shift the joystick origin to the deadband position
            self.joystick_y = self.joystick_y - math.copysign(self.deadband_y, self.joystick_y)

            # Map to a range from 0 to 1
            self.joystick_y = self.joystick_y / (self.y_max_length - self.deadband_y)

            # Apply scaling
            velocity_y = self.velocity_y_max * self.joystick_y

        return velocity_x, velocity_y

    def calculateAngularVelocity(self, l_shoulder_position, r_shoulder_position):
        '''
        Calculate the angular velocity of Pepper's base using the operator's
        shoulder angle.

        Args
        ----
        l_shoulder_position: 3 element list, (x,y,z) position of 'LShoulder_K'
        r_shoulder_position: 3 element list, (x,y,z) position of 'RSHoulder_K'

        Returns
        -------
        velocity_angular: angular velocity of the base
        '''
        velocity_angular = 0.0

        # Translate shoulder projecting onto joystick X-Y plane such that right
        # shoulder is on the origin. Find the angle of the line passing between
        # the shoulders with respect to the Y axis
        l_shoulder_x = l_shoulder_position[0] - r_shoulder_position[0]
        l_shoulder_y = l_shoulder_position[1] - r_shoulder_position[1]

        shoulder_theta = math.atan2(l_shoulder_y, l_shoulder_x) - math.pi/2

        # Apply deadband
        if (abs(shoulder_theta) > self.deadband_angle):
            velocity_angular = self.velocity_angular_scale*(shoulder_theta - math.copysign(self.deadband_angle, shoulder_theta))
            velocity_angular = min(velocity_angular, self.velocity_angular_max)
            velocity_angular = max(velocity_angular, -self.velocity_angular_max)

        return velocity_angular

    def calibrateJoystickPose(self, msg):
        '''
        Reads the "base_link_K" transform for 'calibration_time' seconds and
        averages the orientation. This value is then stored as the joystick
        frame orientation and used to find the velocities based on the deviation
        of the torso with respect to this frame.
        '''
        rospy.loginfo('[{0}]: Calibrating for {1} seconds.'.format(rospy.get_name(), self.calibration_time))
        calibration_start = rospy.get_time()

        try:
            # Obtain the first quaternion
            self.tfListener.waitForTransform(self.fixed_frame, 'base_link_K', rospy.Time(), rospy.Duration.from_sec(1.0))
            position, self.joystick_quaternion = self.tfListener.lookupTransform(self.fixed_frame, 'base_link_K', rospy.Time())
            count = 1

            # Iterate until time is reached
            while (rospy.get_time() - calibration_start) < self.calibration_time:
                # Read next torso orientation
                self.tfListener.waitForTransform(self.fixed_frame, 'base_link_K', rospy.Time(), rospy.Duration.from_sec(1.0))
                position, new_quaternion = self.tfListener.lookupTransform(self.fixed_frame, 'base_link_K', rospy.Time())
                count += 1

                # Average orientation
                self.joystick_quaternion = tf.transformations.quaternion_slerp(self.joystick_quaternion, new_quaternion, 1.0/count)

        except tf.Exception as err:
            rospy.loginfo('[{0}]: {1}'.format(rospy.get_name(), err))
            return False

        rospy.loginfo("[{0}]: Calibration complete. Using joystick frame quaternion [{1}, {2}, {3}, {4}]".format(rospy.get_name(), self.joystick_quaternion[0], self.joystick_quaternion[1], self.joystick_quaternion[2], self.joystick_quaternion[3]))

        return True

    def shutdown(self):
        '''
        Sends a zero velocity command before shutting down the node.
        '''
        rospy.loginfo('Shutting down "{0}", sending 0 velocity command.'.format(rospy.get_name()))
        self.cmd_vel_msg.linear.x = 0
        self.cmd_vel_msg.linear.y = 0
        self.cmd_vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)


if __name__ == '__main__':
    try:
        torso_control = TorsoControl()
        torso_control.spin()
    except rospy.ROSInterruptException:
        pass
