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
        self.deadband_radius = rospy.get_param('~deadband_radius', 0.2)
        self.fixed_frame = rospy.get_param('~fixed_frame', 'camera_link') # name of the parent frame to 'joystick' frame
        self.max_user_tilt_angle = math.pi/4
        self.x_max_length = math.cos(math.pi/2 - self.max_user_tilt_angle)
        self.y_max_length = math.cos(math.pi/2 - self.max_user_tilt_angle)
        self.velocity_x_max = 2 # m/s
        self.velocity_y_max = 2 # m/s
        self.velocity_angular_max = math.pi/4 # rad/s

        # Variables
        self.joystick_x = 0
        self.joystick_y = 0
        self.l_shoulder_x = 0
        self.l_shoulder_y = 0
        self.r_shoulder_x = 0
        self.r_shoulder_y = 0
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
            if not self.calculateLinearVelocity():
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.linear.y = 0
            if not self.calculateAngularVelocity():
                self.cmd_vel_msg.angular.z = 0

            # Publish velocity
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

            self.rate.sleep()

    def calculateLinearVelocity(self):
        '''
        Calculate the X and Y linear velocities of Pepper's base using the
        operator's torso as a joystick.
        '''
        return False

    def calculateAngularVelocity(self):
        '''
        Calculate the angular velocity of Pepper's base using the operator's
        shoulder angle.
        '''
        return False

    def calibrateJoystickPose(self, msg):
        '''
        Reads the "base_link_K" transform for 'calibration_time' seconds and
        averages the orientation. This value is then stored as the joystick
        frame orientation and used to find the velocities based on the deviation
        of the torso with respect to this frame.
        '''
        return False


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
