#!/usr/bin/env python
""" Sets Pepper's head joint angles based off the Vive headset orientation.

Reads the headset orientation from the HTC Vive relative to the Vive's 'world'
frame and then sends the HeadYaw and HeadPitch joint angle commands to Pepper. A
calibration procedure is provided to offset the 'yaw' angle. This allows the
user to face whatever direction they desire relative to the 'world' frame.

ROS Node Description
====================
Parameters
----------
~speed_fraction : float, default: 0.1
    The fraction of the maximum angular velocity for the joints to travel at
    going to the setpoint. The maximum angular velocity is determined inside
    Pepper. Lower values are safer for Pepper, but will make the tracking more
    delayed.
~yaw_offset : float, default: None
    The angle that is added to the 'yaw' command to offset it. This allows the
    user to face a different direction than the 'world' frame 0 position.
~calibration_time : float, default: 3.0
    The time in seconds over which the yaw angle is averaged to determine the
    offset angle.

Published Topics
----------------
/pepper_interface/joint_angles : naoqi_bridge_msgs/JointAnglesWithSpeed
    The joint angle setpoints that will be sent to Pepper.

Subscribed Topics
-----------------
~calibrate : std_msgs/Empty
    Receiving a message starts the calibration procedure and updates the
    'yaw_offset' parameter.
"""

# Python
import numpy as np

# ROS
import rospy
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from std_msgs.msg import Empty
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HeadsetControl():
    def __init__(self):
        #======================================================================#
        # ROS Setup
        #======================================================================#
        rospy.init_node("headset_control")

        #===== Parameters =====#
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        #----- Pepper Motion Parameters -----#
        self.fraction_max_head_speed = rospy.get_param('~speed_fraction', 0.1)
        self.yaw_offset = rospy.get_param('~yaw_offset', None)

        #----- Calibration Parameters -----#
        self.calibration_time = rospy.get_param('~calibration_time', 3.0) # sec
        self.calibration_start = 0.0
        self.calibration_average = 0.0

        #----- Joint Names and Angles -----#
        self.joint_names = ['HeadYaw', 'HeadPitch']
        self.angle_setpoints = {}
        for key in self.joint_names:
            self.angle_setpoints[key] = 0.0

        #===== Transform Listener/Broadcaster =====#
        self.tfListener = tf.TransformListener()
        self.tfBroadcaster = tf.TransformBroadcaster()

        #===== Publisher =====#
        self.joint_angles_msg = JointAnglesWithSpeed()
        self.joint_angles_msg.joint_names = self.joint_names
        self.joint_angles_msg.speed = self.fraction_max_head_speed
        self.joint_angles_pub = rospy.Publisher('/pepper_interface/joint_angles', JointAnglesWithSpeed, queue_size=3)

        #===== Subscriber =====#
        self.calibration_sub = rospy.Subscriber('~calibrate', Empty, self.calibrate, queue_size=1)

        #===== Check for Initial Calibration =====#
        if (self.yaw_offset is None):
            # Perform initial calibration
            print("\n{0}\n{1}{2}\n{0}".format(60*'=', 24*' ', 'Calibration'))
            print("No yaw offset provided. Performing initial calibration.")
            print("Stand facing the desired direction, press 'Enter', and hold for {0} seconds".format(self.calibration_time))
            raw_input("Press Enter:")
            self.calibrate(Empty())

    #==========================================================================#
    # Main Process
    #==========================================================================#
    def spin(self):
        """ Process loop: get angles, publish command. """
        while not rospy.is_shutdown():
            if self.findYawPitch():
                self.publishCommand()
            self.rate.sleep()

    def findYawPitch(self):
        """ Calculates Pepper head 'yaw' and 'pitch' angles from Vive headset's
        orientation.

        Reads the transforms published by the 'vive_ros' node for the Vive
        headset and converts the orientation into a 'yaw' and 'pitch' angle for
        Pepper's head to match the headset orientation.

        Returns
        -------
        bool
            Whether the 'yaw' and 'pitch' joint commands were successfully
            calculated.
        """
        angles_found = False

        try:
            #===== Wait for the transform =====#
            self.tfListener.waitForTransform('world', 'hmd', rospy.Time(), rospy.Duration.from_sec(1.0))
            self.hmd_position, self.hmd_rotation = self.tfListener.lookupTransform('world', 'hmd', rospy.Time())
            self.hmd_rotation_matrix = tf.transformations.quaternion_matrix(self.hmd_rotation)

            #===== Convert into Pepper's standard orientation =====#
            # (the default Vive orientation is X going RIGHT, Y going UP, and Z
            # going BACK)
            self.rotate_to_pepper = tf.transformations.euler_matrix(-np.pi/2, 0, np.pi/2, 'rxyz')
            self.head_rotation = self.hmd_rotation_matrix.dot(self.rotate_to_pepper)

            #===== Broadcast updated transform =====#
            if self.tfListener.canTransform('base_link_K', 'Head_K', rospy.Time()):
                self.tfListener.waitForTransform('base_link_K', 'Head_K', rospy.Time(), rospy.Duration.from_sec(1.0))
                self.head_position, _ = self.tfListener.lookupTransform('base_link_K', 'Head_K', rospy.Time())
                self.tfBroadcaster.sendTransform(self.head_position, tf.transformations.quaternion_from_matrix(self.head_rotation), rospy.Time.now(), 'Head_V', 'base_link_K')
            else:
                self.tfBroadcaster.sendTransform(self.hmd_position, tf.transformations.quaternion_from_matrix(self.head_rotation), rospy.Time.now(), 'Head_V', 'world')

            #===== Extract Yaw and Pitch angles =====#
            yaw, pitch, roll = tf.transformations.euler_from_matrix(self.head_rotation, 'rzyx')
            self.angle_setpoints['HeadYaw'] = yaw + self.yaw_offset
            self.angle_setpoints['HeadPitch'] = pitch
            angles_found = True
        except tf.Exception as err:
            print("[{0}]: TF Error: {1}".format(rospy.get_name(), err))

        return angles_found

    def publishCommand(self):
        """ Publishes the angle commands for Pepper's head. """
        self.joint_angles_msg.joint_angles = [self.angle_setpoints[key] for key in self.joint_names]
        self.joint_angles_pub.publish(self.joint_angles_msg)

    #==========================================================================#
    # Callback Functions
    #==========================================================================#
    def calibrate(self, msg):
        """ Calibrates the 'yaw' angle offset.

        Perform a calibration for the 'yaw' angle offset. This allows users to
        stand facing whatever direction they want and it will update to make
        that the zero position. This process assumes the floor has been
        properly calibrated and the user is facing straight forward (do not
        tilt the headset).
        """
        #===== Set Initial Conditions =====#
        if (self.yaw_offset is None):
            self.yaw_offset = 0
        self.calibration_average = 0
        self.calibration_start = rospy.get_time()

        rospy.loginfo('[{0}]: Calibrating for {1} seconds'.format(rospy.get_name(), self.calibration_time))

        #===== Run Calibration Loop =====@
        n = 0
        while (rospy.get_time() - self.calibration_start) < self.calibration_time:
            if (self.findYawPitch()):
                self.calibration_average = (n*self.calibration_average + self.angle_setpoints['HeadYaw']-self.yaw_offset)/(n+1)
                n += 1
                rospy.loginfo('[{0}]: {1:0.3f}, {2:0.3f}'.format(rospy.get_name(), self.angle_setpoints['HeadYaw']-self.yaw_offset, self.calibration_average))

        #===== Set Offset Parameter =====#
        # The average must be subtracted to make it go to 0
        self.yaw_offset = -self.calibration_average
        rospy.set_param('~yaw_offset', self.yaw_offset)
        rospy.loginfo('[{0}]: New yaw offset calibration: {1}'.format(rospy.get_name(), self.yaw_offset))


if __name__ == '__main__':
    try:
        headset_control = HeadsetControl()
        headset_control.spin()
    except rospy.ROSInterruptException:
        pass
