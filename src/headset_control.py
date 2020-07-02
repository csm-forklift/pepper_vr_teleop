#!/usr/bin/env python
'''
Reads the headset orientation from the HTC Vive and then sends the HeadYaw and HeadPitch commands to Pepper.
'''

import numpy as np
import rospy
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from std_msgs.msg import Empty
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HeadsetControl():
    def __init__(self):
        #===== Start ROS Node =====#
        rospy.init_node("headset_control")
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        self.joint_names = ['HeadYaw', 'HeadPitch']
        self.angle_setpoints = {}
        for key in self.joint_names:
            self.angle_setpoints[key] = 0.0

        #===== Pepper Motion Parameters =====#
        self.fraction_max_head_speed = rospy.get_param('~speed_fraction', 0.1)
        self.yaw_offset = rospy.get_param('~yaw_offset', None)

        #===== Calibration Parameters =====#
        self.calibration_time = rospy.get_param('~calibration_time', 3.0) # sec
        self.calibration_start = 0.0
        self.calibration_average = 0.0

        #===== Publisher =====#
        self.joint_angles_msg = JointAnglesWithSpeed()
        self.joint_angles_msg.joint_names = self.joint_names
        self.joint_angles_msg.speed = self.fraction_max_head_speed
        self.joint_angles_pub = rospy.Publisher('/pepper_interface/joint_angles', JointAnglesWithSpeed, queue_size=3)

        #===== TF Listener/Broadcaster =====#
        self.tfListener = tf.TransformListener()
        self.tfBroadcaster = tf.TransformBroadcaster()

        #===== Check for Initial Calibration =====#
        if (self.yaw_offset is None):
            # Perform initial calibration
            print("\n{0}\n{1}{2}\n{0}".format(60*'=', 24*' ', 'Calibration'))
            print("No yaw offset provided. Performing initial calibration.")
            print("Stand facing the desired direction, press 'Enter', and hold for {0} seconds".format(self.calibration_time))
            raw_input("Press Enter:")
            self.calibrate(Empty())

        #===== Calibration Subscriber =====#
        self.calibration_sub = rospy.Subscriber('~calibrate', Empty, self.calibrate, queue_size=1)

    def spin(self):
        while not rospy.is_shutdown():
            if self.findYawPitch():
                self.publishCommand()
            self.rate.sleep()

    def findYawPitch(self):
        '''
        Reads the transforms published by vive_ros for the Vive headset and
        converts the orientation into a Yaw and Pitch angle for Pepper's head.
        '''
        angles_found = False

        try:
            #--- Wait for the transform
            self.tfListener.waitForTransform('world', 'hmd', rospy.Time(), rospy.Duration.from_sec(1.0))
            self.hmd_position, self.hmd_rotation = self.tfListener.lookupTransform('world', 'hmd', rospy.Time())
            self.hmd_rotation_matrix = tf.transformations.quaternion_matrix(self.hmd_rotation)

            #--- Convert into Pepper's standard orientation
            # (the default Vive orientation is Y going UP, X going RIGHT, and Z
            # going BACK)
            self.rotate_to_pepper = tf.transformations.euler_matrix(-np.pi/2, 0, np.pi/2, 'rxyz')
            self.head_rotation = self.hmd_rotation_matrix.dot(self.rotate_to_pepper)

            #--- Broadcast updated transform
            if self.tfListener.canTransform('base_link_K', 'Head_K', rospy.Time()):
                self.tfListener.waitForTransform('base_link_K', 'Head_K', rospy.Time(), rospy.Duration.from_sec(1.0))
                self.head_position, _ = self.tfListener.lookupTransform('base_link_K', 'Head_K', rospy.Time())
                self.tfBroadcaster.sendTransform(self.head_position, tf.transformations.quaternion_from_matrix(self.head_rotation), rospy.Time.now(), 'Head_V', 'base_link_K')
            else:
                self.tfBroadcaster.sendTransform(self.hmd_position, tf.transformations.quaternion_from_matrix(self.head_rotation), rospy.Time.now(), 'Head_V', 'world')

            #--- Extract Yaw and Pitch angles
            yaw, pitch, roll = tf.transformations.euler_from_matrix(self.head_rotation, 'rzyx')
            self.angle_setpoints['HeadYaw'] = yaw + self.yaw_offset
            self.angle_setpoints['HeadPitch'] = pitch
            angles_found = True
        except tf.Exception as err:
            print("[{0}]: TF Error: {1}".format(rospy.get_name(), err))

        return angles_found

    def publishCommand(self):
        '''
        Publish angle commnds for Pepper.
        '''
        self.joint_angles_msg.joint_angles = [self.angle_setpoints[key] for key in self.joint_names]
        self.joint_angles_pub.publish(self.joint_angles_msg)

    def calibrate(self, msg):
        '''
        Perform a calibration for the Yaw angle offset. This allows users to
        stand facing whatever direction they want and it will update to make
        that the zero position.
        '''
        if (self.yaw_offset is None):
            self.yaw_offset = 0
        self.calibration_average = 0
        self.calibration_start = rospy.get_time()

        rospy.loginfo('[{0}]: Calibrating for {1} seconds'.format(rospy.get_name(), self.calibration_time))

        n = 0
        while (rospy.get_time() - self.calibration_start) < self.calibration_time:
            if (self.findYawPitch()):
                self.calibration_average = (n*self.calibration_average + self.angle_setpoints['HeadYaw']-self.yaw_offset)/(n+1)
                n += 1
                rospy.loginfo('[{0}]: {1:0.3f}, {2:0.3f}'.format(rospy.get_name(), self.angle_setpoints['HeadYaw']-self.yaw_offset, self.calibration_average))

        self.yaw_offset = -self.calibration_average
        rospy.loginfo('[{0}]: New yaw offset calibration: {1}'.format(rospy.get_name(), self.yaw_offset))


if __name__ == '__main__':
    try:
        headset_control = HeadsetControl()
        headset_control.spin()
    except rospy.ROSInterruptException:
        pass
