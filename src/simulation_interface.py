#!/usr/bin/env python
""" Handles communication with Pepper in the Gazebo simulation.

This node receives the joint command messages and converts them into messages
read by the Gazebo simulation. This is designed to allow the controller nodes to
publish a single command which can be used for both the real Pepper and the
simulation by running the corresponding interface node.

ROS Node Description
====================
Parameters
----------
~motion_time : float, default: 0.2
    The time in seconds for Pepper to move to the new target location.

Published Topics
----------------
/pepper_dcm/Head_controller/command : trajectory_msgs/JointTrajectory
    The trajectory command for the head joints.
/pepper_dcm/LeftArm_controller/command : trajectory_msgs/JointTrajectory
    The trajectory command for the left arm joints.
/pepper_dcm/RightArm_controller/command : trajectory_msgs/JointTrajectory
    The trajectory command for the right arm joints.

Subscribed Topics
-----------------
/pepper_interface/joint_angles : naoqi_bridge_msgs/JointAnglesWithSpeed
    The setpoints command for all joints.
/pepper_interface/cmd_vel : geometry_msgs/Twist
    The velocity command for the base, including linear and angular velocity.
"""

# ROS
import rospy
from geometry_msgs.msg import Twist
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SimulationInterface():
    def __init__(self):
        #======================================================================#
        # ROS Setup
        #======================================================================#
        rospy.init_node("simulation_interface")

        #===== Parameters =====#
        self.motion_time = rospy.get_param('~motion_time', 0.2)

        #===== Publishers =====#
        # Head
        self.head_msg = JointTrajectory()
        self.head_msg.joint_names = ['HeadYaw', 'HeadPitch']
        self.head_point = JointTrajectoryPoint()
        self.head_point.time_from_start = rospy.Duration.from_sec(self.motion_time)
        self.head_pub = rospy.Publisher("/pepper_dcm/Head_controller/command", JointTrajectory, queue_size=10)

        # Left Arm
        self.left_arm_msg = JointTrajectory()
        self.left_arm_msg.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
        self.left_arm_point = JointTrajectoryPoint()
        self.left_arm_point.time_from_start = rospy.Duration.from_sec(self.motion_time)
        self.left_arm_pub = rospy.Publisher("/pepper_dcm/LeftArm_controller/command", JointTrajectory, queue_size=10)

        # Right Arm
        self.right_arm_msg = JointTrajectory()
        self.right_arm_msg.joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        self.right_arm_point = JointTrajectoryPoint()
        self.right_arm_point.time_from_start = rospy.Duration.from_sec(self.motion_time)
        self.right_arm_pub = rospy.Publisher("/pepper_dcm/RightArm_controller/command", JointTrajectory, queue_size=10)

        self.angle_setpoints = {}
        for key in self.head_msg.joint_names + self.left_arm_msg.joint_names + self.right_arm_msg.joint_names:
            self.angle_setpoints[key] = 0.0

        #===== Subscribers =====#
        self.joint_angles_sub = rospy.Subscriber('/pepper_interface/joint_angles', JointAnglesWithSpeed, self.jointAnglesCallback, queue_size=3)
        self.cmd_vel_sub = rospy.Subscriber('/pepper_interface/cmd_vel', Twist, self.cmdVelCallback, queue_size=3)

    def jointAnglesCallback(self, msg):
        """ Sends joint command to simulated Pepper. """
        #===== Extract the message values =====#
        for i in range(len(msg.joint_names)):
            if self.angle_setpoints.has_key(msg.joint_names[i]):
                self.angle_setpoints[msg.joint_names[i]] = msg.joint_angles[i]

        #===== Update messages =====#
        # Head Trajectory
        self.head_point.positions = [self.angle_setpoints[key] for key in self.head_msg.joint_names]
        self.head_msg.points = [self.head_point]

        # Left Arm Trajectory
        self.left_arm_point.positions = [self.angle_setpoints[key] for key in self.left_arm_msg.joint_names]
        self.left_arm_msg.points = [self.left_arm_point]

        # Right Arm Trajectory
        self.right_arm_point.positions = [self.angle_setpoints[key] for key in self.right_arm_msg.joint_names]
        self.right_arm_msg.points = [self.right_arm_point]

        #===== Publish =====#
        self.head_pub.publish(self.head_msg)
        self.right_arm_pub.publish(self.right_arm_msg)
        self.left_arm_pub.publish(self.left_arm_msg)

    def cmdVelCallback(self, msg):
        """ Currently, the Gazebo simulation does not allow base movement. """
        pass


if __name__ == '__main__':
    try:
        simulation_interface = SimulationInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
