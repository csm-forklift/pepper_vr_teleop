#!/usr/bin/env python

'''
Node used to set Pepper in a particular pose. Can use the ROS DCM controller
by publishing to topics for either the position control or trajectory control.
Can also use the ALMotion library from Python for comparison.
'''

import math

import rospy
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dynamic_reconfigure.server import Server
from pepper_vr_teleop.cfg import TestControlConfig

from naoqi import ALProxy # for ALMotion API

class TestControl:
    def __init__(self):
        # Control Types (default = trajectory):
        #   trajectory: use the dcm trajectory control topics, for this option
        #               you need to be running the appropriate launch file for
        #               trajectory control.
        #   position: use the dcm position control topics, for this option you
        #             need to be running the appropriate launch file for
        #             position control.
        #   almotion: use the ALMotion python library, for this option
        #             you need to provide the robot's IP address and Port as
        #             parameters to this node.
        #   joint_angles: use the /pepper_robot/pose/joint_angles topic.
        self.control_types = ['trajectory', 'position', 'almotion','joint_angles']
        self.control_types_str = "["
        for i in range(len(self.control_types)):
            if i < (len(self.control_types) - 1):
                self.control_types_str += self.control_types[i] + ", "
            else:
                self.control_types_str += self.control_types[i] + "]"

        #===== ROS Objects =====#
        rospy.init_node("test_control")
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        # Validate "control_type" parameter
        self.control_type = rospy.get_param('~control_type', 'trajectory')
        if (self.control_type not in self.control_types):
            rospy.logwarn("[" + rospy.get_name() + "]: The control_type parameter '%s', does not match any of the available control types:\n%s\nSetting to 'trajectory'", self.control_type, self.control_types_str)
            self.control_type = 'trajectory'
        rospy.loginfo("[" + rospy.get_name() + "]: using control type: " + self.control_type)

        # Network connection variables, if using ALMotion
        self.robot_ip = rospy.get_param('~robot_ip', '127.0.0.1')
        self.robot_port = rospy.get_param('~port', 9559)

        # Set publishers based on control type
        # (trajectory is set as the "else" statement in order to make it the
        # default)
        if self.control_type == self.control_types[3]:
            #--- joint_angles control ---#
            self.joint_angles_pub = rospy.Publisher('/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=3)
            self.joint_angles_msg = JointAnglesWithSpeed()
            self.joint_angles_msg.header.seq = 0
            self.joint_anlges_msg.speed = 0.1
        elif self.control_type == self.control_types[2]:
            #--- almotion control ---#
            # Establish connection with robot
            self.motionProxy = ALProxy("ALMotion", self.robot_ip, self.robot_port)
            self.postureProxy = ALProxy("ALRobotPosture", self.robot_ip, self.robot_port)
            self.motionProxy.wakeUp()
            # Not sure that I need this, it was in Skyler's code
            # self.motionProxy.wbEnable(False)
            self.postureProxy.goToPosture('StandInit', 0.5)

            # Head
            self.HeadJoints = ['HeadYaw', 'HeadPitch']

            # Left Arm
            self.LeftArmJoints = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll']

            # Right Arm
            self.RightArmJoints = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
        elif self.control_type == self.control_types[1]:
            #--- position control ---#
            print "setting position publishers"
            # Head
            self.HeadYaw_pub = rospy.Publisher("/pepper_dcm/HeadYaw_position_controller/command", Float64, queue_size=10)
            self.HeadYaw_msg = Float64()
            self.HeadPitch_pub = rospy.Publisher("/pepper_dcm/HeadPitch_position_controller/command", Float64, queue_size=10)
            self.HeadPitch_msg = Float64()

            # Left Arm
            self.LShoulderPitch_pub = rospy.Publisher("/pepper_dcm/LShoulderPitch_position_controller/command", Float64, queue_size=10)
            self.LShoulderPitch_msg = Float64()
            self.LShoulderRoll_pub = rospy.Publisher("/pepper_dcm/LShoulderRoll_position_controller/command", Float64, queue_size=10)
            self.LShoulderRoll_msg = Float64()
            self.LElbowYaw_pub = rospy.Publisher("/pepper_dcm/LElbowYaw_position_controller/command", Float64, queue_size=10)
            self.LElbowYaw_msg = Float64()
            self.LElbowRoll_pub = rospy.Publisher("/pepper_dcm/LElbowRoll_position_controller/command", Float64, queue_size=10)
            self.LElbowRoll_msg = Float64()

            # Right Arm
            self.RShoulderPitch_pub = rospy.Publisher("/pepper_dcm/RShoulderPitch_position_controller/command", Float64, queue_size=10)
            self.RShoulderPitch_msg = Float64()
            self.RShoulderRoll_pub = rospy.Publisher("/pepper_dcm/RShoulderRoll_position_controller/command", Float64, queue_size=10)
            self.RShoulderRoll_msg = Float64()
            self.RElbowYaw_pub = rospy.Publisher("/pepper_dcm/RElbowYaw_position_controller/command", Float64, queue_size=10)
            self.RElbowYaw_msg = Float64()
            self.RElbowRoll_pub = rospy.Publisher("/pepper_dcm/RElbowRoll_position_controller/command", Float64, queue_size=10)
            self.RElbowRoll_msg = Float64()
        else:
            #--- trajectory control ---#
            # Head
            self.Head_pub = rospy.Publisher("/pepper_dcm/Head_controller/command", JointTrajectory, queue_size=10)
            self.Head_msg = JointTrajectory()
            self.Head_msg.joint_names = ['HeadYaw', 'HeadPitch']

            # Left Arm
            self.LeftArm_pub = rospy.Publisher("/pepper_dcm/LeftArm_controller/command", JointTrajectory, queue_size=10)
            self.LeftArm_msg = JointTrajectory()
            self.LeftArm_msg.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']

            # Right Arm
            self.RightArm_pub = rospy.Publisher("/pepper_dcm/RightArm_controller/command", JointTrajectory, queue_size=10)
            self.RightArm_msg = JointTrajectory()
            self.RightArm_msg.joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']

            # JointState feedback
            self.JointState_sub = rospy.Subscriber("/joint_states", JointState, self.jointStateCallback, queue_size=10)

        #===== Set Desired Starting Pose =====#
        self.head_yaw = -2.0
        self.head_pitch = 0.0
        self.l_shoulder_pitch = 0.0
        self.l_shoulder_roll = 0.0
        self.l_elbow_yaw = 0.0
        self.l_elbow_roll = 0.0
        self.r_shoulder_pitch = 0.0
        self.r_shoulder_roll = 0.0
        self.r_elbow_yaw = 0.0
        self.r_elbow_roll = 0.0

        #===== Previous Angle Positions =====#
        self.head_yaw_current = 0.0
        self.head_pitch_current = 0.0
        self.l_shoulder_pitch_current = 0.0
        self.l_shoulder_roll_current = 0.0
        self.l_elbow_yaw_current = 0.0
        self.l_elbow_roll_current = 0.0
        self.r_shoulder_pitch_current = 0.0
        self.r_shoulder_roll_current = 0.0
        self.r_elbow_yaw_current = 0.0
        self.r_elbow_roll_current = 0.0

        #===== Start Dynamic Reconfigure Server =====#
        # This allows you to change the angles using a slider
        # Start the dynamic reconfigure GUI by using the command
        # $ rosrun rqt_gui rqt_gui -s reconfigure
        reconfig_srv = Server(TestControlConfig, self.dynamicReconfigCallback)

    def spin(self):
        while not rospy.is_shutdown():
            if self.control_type == self.control_types[3]:
                #--- joint_angles control ---#
                self.publishJointAnglesCommand()
            elif self.control_type == self.control_types[2]:
                #--- almotion control ---#
                self.publishALMotionCommand()
            elif self.control_type == self.control_types[1]:
                #--- position control ---#
                self.publishPositionCommmand()
            else:
                #--- trajectory control ---#
                self.publishTrajectoryCommand()
            self.rate.sleep()

    def publishALMotionCommand(self):
        # Motion Parameters
        # fractions from 0 - 1 where 1 = 100% (full speed)
        self.fractionMaxHeadSpeed = 0.25
        self.fractionMaxArmSpeed = 0.2

        # Head
        self.motionProxy.setAngles(self.HeadJoints, [self.head_yaw, self.head_pitch], self.fractionMaxHeadSpeed)

        # Left Arm
        self.motionProxy.setAngles(self.LeftArmJoints, [self.l_shoulder_pitch, self.l_shoulder_roll, self.l_elbow_yaw, self.l_elbow_roll], self.fractionMaxArmSpeed)

        # Right Arm
        self.motionProxy.setAngles(self.RightArmJoints, [self.r_shoulder_pitch, self.r_shoulder_roll, self.r_elbow_yaw, self.r_elbow_roll], self.fractionMaxArmSpeed)

    def publishPositionCommmand(self):
        # Head
        self.HeadYaw_msg.data = self.head_yaw
        self.HeadPitch_msg.data = self.head_pitch

        # Left Arm
        self.LShoulderPitch_msg.data = self.l_shoulder_pitch
        self.LShoulderRoll_msg.data = self.l_shoulder_roll
        self.LElbowYaw_msg.data = self.l_elbow_yaw
        self.LElbowRoll_msg.data = self.l_elbow_roll

        # Right Arm
        self.RShoulderPitch_msg.data = self.r_shoulder_pitch
        self.RShoulderRoll_msg.data = self.r_shoulder_roll
        self.RElbowYaw_msg.data = self.r_elbow_yaw
        self.RElbowRoll_msg.data = self.r_elbow_roll

        # Publish
        self.HeadYaw_pub.publish(self.HeadYaw_msg)
        self.HeadPitch_pub.publish(self.HeadPitch_msg)
        self.LShoulderPitch_pub.publish(self.LShoulderPitch_msg)
        self.LShoulderRoll_pub.publish(self.LShoulderRoll_msg)
        self.LElbowYaw_pub.publish(self.LElbowYaw_msg)
        self.LElbowRoll_pub.publish(self.LElbowRoll_msg)
        self.RShoulderPitch_pub.publish(self.RShoulderPitch_msg)
        self.RShoulderRoll_pub.publish(self.RShoulderRoll_msg)
        self.RElbowYaw_pub.publish(self.RElbowYaw_msg)
        self.RElbowRoll_pub.publish(self.RElbowRoll_msg)

    def publishTrajectoryCommand(self):
        # Motion Parameters
        # (these max values were arbitrarily selected by me based on how fast I felt comfotable with the arm motion, these values may be changed)
        self.max_head_velocity = math.pi # rad/s
        self.max_shoulder_velocity = math.pi
        self.max_elbow_velocity = 2*math.pi

        # Scale factor
        self.fractionMaxHeadSpeed = 1.0
        self.fractionMaxArmSpeed = 1.0

        # Scale velocities
        self.head_velocity_desired = self.max_head_velocity*self.fractionMaxHeadSpeed
        self.shoulder_velocity_desired = self.max_shoulder_velocity*self.fractionMaxArmSpeed
        self.elbow_velocity_desired = self.max_elbow_velocity*self.fractionMaxArmSpeed

        #--- Calculate joint errors
        # self.head_errors = [self.head_yaw - self.head_yaw_current,
        #                     self.head_pitch - self.head_pitch_current]

        self.l_arm_errors = \
            [self.l_shoulder_pitch - self.l_shoulder_pitch_current,
             self.l_shoulder_roll - self.l_shoulder_roll_current,
             self.l_elbow_yaw - self.l_elbow_yaw_current,
             self.l_elbow_roll - self.l_elbow_roll_current]

        self.r_arm_errors = \
            [self.r_shoulder_pitch - self.r_shoulder_pitch_current,
             self.r_shoulder_roll - self.r_shoulder_roll_current,
             self.r_elbow_yaw - self.r_elbow_yaw_current,
             self.r_elbow_roll - self.r_elbow_roll_current]

        # Calculate time required to achieve desired location
        # self.head_times = [abs(self.head_errors[0]/self.head_velocity_desired),
        #                    abs(self.head_errors[1]/self.head_velocity_desired)]

        self.l_arm_times = \
            [abs(self.l_arm_errors[0]/self.shoulder_velocity_desired),
             abs(self.l_arm_errors[1]/self.shoulder_velocity_desired),
             abs(self.l_arm_errors[2]/self.elbow_velocity_desired),
             abs(self.l_arm_errors[3]/self.elbow_velocity_desired)]

        self.r_arm_times = \
            [abs(self.r_arm_errors[0]/self.shoulder_velocity_desired),
             abs(self.r_arm_errors[1]/self.shoulder_velocity_desired),
             abs(self.r_arm_errors[2]/self.elbow_velocity_desired),
             abs(self.r_arm_errors[3]/self.elbow_velocity_desired)]

        # Set the trajectory time to be the largest time required to achieve the desired velocity
        # self.head_motion_time = max(self.head_times)
        self.l_arm_motion_time = max(self.l_arm_times)
        self.r_arm_motion_time = max(self.r_arm_times)

        # DEBUG: hardcode the motion times for testing, comment this section out
        # when you want to use the desired velocities
        self.motion_time = 0.2
        self.head_motion_time = self.motion_time
        self.l_arm_motion_time = self.motion_time
        self.r_arm_motion_time = self.motion_time

        # # Head Trajectory
        # self.head_point = JointTrajectoryPoint()
        # self.head_point.positions = [self.head_yaw, self.head_pitch]
        # self.head_point.time_from_start = rospy.Duration.from_sec(self.head_motion_time)
        # self.Head_msg.points = [self.head_point]

        # Left Arm Trajectory
        self.left_arm_point = JointTrajectoryPoint()
        self.left_arm_point.positions = [self.l_shoulder_pitch, self.l_shoulder_roll, self.l_elbow_yaw, self.l_elbow_roll, 0]
        self.left_arm_point.time_from_start = rospy.Duration.from_sec(self.l_arm_motion_time)
        self.LeftArm_msg.points = [self.left_arm_point]

        # Right Arm Trajectory
        self.right_arm_point = JointTrajectoryPoint()
        self.right_arm_point.positions = [self.r_shoulder_pitch, self.r_shoulder_roll, self.r_elbow_yaw, self.r_elbow_roll, 0]
        self.right_arm_point.time_from_start = rospy.Duration.from_sec(self.r_arm_motion_time)
        self.RightArm_msg.points = [self.right_arm_point]

        # Publish
        self.Head_pub.publish(self.Head_msg)
        self.RightArm_pub.publish(self.RightArm_msg)
        self.LeftArm_pub.publish(self.LeftArm_msg)

    def publishJointAnglesCommand(self):
        self.joint_angles_msg.joint_names = ['HeadYaw','HeadPitch','LShoulderPitch','LShoulderRoll','LElbowYaw','LElbowRoll','RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll']
        self.joint_angles_msg.joint_angles = [self.head_pitch, self.head_yaw, self.l_shoulder_pitch, self.l_shoulder_roll, self.l_elbow_yaw, self.l_elbow_roll, self.r_shoulder_pitch, self.r_shoulder_roll, self.r_elbow_yaw, self.r_elbow_roll]

    def jointStateCallback(self, msg):
        # Parse the JointState message and only take the values of interest
        # self.head_yaw_current = msg.position[msg.name.index("HeadYaw")]
        # self.head_pitch_current = msg.position[msg.name.index("HeadPitch")]
        self.l_shoulder_pitch_current = msg.position[msg.name.index("LShoulderPitch")]
        self.l_shoulder_roll_current = msg.position[msg.name.index("LShoulderRoll")]
        self.l_elbow_yaw_current = msg.position[msg.name.index("LElbowYaw")]
        self.l_elbow_roll_current = msg.position[msg.name.index("LElbowRoll")]
        self.r_shoulder_pitch_current = msg.position[msg.name.index("RShoulderPitch")]
        self.r_shoulder_roll_current = msg.position[msg.name.index("RShoulderRoll")]
        self.r_elbow_yaw_current = msg.position[msg.name.index("RElbowYaw")]
        self.r_elbow_roll_current = msg.position[msg.name.index("RElbowRoll")]

    def dynamicReconfigCallback(self, config, level):
        self.head_yaw = config.head_yaw
        self.head_pitch = config.head_pitch
        self.l_shoulder_pitch = config.l_shoulder_pitch
        self.l_shoulder_roll = config.l_shoulder_roll
        self.l_elbow_yaw = config.l_elbow_yaw
        self.l_elbow_roll = config.l_elbow_roll
        self.r_shoulder_pitch = config.r_shoulder_pitch
        self.r_shoulder_roll = config.r_shoulder_roll
        self.r_elbow_yaw = config.r_elbow_yaw
        self.r_elbow_roll = config.r_elbow_roll

        return config


if __name__ == '__main__':
    try:
        test_control = TestControl()
        test_control.spin()
    except rospy.ROSInterruptException:
        pass
