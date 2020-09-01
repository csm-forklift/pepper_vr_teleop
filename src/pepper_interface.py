#!/usr/bin/env python
""" Handles direct communication with Pepper.

This node acts as the bridge between ROS commands and the Pepper SDK. Running
the "pepper_full_py.launch" file will establish a ROS connection to the full
suite of Pepper's sensors, but not all of the sensors are required for the VR
teleoperation functions. This node uses only the sensors required in order to
help reduce communication and to directly access the desired commands from the
Python SDK.

ROS Node Description
====================
Parameters
----------
~robot_ip : str, default: 138.67.198.34
    The IPv4 IP address of Pepper as a string. For ethernet the subnet is
    '169.254.X.X' and for wifi the subnet is '138.67.X.X'.
~robot_port : int, default: 9559
    The port used for TCP connection to Pepper. Pepper's default value is 9559.
~disable_external_collisions : bool, default: False
    Whether to disable the external collision protection, defaults to False for
    safety reasons. This can only be done after allowing this action in
    Pepper's advanced settings. This is accessed through 'http://<robot
    ip>/advanced/#/settings'
~command_duration : float, default: 1.0
    How long the velocity command should drive the base after receiving a
    message on the topic. If no new message is received after this time, the
    velocity is set to 0.
~use_camera : bool, default: True
    Whether to subscribe to Pepper's camera and publish the images.
~image_topic : str, default: '/pepper_interface/camera/front/image_raw'
    The name of the topic to publish Pepper's camera images on.
~frame_rate : int, default: 30
    The desired frame rate to receive images from Pepper's camera.

Published Topics
----------------
<~image_topic> : sensor_msgs/Image
    The image from Pepper's camera. The default topic name is
    '/pepper_interface/camera/front/image_raw'.

Subscribed Topics
-----------------
~joint_angles : naoqi_bridge_msgs/JointAnglesWithSpeed
    Reads the joint angle setpoints and desired speed fraction and sends the
    command to Pepper to move to those angles using the Python SDK.
~cmd_vel : geometry_msgs/Twist
    Reads the desired (x,y) linear velocities and the (theta) angular velocity
    and sends the command to move Pepper's base using the Python SDK. The
    command will last for <command_duration> unless a new message is received.
~grasp/left : std_msgs/Float64
    Controls the grasp position of the left hand. 1 will close the hand, 0 will
    open it.
~grasp/right : std_msgs/Float64
    Controls the grasp position of the right hand. 1 will close the hand, 0 will
    open it.
"""

# NaoQI SDK
from naoqi import ALProxy

# ROS
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

# Python
import numpy as np
from copy import deepcopy


class PepperInterface():
    """ ROS node class for interfacing with Pepper. """
    def __init__(self):
        #===== ROS Setup =====#
        rospy.init_node('pepper_interface')
        rospy.on_shutdown(self.shutdown)

        #----- Parameters -----#
        # Loop Rate (for camera image processing)
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        # Pepper Connection
        self.robot_ip = rospy.get_param('~robot_ip', '138.67.198.34')
        self.robot_port = rospy.get_param('~robot_port', 9559)

        # Locomotion
        self.disable_external_collisions = rospy.get_param('~disable_external_collisions', False)
        self.grasp_threshold = 0.5 # trigger value when hand closes
        # Hand grasp: False = open, True = closed
        self.left_grasp_prev = False
        self.right_grasp_prev = False
        self.command_duration = rospy.get_param('~command_duration', 1.0)
        self.command_start_time = 0.0
        self.x_velocity = 0.0
        self.y_velocity = 0.0
        self.theta_velocity = 0.0

        #--- Camera
        self.use_camera = rospy.get_param('~use_camera', True)
        self.image_topic = rospy.get_param('~image_topic', '/pepper_interface/camera/front/image_raw')
        self.frame_rate = rospy.get_param('~frame_rate', 30)
        # The following parameters are specified in the Pepper SDK documentation
        # and used with the 'ALVideoDevice' module.
        # See: http://doc.aldebaran.com/2-5/family/pepper_technical/video_2D_pep_v18a.html
        # ColorSpace
        # RGB: 11
        # BGR: 13
        self.color_space = 13 # opencv uses BGR as default
        # Resolution
        # 320x240 (1-30fps): 1
        # 640x480 (1-30fps): 2
        # 1280x960   (1fps): 3
        self.resolution = 2
        # Camera Index
        # front top:    0
        # front bottom: 1
        # depth:        2
        self.camera_index = 0
        self.previous_image = 12*[0]
        self.bridge = CvBridge()

        #--- Connect to Pepper
        self.motion_proxy = ALProxy('ALMotion', self.robot_ip, self.robot_port)
        if self.use_camera:
            self.camera_proxy = ALProxy('ALVideoDevice', self.robot_ip, self.robot_port)
            self.camera_sub = self.camera_proxy.subscribeCamera('camera_front', self.camera_index, self.resolution, self.color_space, self.frame_rate)
            rospy.loginfo('[{0}]: subscribed to front camera ({1})'.format(rospy.get_name(), self.camera_sub))

        #--- Turn off the External Collision Detection
        if self.disable_external_collisions:
            rospy.loginfo('[{0}]: Turning off external collision detection for base'.format(rospy.get_name()))
            try:
                self.motion_proxy.setExternalCollisionProtectionEnabled('Move', False)
            except:
                rospy.logwarn('[{0}]: Error turning off external collision detection for base. Base may have trouble moving if it thinks there is an obstacle nearby. Type the following URL into a web browser and check the settings to see if disabling this feature is allowed: "http://<Pepper\'s IP>/advanced/#/settings"'.format(rospy.get_name()))
            rospy.loginfo('[{0}]: Turning off external collision detection for arms'.format(rospy.get_name()))
            try:
                self.motion_proxy.setExternalCollisionProtectionEnabled('Arms', False)
            except:
                rospy.logwarn('[{0}]: Error turning off external collision detection for the arms. The arms may have trouble moving if it thinks there is an obstacle nearby.'.format(rospy.get_name()))

        #--- Turn off AutonomousLife functionality
        self.autonomouslife_proxy = ALProxy('ALAutonomousLife', self.robot_ip, self.robot_port)
        rospy.loginfo('[{0}]: Turning off AutonomousLife functionality.'.format(rospy.get_name()))
        self.autonomouslife_proxy.setAutonomousAbilityEnabled('All', False)

        #----- Publishers -----#
        if self.use_camera:
            self.image_pub = rospy.Publisher('{0}'.format(self.image_topic), Image, queue_size=1)

        #----- Subscribers -----#
        self.joint_angles_sub = rospy.Subscriber('~joint_angles', JointAnglesWithSpeed, self.jointAnglesCallback, queue_size=3)
        self.cmd_vel_sub = rospy.Subscriber('~cmd_vel', Twist, self.cmdVelCallback, queue_size=3)
        self.left_hand_grasp_sub = rospy.Subscriber('~grasp/left', Float64, self.leftGraspCallback, queue_size=3)
        self.right_hand_grasp_sub = rospy.Subscriber('~grasp/right', Float64, self.rightGraspCallback, queue_size=3)

    def spin(self):
        """ Process loop: reads image, sends motion commands. """
        while not rospy.is_shutdown():
            #===== Send the velocity command =====#
            if (rospy.get_time() - self.command_start_time) < self.command_duration:
                self.motion_proxy.move(self.x_velocity, self.y_velocity, self.theta_velocity)
            else:
                self.motion_proxy.move(0.0, 0.0, 0.0)

            #===== Read the camera image =====#
            if self.use_camera:
                # Get image
                # the output is a list of image values as described here:
                # http://doc.aldebaran.com/2-5/naoqi/vision/alvideodevice-api.html#image
                # The important indices are:
                # 0: width
                # 1: height
                # 4: time stamp seconds
                # 5: time stamp microseconds
                # 6: binary array containing image
                self.image = self.camera_proxy.getImageRemote(self.camera_sub)

                if (self.image is not None):
                    # Convert to OpenCV format
                    height = self.image[1]
                    width = self.image[0]
                    byte_image = bytearray(self.image[6])
                    cv_image = np.array(byte_image).reshape(height, width, 3)

                    # Convert to ROS message
                    ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

                    # Publish
                    self.image_pub.publish(ros_image)

                    # Update previous image
                    self.previous_image = deepcopy(self.image)

            self.rate.sleep()

    def jointAnglesCallback(self, msg):
        """ Reads joint angles passes them to Pepper using the SDK function. """
        self.motion_proxy.setAngles(msg.joint_names, msg.joint_angles, msg.speed)

    def cmdVelCallback(self, msg):
        """ Base velocity command callback.

        Reads the Twist message and sends the linear X and Y and angular Z
        values to Pepper through the SDK. Commands are run for
        "command_duration" seconds. Everytime this function is called it resets
        the timer.
        """
        # Restart the timer
        self.command_start_time = rospy.get_time()
        self.x_velocity = msg.linear.x
        self.y_velocity = msg.linear.y
        self.theta_velocity = msg.angular.z

    def leftGraspCallback(self, msg):
        """ Sets the grasp position for the left hand. """
        if (msg.data >= self.grasp_threshold):
            self.motion_proxy.closeHand('LHand')
            self.left_grasp_prev = True
        elif (msg.data < self.grasp_threshold and self.left_grasp_prev):
            self.motion_proxy.openHand('LHand')
            self.left_grasp_prev = False

    def rightGraspCallback(self, msg):
        """ Sets the grasp position for the right hand. """
        if (msg.data >= self.grasp_threshold):
            self.motion_proxy.closeHand('RHand')
            self.right_grasp_prev = True
        elif (msg.data < self.grasp_threshold and self.right_grasp_prev):
            self.motion_proxy.openHand('RHand')
            self.right_grasp_prev = False

    def shutdown(self):
        """ Closes camera and restores collision protection and autonomous
        life on Pepper.
        """
        if self.use_camera:
            # Close camera connection
            rospy.loginfo('[{0}]: Closing camera subscriber ({1})'.format(rospy.get_name(), self.camera_sub))
            self.camera_proxy.unsubscribe(self.camera_sub)

        # Restore external collision protection
        rospy.loginfo('[{0}]: Turning external collision protection back on.'.format(rospy.get_name()))
        try:
            self.motion_proxy.setExternalCollisionProtectionEnabled('All', True)
        except:
            rospy.logwarn('[{0}]: Failed to turn external collision protection back on. Please try to set this manually.'.format(rospy.get_name()))

        # Restore AutonomousLife
        rospy.loginfo('[{0}]: Turning AutonomousLife back on.'.format(rospy.get_name()))
        self.autonomouslife_proxy.setAutonomousAbilityEnabled('All', True)

if __name__ == '__main__':
    try:
        pepper_interface = PepperInterface()
        pepper_interface.spin()
    except rospy.ROSInterruptException:
        pass
