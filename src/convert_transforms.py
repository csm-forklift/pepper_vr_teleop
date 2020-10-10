#!/usr/bin/env python
""" Converts the Kinect joint frames from OpenNI into Pepper's orientation.

The OpenNI skeleton tracker broadcasts its frames using a different standard
than Pepper. The torso and joint frames are defined from the camera's viewpoint
with the X axis pointing LEFT, the Y axis pointing UP, and the Z axis pointing
OUT. The end effectors (hands and feet) have frames defined from the camera's
viewpoint with the X axis pointing RIGHT, the Y axis pointing DOWN, and the Z
axis pointing OUT. These frames need to be rotated to match the standard
robotics orientation used by Pepper where the X axis points FORWARD, the Y axis
points LEFT, and the Z axis points UP. This node reads the OpenNI frames and
rotates them to match Pepper's frames and then re-broadcasts them.

ROS Node Description
====================
Parameters
----------
~main_frame : str, default: 'camera_link'
    The name of the main parent frame of the TF tree being broadcast by OpenNI.
    It should be the name of the camera frame.
~publish_skeleton : bool, default: True
    Whether to publish the skeleton visualization markers for Rviz. This allows
    for easy viewing. Otherwise they are simply viewed as transform frames.

Published Topics
----------------
visualization_marker : visualization_msgs/Marker
    Publishes the skeleton as a linepath visualization marker for Rviz.

Subscribed Topics
-----------------
None (Uses tf.TransformListener)
"""

import math
from geometry_msgs.msg import Point, Vector3
import rospy
import sys, signal # used for ctrl-c shutdown while waiting for transform
from std_msgs.msg import ColorRGBA
import tf
from visualization_msgs.msg import Marker


class Transform:
    """ Member variables defining a transformation frame.

    This class acts as a "struct" containing the main pieces needed to define
    each transform. The position is the (x,y.z) position of the origin in the
    parent frame. The rotation is the orientation of the frame represented as a
    quaternion (x,y,z,w). The parent is a string representing the parent frame
    of the transform.
    """
    def __init__(self, position=[0,0,0], rotation=[0,0,0,1], name='frame', parent='parent'):
        self.position = position
        self.rotation = rotation
        self.name = name
        self.parent = parent

class ConvertTransforms:
    def __init__(self):
        #======================================================================#
        # ROS Setup
        #======================================================================#
        rospy.init_node("convert_transforms")

        #===== Parameters =====#
        # Publishing frequency
        self.frequency = 30 # Hz
        self.rate = rospy.Rate(self.frequency)

        # Transform Variables
        # The main camera frame from which all the joints stem
        self.main_parent = rospy.get_param('~main_frame', 'camera_link')
        self.base_link_name = 'base_link_K'

        # Set to "True" to publish the skeleton in rviz
        self.publish_skeleton = rospy.get_param('~publish_skeleton', True)

        # NOTE: OpenNI uses frames facing outward from the Kinect. So the
        # "left_shoulder_1" frame is actually the "right" arm of the operator,
        # which corresponds to the right arm of Pepper, "RShoulder_K".
        self.openni_frames = ['left_shoulder_1', 'left_elbow_1', 'left_hand_1', 'right_shoulder_1', 'right_elbow_1', 'right_hand_1', 'head_1']
        self.pepper_frames = ['RShoulder_K', 'RElbow_K', 'RHand_K', 'LShoulder_K', 'LElbow_K', 'LHand_K', 'Head_K'] # used pepper frame names with "_K" appended for "Kinect"
        self.transforms = {}
        for i in range(len(self.pepper_frames)):
            self.transforms[self.pepper_frames[i]] = Transform([0,0,0], [0,0,0,1], self.pepper_frames[i], self.base_link_name)
        self.euler_angles = {self.pepper_frames[0]: [-math.pi/2, 0, 0, 'rxyz'],
                             self.pepper_frames[1]: [-math.pi/2, 0, 0, 'rxyz'],
                             self.pepper_frames[2]: [-math.pi/2, math.pi/2, 0, 'rzyx'],
                             self.pepper_frames[3]: [math.pi/2, math.pi, 0, 'rxyz'],
                             self.pepper_frames[4]: [math.pi/2, math.pi, 0, 'rxyz'],
                             self.pepper_frames[5]: [-math.pi/2, math.pi/2, 0, 'rzyx'],
                             self.pepper_frames[6]: [math.pi/2, math.pi/2, 0, 'rzyx']}

        #===== Transform Listener/Broadcaster =====#
        self.tfListener = tf.TransformListener()
        self.tfBroadcaster = tf.TransformBroadcaster()

        #===== Publishers =====#
        # Skeleton publisher
        if self.publish_skeleton:
            self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        #======================================================================#
        # Setup "base_link" For New TF Tree
        #======================================================================#
        # Wait for torso frame from Kinect transforms
        # I have to use a "signal" here because the "ctrl+c" sequence does not
        # work if ROS is using simulation time.
        signal.signal(signal.SIGINT, self.shutdown)
        message_delay = 10 # seconds to delay before displaying error again
        start_time = rospy.Time.now()
        transform_received = False
        while not transform_received:
            if self.tfListener.frameExists('torso_1'):
                transform_received = True
            if rospy.Time.now() > start_time + rospy.Duration.from_sec(message_delay):
                start_time = rospy.Time.now()
                rospy.logwarn('[{0}]: Have not received transform for \'torso_1\'. Make sure \'openni_tracker\' is running.'.format(rospy.get_name()))
            self.rate.sleep()

        position, rotation = self.tfListener.lookupTransform(self.main_parent, 'torso_1', rospy.Time())

        # Generate the new rotation matrix
        R = tf.transformations.quaternion_matrix(rotation)
        R_new = tf.transformations.euler_matrix(math.pi/2, math.pi/2, 0, 'rzyx')
        R_base_link = R.dot(R_new)

        # Convert to quaternion and save new transform data
        q_base_link = tf.transformations.quaternion_from_matrix(R_base_link)
        self.base_link = Transform(position, q_base_link, self.base_link_name, self.main_parent)

        # Send initial broadcast so later transforms can be built off of
        # "base_link"
        self.tfBroadcaster.sendTransform(self.base_link.position, self.base_link.rotation, rospy.Time.now(), self.base_link.name, self.base_link.parent)

    #==========================================================================#
    # Main Process
    #==========================================================================#
    def spin(self):
        """ Process loop: read transforms, generate new transforms, broadcast.
        """
        while not rospy.is_shutdown():
            if self.updateTransforms():
                self.publishTransforms()
                if self.publish_skeleton:
                    self.publishSkeletonMarker()
            self.rate.sleep()

    def updateTransforms(self):
        """ Updates the transforms based on new data from /tf.

        This function reads the transforms coming in through the /tf topic. It
        applies appropriate rotation to update the frames and stores the data in
        member variables. It does not publish the new transforms. If there are
        any errors, the function is exited and returns "False".

        Returns
        -------
        bool
            True if the transforms were updated. False if there were any errors.
        """
        try:
            #===== Update base_link =====#
            self.tfListener.waitForTransform(self.main_parent, 'torso_1', rospy.Time(), rospy.Duration.from_sec(1))
            position, rotation = self.tfListener.lookupTransform(self.main_parent, 'torso_1', rospy.Time())

            # Generate the new rotation matrix
            R = tf.transformations.quaternion_matrix(rotation)
            R_new = tf.transformations.euler_matrix(math.pi/2, math.pi/2, 0, 'rzyx')
            R_base_link = R.dot(R_new)

            # Convert to quaternion and save new transform data
            self.base_link.position = position
            self.base_link.rotation = tf.transformations.quaternion_from_matrix(R_base_link)

            #===== Publish base_link =====#
            # base_link must be updated before the other transforms can be updated
            self.tfBroadcaster.sendTransform(self.base_link.position, self.base_link.rotation, rospy.Time.now(), self.base_link.name, self.base_link.parent)

            #===== Rotate arm joints to new frame =====#
            for i in range(len(self.pepper_frames)):
                position, rotation = self.tfListener.lookupTransform(self.base_link.name, self.openni_frames[i], rospy.Time())
                R = tf.transformations.quaternion_matrix(rotation)
                R_new = tf.transformations.euler_matrix(self.euler_angles[self.pepper_frames[i]][0], self.euler_angles[self.pepper_frames[i]][1], self.euler_angles[self.pepper_frames[i]][2], self.euler_angles[self.pepper_frames[i]][3])
                R_joint = R.dot(R_new)
                self.transforms[self.pepper_frames[i]].position = position
                self.transforms[self.pepper_frames[i]].rotation = tf.transformations.quaternion_from_matrix(R_joint)

            # Success
            return True
        except tf.Exception as err:
            print "[convert_transforms]: Transform read error: {0}".format(err)
            # Failure
            return False

    def publishTransforms(self):
        """ Publishes the updated transforms. """
        #===== Publish arm joints =====#
        for i in range(len(self.pepper_frames)):
            self.tfBroadcaster.sendTransform(self.transforms[self.pepper_frames[i]].position, self.transforms[self.pepper_frames[i]].rotation, rospy.Time.now(), self.transforms[self.pepper_frames[i]].name, self.base_link.name)

    def publishSkeletonMarker(self):
        """ Publishes an Rviz visualization of the skeleton.

        All of the newly created transforms are read on the /tf topic in order
        to obtain their positions. The positions are used as points in a
        "linepath" marker for easily visualizing the torso and arms in Rviz
        using lines instead of frame axes.
        """
        #===== Read Transforms =====#
        # Use lookupTransform for each important joint: baselink, shoulders,
        # elbows, hands
        self.tfListener.waitForTransform(self.base_link_name, 'LShoulder_K', rospy.Time(), rospy.Duration.from_sec(1.0))
        l_shoulder_position, _ = self.tfListener.lookupTransform(self.base_link_name, 'LShoulder_K', rospy.Time())
        self.tfListener.waitForTransform(self.base_link_name, 'LElbow_K', rospy.Time(), rospy.Duration.from_sec(1.0))
        l_elbow_position, _ = self.tfListener.lookupTransform(self.base_link_name, 'LElbow_K', rospy.Time())
        self.tfListener.waitForTransform(self.base_link_name, 'LHand_K', rospy.Time(), rospy.Duration.from_sec(1.0))
        l_hand_position, _ = self.tfListener.lookupTransform(self.base_link_name, 'LHand_K', rospy.Time())
        self.tfListener.waitForTransform(self.base_link_name, 'RShoulder_K', rospy.Time(), rospy.Duration.from_sec(1.0))
        r_shoulder_position, _ = self.tfListener.lookupTransform(self.base_link_name, 'RShoulder_K', rospy.Time())
        self.tfListener.waitForTransform(self.base_link_name, 'RElbow_K', rospy.Time(), rospy.Duration.from_sec(1.0))
        r_elbow_position, _ = self.tfListener.lookupTransform(self.base_link_name, 'RElbow_K', rospy.Time())
        self.tfListener.waitForTransform(self.base_link_name, 'RHand_K', rospy.Time(), rospy.Duration.from_sec(1.0))
        r_hand_position, _ = self.tfListener.lookupTransform(self.base_link_name, 'RHand_K', rospy.Time())

        #===== Create Visualization Message =====#
        # Generate "Point" objects using joint positions
        l_shoulder_point = Point(l_shoulder_position[0], l_shoulder_position[1], l_shoulder_position[2])
        l_elbow_point = Point(l_elbow_position[0], l_elbow_position[1], l_elbow_position[2])
        l_hand_point = Point(l_hand_position[0], l_hand_position[1], l_hand_position[2])
        r_shoulder_point = Point(r_shoulder_position[0], r_shoulder_position[1], r_shoulder_position[2])
        r_elbow_point = Point(r_elbow_position[0], r_elbow_position[1], r_elbow_position[2])
        r_hand_point = Point(r_hand_position[0], r_hand_position[1], r_hand_position[2])

        # Create torso line marker
        torso_marker_msg = Marker()
        torso_marker_msg.header.frame_id = self.base_link_name
        torso_marker_msg.header.stamp = rospy.Time()
        torso_marker_msg.id = 0
        torso_marker_msg.type = Marker.LINE_STRIP
        torso_marker_msg.action = Marker.ADD
        torso_marker_msg.scale = Vector3(0.01,1,1)
        torso_marker_msg.color = ColorRGBA(0,1,0,1)
        torso_marker_msg.frame_locked = True
        torso_marker_msg.points = [Point(0,0,0), l_shoulder_point, r_shoulder_point, Point(0, 0, 0)]

        # Create left arm line marker
        l_arm_marker_msg = Marker()
        l_arm_marker_msg.header.frame_id = self.base_link_name
        l_arm_marker_msg.header.stamp = rospy.Time()
        l_arm_marker_msg.id = 1
        l_arm_marker_msg.type = Marker.LINE_STRIP
        l_arm_marker_msg.action = Marker.ADD
        l_arm_marker_msg.scale = Vector3(0.01,1,1)
        l_arm_marker_msg.color = ColorRGBA(0,1,0,1)
        l_arm_marker_msg.frame_locked = True
        l_arm_marker_msg.points = [l_shoulder_point, l_elbow_point, l_hand_point]

        # Create right arm line marker
        r_arm_marker_msg = Marker()
        r_arm_marker_msg.header.frame_id = self.base_link_name
        r_arm_marker_msg.header.stamp = rospy.Time()
        r_arm_marker_msg.id = 2
        r_arm_marker_msg.type = Marker.LINE_STRIP
        r_arm_marker_msg.action = Marker.ADD
        r_arm_marker_msg.scale = Vector3(0.01,1,1)
        r_arm_marker_msg.color = ColorRGBA(0,1,0,1)
        r_arm_marker_msg.frame_locked = True
        r_arm_marker_msg.points = [r_shoulder_point, r_elbow_point, r_hand_point]

        # Publish the marker messages
        self.marker_pub.publish(torso_marker_msg)
        self.marker_pub.publish(l_arm_marker_msg)
        self.marker_pub.publish(r_arm_marker_msg)

    def shutdown(self, signum, frame):
        """ Callback function for shutting down.

        In the case that ROS is using simulation time, if the user tries to
        'ctrl-c' during a while loop with a rospy.Rate().sleep() function, it
        will not exit. A Python 'signal' is used to then call this function to
        shutdown ROS.
        """
        rospy.signal_shutdown("No tf for 'torso_1', keyboard interrupt")
        sys.exit(1)

if __name__ == "__main__":
    try:
        convert_transforms = ConvertTransforms()
        convert_transforms.spin()
    except rospy.ROSInterruptException:
        pass
