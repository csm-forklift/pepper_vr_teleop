#!/usr/bin/env python
'''
Converts a bag file containing the transforms for the joints as put out by
convert_transforms.py into points in the camera_link frame in a 'csv' file.
'''
import csv

import rospy
import tf

class ConvertPoints():
    def __init__(self):
        rospy.init_node("convert_points")
        rospy.on_shutdown(self.shutdown)
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        # Create transform listener for obtaining the positions
        self.tfListener = tf.TransformListener()

        # Create a file for storing the converted data
        self.path = '/home/kyle/pepper_vr_ws/src/pepper_vr_teleop/bags/'
        self.filename = self.path + 'test_pose2.csv'
        self.file = open(self.filename, 'w')
        # Set the column headers
        self.file.write('time,bx,by,bz,lsx,lsy,lsz,lex,ley,lez,lhx,lhy,lhz,rsx,rsy,rsz,rex,rey,rez,rhx,rhy,rhz\n')

    def getFirstPositions(self):
        # Grab the positions for the first time
        self.tfListener.waitForTransform('camera_link', 'base_link_K', rospy.Time(), rospy.Duration.from_sec(10))
        self.base_link_position, _ = self.tfListener.lookupTransform('camera_link', 'base_link_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'LShoulder_K', rospy.Time(), rospy.Duration.from_sec(10))
        self.left_shoulder_position, _ = self.tfListener.lookupTransform('camera_link', 'LShoulder_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'LElbow_K', rospy.Time(), rospy.Duration.from_sec(10))
        self.left_elbow_position, _ = self.tfListener.lookupTransform('camera_link', 'LElbow_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'LHand_K', rospy.Time(), rospy.Duration.from_sec(10))
        self.left_hand_position, _ = self.tfListener.lookupTransform('camera_link', 'LHand_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'RShoulder_K', rospy.Time(), rospy.Duration.from_sec(10))
        self.right_shoulder_position, _ = self.tfListener.lookupTransform('camera_link', 'RShoulder_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'RElbow_K', rospy.Time(), rospy.Duration.from_sec(10))
        self.right_elbow_position, _ = self.tfListener.lookupTransform('camera_link', 'RElbow_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'RHand_K', rospy.Time(), rospy.Duration.from_sec(10))
        self.right_hand_position, _ = self.tfListener.lookupTransform('camera_link', 'RHand_K', rospy.Time())

    def getPositions(self):
        self.tfListener.waitForTransform('camera_link', 'base_link_K', rospy.Time(), rospy.Duration.from_sec(1))
        self.base_link_position, _ = self.tfListener.lookupTransform('camera_link', 'base_link_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'LShoulder_K', rospy.Time(), rospy.Duration.from_sec(1))
        self.left_shoulder_position, _ = self.tfListener.lookupTransform('camera_link', 'LShoulder_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'LElbow_K', rospy.Time(), rospy.Duration.from_sec(1))
        self.left_elbow_position, _ = self.tfListener.lookupTransform('camera_link', 'LElbow_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'LHand_K', rospy.Time(), rospy.Duration.from_sec(1))
        self.left_hand_position, _ = self.tfListener.lookupTransform('camera_link', 'LHand_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'RShoulder_K', rospy.Time(), rospy.Duration.from_sec(1))
        self.right_shoulder_position, _ = self.tfListener.lookupTransform('camera_link', 'RShoulder_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'RElbow_K', rospy.Time(), rospy.Duration.from_sec(1))
        self.right_elbow_position, _ = self.tfListener.lookupTransform('camera_link', 'RElbow_K', rospy.Time())
        self.tfListener.waitForTransform('camera_link', 'RHand_K', rospy.Time(), rospy.Duration.from_sec(1))
        self.right_hand_position, _ = self.tfListener.lookupTransform('camera_link', 'RHand_K', rospy.Time())

        self.time = rospy.get_time()

    def writePositions(self):
        self.file.write('%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f\n' % (self.time,
        self.base_link_position[0],
        self.base_link_position[1],
        self.base_link_position[2],
        self.left_shoulder_position[0],
        self.left_shoulder_position[1],
        self.left_shoulder_position[2],
        self.left_elbow_position[0],
        self.left_elbow_position[1],
        self.left_elbow_position[2],
        self.left_hand_position[0],
        self.left_hand_position[1],
        self.left_hand_position[2],
        self.right_shoulder_position[0],
        self.right_shoulder_position[1],
        self.right_shoulder_position[2],
        self.right_elbow_position[0],
        self.right_elbow_position[1],
        self.right_elbow_position[2],
        self.right_hand_position[0],
        self.right_hand_position[1],
        self.right_hand_position[2]))

    def spin(self):
        self.waiting_for_transform = True
        print("Waiting for transforms")
        while self.waiting_for_transform:
            try:
                self.getFirstPositions()
                self.waiting_for_transform = False
            except tf.Exception as e:
                print("Transform error: {0}".format(e))
        print("Transforms received")

        while not rospy.is_shutdown():
            self.getPositions()
            self.writePositions()
            self.rate.sleep()

    def shutdown(self):
        print("Closing file")
        self.file.close()

if __name__ == '__main__':
    try:
        convert_points = ConvertPoints()
        convert_points.spin()
    except rospy.ROSInterruptException:
        pass
