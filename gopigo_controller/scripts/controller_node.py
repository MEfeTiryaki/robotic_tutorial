'''
    File name: RosTfPublisher.py
    Author: Mehmet Efe Tiryaki
    E-mail: m.efetiryaki@gmail.com
    Date created: 24.09.2018
    Date last modified: 24.09.2018
    Python Version: 2.7
'''
import sys
sys.dont_write_bytecode = True


import numpy as np
from threading import Lock
import time

from geometry_msgs.msg import PoseStamped

import tf
import tf.transformations as tr

import rospy
from ros_node_base_py.RosExecuterNodeBase import RosExecuterNodeBase

class ControllerNode(RosExecuterNodeBase):
    def create(self):
        super(RosExecuterNodeBase,self).create()
        self.Position_     = [0,0,0]
        self.Orientation_  = [0,0,0,0]
        self.Child_ = "base"
        self.Parent_ = "camera_link_optical"
    def readParameters(self):
        pass
    def initilizePublishers(self):
        pass

    def initilizeSubscribers(self):
        rospy.Subscriber("/aruco_multi/pose_0", PoseStamped, callback =self.poseCallback)

    def getTransforms(self):
        # TODO : get the transform  T_OC from odom to camera_link_optical

    def poseCallback(self,msg):
        # TODO : Build the tranform T_CM from camera_link_optical to marker using
        # msg. Then define a transform T_MR from Marker to robot
        # Then the transform odom to robot  is T_OR = T_OC * T_CM * T_MR
        # Finally extract position and orientation(as quaternion) and assign them

        # XXX: Correct Following lines
        self.Position_     = [0,0,0]
        self.Orientation_  = [0,0,0,0]
        self.Position_[0]     = msg.pose.position.x
        self.Position_[1]     = msg.pose.position.y
        self.Position_[2]     = msg.pose.position.z
        self.Orientation_[0]  = msg.pose.orientation.x
        self.Orientation_[1]  = msg.pose.orientation.y
        self.Orientation_[2]  = msg.pose.orientation.z
        self.Orientation_[3]  = msg.pose.orientation.w



    def advance(self):
        # TODO : Write your control cycle here

        self.tfPublish()


    def execute(self):
        self.frequency_ = 50
        self.rate_ = rospy.Rate(self.frequency_)
        while not rospy.is_shutdown():
          self.advance()
          self.rate_.sleep()
    def tfPublish(self):
        currentTime = rospy.Time.now()
        br = tf.TransformBroadcaster()
        br.sendTransform( self.Position_,
                          self.Orientation_,
                          currentTime,
                          self.Child_,
                          self.Parent_)
