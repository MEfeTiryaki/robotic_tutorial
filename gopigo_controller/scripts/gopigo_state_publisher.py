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
#from sensor_msgs.msg import JointState

import tf
import tf.transformations as tr

import rospy
from ros_node_base_py.RosExecuterNodeBase import RosExecuterNodeBase

class GopigoStatePublisher(RosExecuterNodeBase):
    def create(self):
        super(RosExecuterNodeBase,self).create()
        self.Position_     = [0,0,0]
        self.Orientation_  = [1,0,0,0]
        self.Child_ = "base"
        self.Parent_ = "camera_link_optical"
        self.wheelDefaultPositions_ = [ 0.0 , 0.0 ]
        self.listener = tf.TransformListener()
        self.pos = np.array([0.0,0.0,0.0])
        self.euler =  np.array([0.0,0.0,0.0])
        self.rot =  np.array([1.0,0.0,0.0,0.0])
    def initilize(self):
        self.getTransforms()

    def initilizePublishers(self):
        self.poseStatePublisher_ =  rospy.Publisher(self.ns_+'/pose',PoseStamped,queue_size=10)

    def initilizeSubscribers(self):
        rospy.Subscriber("/aruco_football_node/pose_0", PoseStamped, callback =self.poseCallback)
        print("AA")

    def getTransforms(self):
        isTry = True
        trial = 0
        while isTry :
            try:
                #(trans,rot) = self.listener.lookupTransform('mobile_printer_0001_base_link', 'mobile_printer_0001_camera_link_optical', rospy.Time(0))
                self.listener.waitForTransform('/world','/camera_link_optical',rospy.Time(), rospy.Duration(4.0))
                (trans,rot) = self.listener.lookupTransform('/world','/camera_link_optical', rospy.Time(0))
                isTry = False
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            trial += 1
            if trial> 5:
                print("Couldnt find camera")
                break
        if trial < 5:
            self.cameraPositionToOdom = trans
            self.cameraOrientationToOdom = rot
        else :
            self.cameraPositionToOdom = np.array([0.0,0.0,0.0])
            self.cameraOrientationToOdom = np.array([1.0,0.0,0.0,0.0])
        self.T_OC = tr.quaternion_matrix(self.cameraOrientationToOdom)
        self.T_OC[0:3,3] = self.cameraPositionToOdom
        isTry = True
        trial = 0
        while isTry :
            try:
                #(trans,rot) = self.listener.lookupTransform('mobile_printer_0001_base_link', 'mobile_printer_0001_camera_link_optical', rospy.Time(0))
                self.listener.waitForTransform('/world','/camera_link_optical',rospy.Time(), rospy.Duration(4.0))
                (trans,rot) = self.listener.lookupTransform('/world','/camera_link_optical', rospy.Time(0))
                isTry = False
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            trial += 1
            if trial> 5:
                print("Couldnt find camera")
                break
        if trial < 5:
            self.cameraPositionToOdom = trans
            self.cameraOrientationToOdom = rot
        else :
            self.cameraPositionToOdom = np.array([0.0,0.0,0.0])
            self.cameraOrientationToOdom = np.array([1.0,0.0,0.0,0.0])

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
        T_CM = tr.quaternion_matrix(self.Orientation_)
        T_CM[0:3,3] = self.Position_
        T_MMR = np.array([[0.0, -1.0, 0.0 ,0.0]
                         ,[0.0, 0.0, 1.0,0.0]
                         ,[-1.0, 0.0, 0.0,0.0]
                         ,[0.0, 0.0, 0.0,1.0]]);
        T_MRR = np.array([[1.0, 0.0, 0.0 ,0.0]
                         ,[0.0, 1.0, 0.0 ,0.0]
                         ,[0.0, 0.0, 1.0 ,-0.05]
                         ,[0.0, 0.0, 0.0 ,1.0]]);

        T_OR = self.T_OC.dot(T_CM.dot(T_MMR))
        self.pos = T_OR[0:3,3]
        self.euler = tr.euler_from_matrix(T_OR)
        self.rot =  tr.quaternion_from_matrix(T_OR)



    def advance(self):
        #self.jointPublish()
        self.publish()


    def execute(self):
        self.frequency_ = 50
        self.rate_ = rospy.Rate(self.frequency_)
        while not rospy.is_shutdown():
          self.advance()
          self.rate_.sleep( )

    def publish(self):
        msg = PoseStamped()
        msg.pose.position.x = self.pos[0]
        msg.pose.position.y = self.pos[1]
        msg.pose.position.z = self.pos[2]
        msg.pose.orientation.x = self.rot[0]
        msg.pose.orientation.y = self.rot[1]
        msg.pose.orientation.z = self.rot[2]
        msg.pose.orientation.w = self.rot[3]
        self.poseStatePublisher_.publish(msg)
