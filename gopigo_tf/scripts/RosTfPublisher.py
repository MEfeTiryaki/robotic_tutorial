'''
    File name: RosTfPublisher.py
    Author: Mehmet Efe Tiryaki
    E-mail: m.efetiryaki@gmail.com
    Date created: 11.07.2018
    Date last modified: 11.07.2018
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

class RosTfPublisher(RosExecuterNodeBase):
    def create(self):
        super(RosExecuterNodeBase,self).create()
        self.tfParent_ = []
        self.tfChild_ = []
        self.tfPosition_ = []
        self.tfOrientation_ = []
        self.tfSourceName_ = []
        self.tfSourceQueueSize_ = []
        # Lock for callbacks
        self.lock_ = Lock()

    def readParameters(self):

        self.debug_ = rospy.get_param("tf_publisher/debug")
        self.frequency_ = rospy.get_param("tf_publisher/frequency")

        continueToRead = True
        i = 0
        while continueToRead:
            if rospy.has_param("tf_publisher/tf_publisher_" + str(i) + "/parent"):
                self.tfParent_.append(rospy.get_param("tf_publisher/tf_publisher_" + str(i) + "/parent"))
                self.tfChild_.append(rospy.get_param("tf_publisher/tf_publisher_" + str(i) + "/child"))
                self.tfPosition_.append(rospy.get_param("tf_publisher/tf_publisher_" + str(i) + "/position"))
                self.tfOrientation_.append(rospy.get_param("tf_publisher/tf_publisher_" + str(i) + "/orientation"))
                self.tfSourceName_.append(rospy.get_param("tf_publisher/tf_publisher_" + str(i) + "/source_topic"))
                self.tfSourceQueueSize_.append(rospy.get_param("tf_publisher/tf_publisher_" + str(i) + "/source_queue_size"))
            else:
                continueToRead = False
            i = i + 1

    def initilizePublishers(self):
        rospy.loginfo("[TfPublisher] : The Publishers are initilized")

    def initilizeSubscribers(self):
        self.poseSubscribers_ = []
        for i in range(0,len(self.tfSourceName_)):
            if self.tfSourceName_[i] != "None":
                self.poseSubscribers_.append(rospy.Subscriber( self.tfSourceName_[i]   \
                                          , PoseStamped                       \
                                          , callback = self.poseCallback \
                                          , callback_args =  i    \
                                          ))
                #rospy.loginfo("[TfPublisher] :" + self.tfSourceName_[i]  )
        rospy.loginfo("[TfPublisher] : The Subscribers are initilized")

    def initilizeServices(self):
        rospy.loginfo("[TfPublisher] : The Servises are initilized")

    def execute(self):
        self.rate_ = rospy.Rate(self.frequency_)
        while not rospy.is_shutdown():
          self.advance()
          self.rate_.sleep()

    def advance(self):
        with self.lock_:
            self.tfPublish()

    def tfPublish(self):
        currentTime = rospy.Time.now()
        for i in range(0,len(self.tfSourceName_)):
            #print(self.tfChild_[i],self.tfParent_[i])
            #print(self.tfPosition_[i])
            #print(self.tfOrientation_[i])
            br = tf.TransformBroadcaster()
            br.sendTransform( self.tfPosition_[i],
                              self.tfOrientation_[i],
                              currentTime,
                              self.tfChild_[i],
                              self.tfParent_[i])

    def poseCallback(self,msg,index):
        with self.lock_:
            self.tfPosition_[index]     = [0,0,0]
            self.tfOrientation_[index]  = [0,0,0,0]
            self.tfPosition_[index][0]     = msg.pose.position.x
            self.tfPosition_[index][1]     = msg.pose.position.y
            self.tfPosition_[index][2]     = msg.pose.position.z
            self.tfOrientation_[index][0]  = msg.pose.orientation.x
            self.tfOrientation_[index][1]  = msg.pose.orientation.y
            self.tfOrientation_[index][2]  = msg.pose.orientation.z
            self.tfOrientation_[index][3]  = msg.pose.orientation.w
            #print("index :" + str(index))
            #print("pos : " + str(self.tfPosition_[index]) + "\nori : " + str(self.tfOrientation_[index]) )
