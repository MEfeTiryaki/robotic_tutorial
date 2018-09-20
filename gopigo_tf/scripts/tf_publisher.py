import rospy
import numpy as np

from threading import Lock
import time

from geometry_msgs.msg import PoseStamped
import tf
import tf.transformations as tr


class TfPublisher(object):
    """
        This class localizes the robot

        Attributes:

        Services:

        Publishers:

        Subscribers:

    """


    def __init__(self,nodeName):
        self.nodeName_ = nodeName

        # initize class variables
        self.initilize()

        # Reading parameters
        self.readParameters()
        # Setting debug
        self.debug_ = rospy.DEBUG if self.debug_ else None

        # ROS Node initilization
        rospy.init_node(self.nodeName_,log_level=self.debug_)

        # init Publisher
        self.initilizePublishers()
        # init Subscribers
        self.initilizeSubscribers()
        # init Services
        self.initilizeServices()

        rospy.loginfo("[TfPublisher] : The ROS Node for tf publisher is initilized.")
        self.execute()

    def initilize(self):
        self.tfParent_ = []
        self.tfChild_ = []
        self.tfPosition_ = []
        self.tfOrientation_ = []
        self.tfSourceName_ = []
        self.tfSourceQueueSize_ = []
        # Lock for callbacks
        self.lock_ = Lock()


    def execute(self):
        # TODO : Make Parameteric
        self.rate_ = rospy.Rate(self.frequency_)
        while not rospy.is_shutdown():
          self.advance()
          self.rate_.sleep()

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
        """ Initlize the Publishers
            Publishers:
        """
        rospy.loginfo("[TfPublisher] : The Publishers are initilized")

    def initilizeSubscribers(self):
        """ Initlize the Subscriptions
            Subscribers:
                markerPositionSubscriber_ : /aruco_single/pose
        """
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
        """ Initlize the Services
            Servises:
        """
        rospy.loginfo("[TfPublisher] : The Servises are initilized")

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

    def advance(self):
        """ Advances the state estimator one step in time
            Note : For now it waits the input and measurement. But
                it should also work with out measurement

        """
        with self.lock_:
            self.tfPublish()
