#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

class Sender:
    def __init__(self,nodeName,topicName = "message",rate = 10):
        self.nodeName_ = nodeName
        self.topicName_ = topicName
        self.initilizePublishers()
        rospy.init_node(self.nodeName_,anonymous=True)
        self.rate_ = rospy.Rate(rate)

    def initilizePublishers(self):
        self.publisher_ = rospy.Publisher(self.topicName_,String,queue_size=10)

    def loop(self):
        while not rospy.is_shutdown():
            self.publisher_.publish("["+self.nodeName_+"] : "+"time is " + str(rospy.get_time()))
            self.rate_.sleep()

if __name__ == '__main__':
    try:
        sender = Sender("the_mighty_one")
        sender.loop()
    except rospy.ROSInterruptException:
        pass
