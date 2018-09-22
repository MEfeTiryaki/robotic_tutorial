#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64


class Receiver:
    def __init__(self,nodeName,topicName = "message",rate = 10):
        rospy.init_node(nodeName, anonymous=True)
        self.stringSubscriber_ = rospy.Subscriber(topicName, String \
                , callback = self.callback , callback_args =  1)
        rospy.spin()

    def callback(self,data,index):
        rospy.loginfo(data.data)


if __name__ == '__main__':
    receiver = Receiver("the_mighty_two")
