#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

def callback(data):
    #import IPython; IPython.embed();
    rospy.loginfo(data.data)

def receiver():
    rospy.init_node('receiver', anonymous=True)
    #rospy.Subscriber("message", String, callback)
    rospy.Subscriber("message", Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    receiver()
