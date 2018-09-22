#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


def send():
    publisher = rospy.Publisher('message',String,queue_size=10)
    #publisher = rospy.Publisher('message',Float64,queue_size=10)
    rospy.init_node('sender',anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publisher.publish("[sender] : time is " + str(rospy.get_time()))
        #publisher.publish(rospy.get_time())
        rate.sleep()

if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass
