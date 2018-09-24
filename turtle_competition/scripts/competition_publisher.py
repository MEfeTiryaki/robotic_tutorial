#!/usr/bin/env python
import rospy
from turtle_competition.msg import Competition_start


def send():
    publisher = rospy.Publisher('/best_turtle_ever/starter',Competition_start,queue_size=10)
    rospy.init_node('sender',anonymous=True)

    msg = Competition_start()
    msg.start = [10,10,1]
    msg.end = [1,0.5,1]
    publisher.publish(msg)

if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass
