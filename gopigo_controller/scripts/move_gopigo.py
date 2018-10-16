#!/usr/bin/env python


import sys
import rospy

from std_msgs.msg import Int8

def move(v,w):
    rospy.init_node('gopigo_mover',anonymous=True)
    left_publisher = rospy.Publisher('/motor/pwm/left',Int8,queue_size=10)
    right_publisher = rospy.Publisher('/motor/pwm/right',Int8,queue_size=10)

    scale = 200
    v_l = int(scale*(v + w)/2)
    v_r = int(scale*(v - w)/2)
    print(v_l,v_r)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        left_publisher.publish(v_l)
        right_publisher.publish(v_r)
        rate.sleep()


if __name__ == "__main__":
    if len(sys.argv) == 3:
        v = float(sys.argv[1])
        w = float(sys.argv[2])
        if v>1:
            v = 1
        elif v<-1:
            v = -1
        if w>1:
            w = 1
        elif w<-1:
            w = -1
        print(v,w)
        move(v,w)
