#!/usr/bin/env python


import sys
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
import tf
import tf.transformations as tr

# VARIABLES
robotPose = np.array([0.0,0.0,0.0])
ballPosition = np.array([0.0,0.0])
robotState = False
ballState = False
scale = 470
d = 0.06
k_rot = 2
k_lin = 0.5

def robotPoseCallback(msg):
    robotPose[0] = -msg.pose.position.x
    robotPose[1] = msg.pose.position.y
    quaternion = [msg.pose.orientation.x
                 ,msg.pose.orientation.y
                 ,msg.pose.orientation.z
                 ,msg.pose.orientation.w]
    robotPose[2] = -tr.euler_from_quaternion(quaternion)[2]
    print("robot",robotPose)

def ballPoseCallback(msg):
    ballPosition[0] = -msg.pose.position.x
    ballPosition[1] = msg.pose.position.y
    print("ball",ballPosition)


def track():
    ns = rospy.get_namespace()
    rospy.init_node('ball_tracker')
    left_publisher = rospy.Publisher(ns+'motor/pwm/left',Int8,queue_size=10)
    right_publisher = rospy.Publisher(ns+'motor/pwm/right',Int8,queue_size=10)

    robotPoseSubscriber = rospy.Subscriber(ns+"pose", PoseStamped, robotPoseCallback)
    ballPoseSubscriber = rospy.Subscriber("/aruco_football_node/pose_ball", PoseStamped, ballPoseCallback)
    #rospy.Subscriber("message", Float64, callback)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        v_l,v_r = calculateControlInput()
        left_publisher.publish(v_l)
        right_publisher.publish(v_r)
        rate.sleep()

def calculateControlInput():
    # XXX : WRITE YOUR CONTROL LAW HERE
    if robotPose[0]!=0 and ballPosition[0]!=0:
        robotToBall= ballPosition - robotPose[0:2]
        print("robotToBall : ", robotToBall)
        robotToBallUnit= robotToBall/np.linalg.norm(robotToBall)
        print("robotToBallUnit : ", robotToBallUnit)
        angle = np.arctan2(robotToBallUnit[1],robotToBallUnit[0])
        print("angle : ", angle)

        # ANGLE ERROR
        del_angle = angle-robotPose[2]

        w = k_rot * del_angle
    else:
        v = 0
        w = 0

    print("------------------------")
    v = 0
    # OpenLoop wheel speed calculation
    v_l = int(scale*(v + d*w))
    v_r = int(scale*(v - d*w))
    #print(v_l,v_r)
    max_vel = 90
    if v_l>max_vel:
        v_l = max_vel
        #print("v_l_max : " + str(v_l))
    elif v_l<-max_vel:
        v_l = -max_vel
        #print("v_l_min : " + str(v_l))
    else:
        pass
        #print("v_l : " + str(v_l))

    if v_r>max_vel:
        v_r = max_vel
        #print("v_r_max : " + str(v_r))
    elif v_r<-max_vel:
        v_r = -max_vel
        #print("v_r_min : " + str(v_r))
    else:
        pass
        #print("v_r : " + str(v_r))


    return v_l,v_r


if __name__ == "__main__":
    track()
