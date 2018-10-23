#!/usr/bin/env python


import sys
import rospy

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
k_rot = 0.1
k_lin = 0.1

def robotPoseCallback(msg):
    robotPose[0] = msg.pose.position.x
    robotPose[1] = msg.pose.position.y
    quaternion = [msg.pose.orientation.x
                 ,msg.pose.orientation.y
                 ,msg.pose.orientation.z
                 ,msg.pose.orientation.w]
    robotPose[2] = tr.euler_from_quaternion(quaternion)[2]

def ballPoseCallback(msg):
    ballPosition[0] = msg.pose.position.x
    ballPosition[1] = msg.pose.position.y
    #print(ballPosition)
def track():
    ns = rospy.get_namespace()
    rospy.init_node('ball_tracker')
    left_publisher = rospy.Publisher(ns+'motor/pwm/left',Int8,queue_size=10)
    right_publisher = rospy.Publisher(ns+'motor/pwm/right',Int8,queue_size=10)

    robotPoseSubscriber = rospy.Subscriber(ns+"pose", PoseStamped, robotPoseCallback)
    ballPoseSubscriber = rospy.Subscriber("/aruco_football_node/pose_"+ns[-2], PoseStamped, ballPoseCallback)
    #rospy.Subscriber("message", Float64, callback)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        v_l,v_r = calculateControlInput()
        left_publisher.publish(v_l)
        right_publisher.publish(v_r)
        rate.sleep()

def calculateControlInput():
    if robotPose[2]!=0 and ballPosition[1]!=0:
        print(robotPose,ballPosition)
        robotHeading = np.array([np.cos(robotPose[2]), np.sin(robotPose[2]) ])
        ballToRobotVector = ballPosition-robotPose[0:2]
        ballToRobotVectorUnit = ballPosition/ np.linalg.norm(ballPosition)
        distanceProjectedToHeading = ballToRobotVector.dot(robotHeading)
        print("ball to robot",ballToRobotVectorUnit)
        print("distance",distanceProjectedToHeading)
        robotToballDirection = np.arctan2(ballToRobotVectorUnit[1],ballToRobotVectorUnit[0])

        deltaDirection = robotToballDirection-robotPose[2]
        if deltaDirection<-np.pi:
            deltaDirection = deltaDirection + 2 * np.pi
        elif deltaDirection>np.pi:
            deltaDirection = deltaDirection - 2 * np.pi
        print("direction",robotToballDirection,robotPose[2],deltaDirection)
        v = k_lin * (distanceProjectedToHeading)
        w = k_rot * (robotToballDirection-robotPose[2])
        print(v,w)
    else:
        v = 0
        w = 0
    # OpenLoop wheel speed calculation
    v_l = int(scale*(v + d*w))
    v_r = int(scale*(v - d*w))
    #print(v_l,v_r)
    if v_l>127:
        v_l = 127
        print("v_l_max : " + str(v_l))
    elif v_l<-127:
        v_l = -127
        print("v_l_min : " + str(v_l))
    else:
        print("v_l : " + str(v_l))

    if v_r>127:
        v_r = 127
        print("v_r_max : " + str(v_r))
    elif v_r<-127:
        v_r = -127
        print("v_r_min : " + str(v_r))
    else:
        print("v_r : " + str(v_r))

    return v_l,v_r


if __name__ == "__main__":
    track()
