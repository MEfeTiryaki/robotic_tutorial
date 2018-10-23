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
    ballPoseSubscriber = rospy.Subscriber("/aruco_football_node/pose_ball", PoseStamped, ballPoseCallback)
    #rospy.Subscriber("message", Float64, callback)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        v_l,v_r = calculateControlInput()
        print(v_l,v_r)
        left_publisher.publish(v_l)
        right_publisher.publish(v_r)
        rate.sleep()

def calculateControlInput():
    if robotPose[2]!=0 and ballPosition[1]!=0 and (not np.isnan(ballPosition[1])):
        print(robotPose,ballPosition)
        robotHeading = np.array([np.cos(robotPose[2]), np.sin(robotPose[2]) ])

        ballToRobotVector = ballPosition-robotPose[0:2]
        ballToRobotVectorUnit = ballToRobotVector/ np.linalg.norm(ballToRobotVector)
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
        w = 0
        v = 0
        print(v,w)

    #v = 0
    # OpenLoop wheel speed calculation
    v_l = int(scale*(v + d*w))
    v_r = int(scale*(v - d*w))
    #print(v_l,v_r)
    max_vel = 90
    if v_l>max_vel:
        v_l = max_vel
        print("v_l_max : " + str(v_l))
    elif v_l<-max_vel:
        v_l = -max_vel
        print("v_l_min : " + str(v_l))
    else:
        print("v_l : " + str(v_l))

    if v_r>max_vel:
        v_r = max_vel
        print("v_r_max : " + str(v_r))
    elif v_r<-max_vel:
        v_r = -max_vel
        print("v_r_min : " + str(v_r))
    else:
        print("v_r : " + str(v_r))


    return v_l,v_r


if __name__ == "__main__":
    track()
