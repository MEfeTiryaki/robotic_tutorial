#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np


def publish():
    publisher = rospy.Publisher('robotJointStates',JointState,queue_size=10)
    rospy.init_node('joint_state_publisher',anonymous=True)
    rate = rospy.Rate(1)
    stateSize = rospy.get_param("robot_state_size")

    while not rospy.is_shutdown():
        state = JointState()
        state.header.stamp = rospy.Time.now()
        state.position = np.random.rand(stateSize)
        state.velocity = [0]*stateSize
        publisher.publish(state)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
