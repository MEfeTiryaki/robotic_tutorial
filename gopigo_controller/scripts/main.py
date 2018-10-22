#! /usr/bin/env python
import rospy
from gopigo_state_publisher import GopigoStatePublisher


if __name__ == "__main__":
    # ROS node
    node_name = "estimator"
    tf_pub = GopigoStatePublisher(node_name)
