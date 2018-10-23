#! /usr/bin/env python
import rospy
from gopigo_state_publisher import GopigoStatePublisher


if __name__ == "__main__":
    # ROS node
    node_name = "gopigo_state_publisher"
    tf_pub = GopigoStatePublisher(node_name)
