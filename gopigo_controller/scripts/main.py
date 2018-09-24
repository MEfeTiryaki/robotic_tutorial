#! /usr/bin/env python
import rospy
from controller_node import ControllerNode


if __name__ == "__main__":
    # ROS node
    node_name = "controller"
    tf_pub = ControllerNode(node_name)
