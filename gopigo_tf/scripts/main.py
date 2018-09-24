#! /usr/bin/env python
import rospy
from ros_tf_publisher_py.RosTfPublisher import RosTfPublisher


if __name__ == "__main__":
    # ROS node
    node_name = "tf_publisher"
    tf_pub = RosTfPublisher(node_name)
