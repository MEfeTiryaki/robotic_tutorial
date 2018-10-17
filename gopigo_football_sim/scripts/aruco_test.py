
import rospy
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

# Instantiate CvBridge
bridge = CvBridge()


def showImage(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('image',cv2_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite('camera_image.jpeg', cv2_img)


def showPose(msg):
    #print(msg)
    pass
def main():
    node_name = "aruco_test"
    rospy.init_node(node_name,log_level=rospy.DEBUG)

    """
    sub = rospy.Subscriber( "/aruco_single/pose"     \
                         , PoseStamped                       \
                         , showPose)
    #"""
    """
    sub_img = rospy.Subscriber( "/aruco_multi/debug"     \
                         , Image                       \
                         , showImage)
    #"""
    #"""
    sub_img = rospy.Subscriber( "/aruco_multi/result"   \
                         , Image                       \
                         , showImage)
    #"""
    rospy.spin()

if __name__ == '__main__':
    main()
