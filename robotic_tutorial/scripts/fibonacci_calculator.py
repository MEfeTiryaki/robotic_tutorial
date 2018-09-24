#!/usr/bin/env python
import rospy
from robotic_tutorial_msgs.srv import Fibonacci,FibonacciResponse

def callback(req):
    print(req)
    if req.number<0:
        rospy.loginfo("Wrong number!!")
        return FibonacciResponse(False,-1)
    if req.number > 20:
        rospy.loginfo("Too big number!!")
        return FibonacciResponse(False,0)
    res = 1
    for i in range(1,req.number+1):
        res *=i
    return FibonacciResponse(True, res)

def server():
    rospy.init_node('fibonacci_calculator')
    s = rospy.Service('fibonacci_service', Fibonacci, callback)
    print("Server is running!!!")
    rospy.spin()

if __name__ == "__main__":
    server()
