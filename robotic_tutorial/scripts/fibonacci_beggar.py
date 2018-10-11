#!/usr/bin/env python

import sys
import rospy
from robotic_tutorial.srv import Fibonacci,FibonacciResponse

def client(x):
    rospy.wait_for_service('fibonacci_service')
    try:
        service_proxy = rospy.ServiceProxy('fibonacci_service', Fibonacci)
        resp1 = service_proxy(x)
        return resp1.succeed,resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = int(sys.argv[1])
    else:
        x = 5
    succ,res = client(x)
    if succ :
        print("Fibonacci number for "+ str(x) + " is "+  str(res))
    else:
        print("Failled to calculate!!")
