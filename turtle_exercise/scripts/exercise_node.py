#!/usr/bin/env python

from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtle_exercise.msg import Exercise_start
from geometry_msgs.msg import Twist
import rospy
import numpy as np

class MyTurtle:
    def __init__(self):
        self.isReceived = False
        self.isPlanned = False
        self.start = [0.0,0.0,0,0]
        self.end = [0.0,0.0,0,0]

        self.ns_ = rospy.get_namespace()
        rospy.init_node('~',anonymous=True)

        self.readParameters()
        self.initilizeSubscribers()
        self.initilizePublishers()

        self.rate = rospy.Rate(50)

        self.loop()

    def readParameters(self):
        # TODO : Read rosparameters "/max_vel/lin" and "/max_vel/ang" and assign
        # them to self.max_lin_vel and self.max_ang_vel respectively. Then
        # print the values using rospy.loginfo
        self.max_lin_vel = rospy.get_param(self.ns_+"/max_vel/lin")
        self.max_ang_vel = rospy.get_param(self.ns_+"/max_vel/ang")
        rospy.loginfo("Max linear velocity : " + str(self.max_lin_vel))
        rospy.loginfo("Max angular velocity :  " + str(self.max_ang_vel))

    def initilizeSubscribers(self):
        # TODO : create a subscriber called self.startSubscriber_ which subscribers
        # 'starter' with type 'Exercise_start' and evoke self.start_Callback
        self.startSubscriber_ = rospy.Subscriber("starter", Exercise_start \
                , callback = self.start_Callback )

        # TODO : create a subscriber called self.stateSubscriber_ which subscribers
        # '/turtle1/pose' with type 'Pose' and evoke self.state_Callback
        self.stateSubscriber_ = rospy.Subscriber("/turtle1/pose", Pose \
                , callback = self.state_Callback )

    def initilizePublishers(self):
        # TODO : create a publisher called self.velPublisher which publishes
        # '/turtle1/cmd_vel' topic with type 'Twist' and queue_size 10
        self.velPublisher =  rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)

    def start_Callback(self,msg):
        # TODO : Write the start and end state arrays into self.start and
        # self.end variables and print these variables with rospy.loginfo()
        self.start = msg.start
        self.end = msg.end
        rospy.loginfo("Start : " + str(self.start))
        rospy.loginfo("End : " + str(self.end))
        # XXX : Don't touch this part
        if not self.isReceived:
            self.initPosition()
            self.plan()

    def state_Callback(self,msg):
        # TODO : create 3 dim numpy.array "self.state" receiving x,y,theta from
        # msg
        self.state = np.array([msg.x,msg.y,msg.theta])

    def publish_vel_vmd(self,vel):
        # TODO : create instance of Twist with name msg and assign velocities.
        msg = Twist()
        msg.linear.x = vel[0]
        msg.angular.z = vel[1]
        # TODO : publish msg using self.velPublisher
        self.velPublisher.publish(msg)

    def initPosition(self):
        # TODO : Move turtle to start point using "/turtle1/teleport_absolute"
        # service
        rospy.wait_for_service('/turtle1/teleport_absolute')
        try:
            # TODO : create service_proxy for '('/turtle1/teleport_absolute' using
            # 'TeleportAbsolute' service. Then call service proxy with self.start
            # as request
            service_proxy = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
            resp1 = service_proxy(self.start[0],self.start[1],self.start[2])
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # TODO : Clean the path using "/clear" services
        rospy.wait_for_service('/clear')
        try:
            # TODO : create service_proxy for '/clear' using Empty service. Then
            # run service proxy with empty request.
            service_proxy = rospy.ServiceProxy('/clear', Empty)
            resp1 = service_proxy()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def plan(self):
        self.vel_cmd = []
        self.exe_time = []
        direction_vector = np.array(self.end[0:2])-np.array(self.start[0:2])
        if (np.linalg.norm(direction_vector) != 0):
            # TODO : Rotate turtle to the direction of the end point
            unit_dir = direction_vector/np.linalg.norm(direction_vector)
            ang = np.arctan2(unit_dir[1],unit_dir[0])
            first_turn = ang-self.start[2]
            first_turn_duration =  np.abs(first_turn)/self.max_ang_vel
            self.vel_cmd.append([0.0,self.max_ang_vel if first_turn>0 else -self.max_ang_vel])
            self.exe_time.append(first_turn_duration)

            # TODO : Move turtle to the direction of the end point
            move_forward_duration = np.linalg.norm(direction_vector)/self.max_lin_vel
            self.vel_cmd.append([self.max_lin_vel,0.0])
            self.exe_time.append(self.exe_time[-1]+move_forward_duration)

            # TODO : Rotate turtle to the final direction
            last_turn = self.end[2]-ang
            last_turn_duration = np.abs(last_turn)/self.max_ang_vel
            self.vel_cmd.append([0.0,self.max_ang_vel if last_turn>0 else -self.max_ang_vel])
            self.exe_time.append(self.exe_time[-1]+last_turn_duration)
        else:
            only_turn = self.end[2]-self.start[2]
            only_turn_duration = np.abs(only_turn)/self.max_ang_vel
            self.vel_cmd.append([0.0,self.max_ang_vel if only_turn>0 else -self.max_ang_vel])
            self.exe_time.append(only_turn_duration)

        # Don't Touch this part of the code
        self.start_time = rospy.Time.now()
        self.isPlanned = True

    def track(self):
        # XXX : Don't touch this part of the code
        execution_time = rospy.Time.now()
        self.time_from_start = execution_time - self.start_time
        #print(self.time_from_start.to_sec())
        for i in range(0,len(self.exe_time)):
            if self.exe_time[i]> self.time_from_start.to_sec():
                self.publish_vel_vmd(self.vel_cmd[i])
                return
        # if self.time_from_start greater than all this part runs
        self.isPlanned = False
        self.isReceived = False

    def loop(self):
        # XXX : Dont touch this part of the code
        while not rospy.is_shutdown():
            if self.isPlanned:
                self.track()
            else:
                self.publish_vel_vmd([0.0,0.0])
            self.rate.sleep()


if __name__ == "__main__":
    t = MyTurtle()
