#!/usr/bin/python

import sys

import rospy
from NavGoalPublisher import NavGoalPublisher

if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    x = float(myargv[1])
    y = float(myargv[2])
    yaw = float(myargv[3])

    rospy.init_node('navgoal_publisher')

    publisher = NavGoalPublisher(x, y, yaw)
    publisher.publish()
