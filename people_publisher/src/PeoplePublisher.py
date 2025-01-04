#!/usr/bin/python

import rospy
from people_msgs.msg import People, Person


class PeoplePublisher:
    def __init__(self, people):
        self.pub = rospy.Publisher('/people', People, queue_size=10)
        self.people = people

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = People()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'odom'
            msg.people = self.people
            self.pub.publish(msg)
            rate.sleep()
