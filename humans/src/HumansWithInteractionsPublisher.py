#!/usr/bin/python

import rospy
from humans.msg import HumansWithInteractions, HumanWithInteractions, Human, HumanInteraction, Object, ObjectInteraction


class HumansWithInteractionsPublisher:
    def __init__(self, humans_with_interactions):
        self.pub = rospy.Publisher('/humans', HumansWithInteractions, queue_size=10)
        self.humans_with_interactions = humans_with_interactions

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = HumansWithInteractions()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'odom'
            msg.humans_with_interactions = self.humans_with_interactions
            self.pub.publish(msg)
            rate.sleep()
