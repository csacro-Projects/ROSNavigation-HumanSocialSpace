#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler


class NavGoalPublisher:
    def __init__(self, x, y, yaw):
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.x = x
        self.y = y
        self.yaw = yaw

    def publish(self):
        rate = rospy.Rate(10)
        while self.pub.get_num_connections() < 1:
            rate.sleep()
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.position = Point(self.x, self.y, 0)
        tmp = quaternion_from_euler(0, 0, self.yaw)
        msg.pose.orientation = Quaternion(tmp[0], tmp[1], tmp[2], tmp[3])
        self.pub.publish(msg)
