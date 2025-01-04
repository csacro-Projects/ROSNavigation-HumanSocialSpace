#!/usr/bin/python

import rospy
from PeoplePublisher import PeoplePublisher
from people_msgs.msg import Person


def create_scenario():
    person = Person()
    person.position.x = 2.0
    person.position.y = 1.5
    person.position.z = 0.5
    person.velocity.x = 0.0
    person.velocity.y = 0.0
    person.name = 'static_person'
    person.reliability = 1.0
    people = [person]

    return people


if __name__ == '__main__':
    rospy.init_node('people_publisher')

    people = create_scenario()

    publisher = PeoplePublisher(people)
    publisher.spin()
