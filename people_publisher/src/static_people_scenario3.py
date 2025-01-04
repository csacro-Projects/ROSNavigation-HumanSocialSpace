#!/usr/bin/python

import rospy
from PeoplePublisher import PeoplePublisher
from people_msgs.msg import Person


def create_scenario():
    personA = Person()
    personA.position.x = 2.5
    personA.position.y = 2.0
    personA.position.z = 0.5
    personA.velocity.x = 0.0
    personA.velocity.y = 0.0
    personA.name = 'static_personA'
    personA.reliability = 1.0
	
    personB = Person()
    personB.position.x = 0.5
    personB.position.y = -0.5
    personB.position.z = 0.5
    personB.velocity.x = 0.0
    personB.velocity.y = 0.0
    personB.name = 'static_personB'
    personB.reliability = 1.0
	
    people = [personA, personB]

    return people


if __name__ == '__main__':
    rospy.init_node('people_publisher')

    people = create_scenario()

    publisher = PeoplePublisher(people)
    publisher.spin()
