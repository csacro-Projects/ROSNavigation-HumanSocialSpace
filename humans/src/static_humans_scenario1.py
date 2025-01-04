#!/usr/bin/python

import sys

import rospy
from HumansWithInteractionsPublisher import HumansWithInteractionsPublisher
from humans.msg import HumanWithInteractions, Human, Object, ObjectInteraction


def create_scenario(with_interaction=True):
    object1 = Object(-3.5, 2.0, -3.0, 3.0)

    humanA = Human(x=2.0, y=1.5, theta=170.0 / 180.0 * 3.14, velocity=0.0, confidence=1.0)

    object_interactions_hA = [ObjectInteraction(object1, 1.0)] if with_interaction else []
    hA = HumanWithInteractions(human=humanA, object_interactions=object_interactions_hA)

    humans_with_interactions = [hA]
    return humans_with_interactions


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    with_interaction = myargv[1].lower() == 'true'

    rospy.init_node('humansWithInteractions_publisher')

    humans_with_interactions = create_scenario(with_interaction=with_interaction)

    publisher = HumansWithInteractionsPublisher(humans_with_interactions)
    publisher.spin()
