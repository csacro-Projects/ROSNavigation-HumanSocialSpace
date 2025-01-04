#!/usr/bin/python

import sys

import rospy
from HumansWithInteractionsPublisher import HumansWithInteractionsPublisher
from humans.msg import HumanWithInteractions, Human, Object, HumanInteraction


def create_scenario(with_interaction=True):
    object1 = Object(-2.0, 5.0, 0.0, 5.0)

    humanA = Human(x=2.5, y=2.0, theta=-135.0 / 180.0 * 3.14, velocity=0.0, confidence=1.0)
    humanB = Human(x=0.5, y=-0.5, theta=45.0 / 180.0 * 3.14, velocity=0.0, confidence=1.0)

    human_interactions_hA = [HumanInteraction([humanB], 1.4, 0.8, 1.0)] if with_interaction else []
    human_interactions_hB = [HumanInteraction([humanA], 1.4, 0.8, 1.0)] if with_interaction else []
    hA = HumanWithInteractions(human=humanA, human_interactions=human_interactions_hA)
    hB = HumanWithInteractions(human=humanB, human_interactions=human_interactions_hB)

    humans_with_interactions = [hA, hB]
    return humans_with_interactions


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    with_interaction = myargv[1].lower() == 'true'

    rospy.init_node('humansWithInteractions_publisher')

    humans_with_interactions = create_scenario(with_interaction=with_interaction)

    publisher = HumansWithInteractionsPublisher(humans_with_interactions)
    publisher.spin()
