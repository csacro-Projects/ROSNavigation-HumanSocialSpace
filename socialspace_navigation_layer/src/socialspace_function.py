#!/usr/bin/env python

from socialspace_navigation_layer.srv import SocialspaceFunction, SocialspaceFunctionResponse
import rospy
import numpy as np
from lib_socialspace_function import social_space


def handle_request(req):
    # collect information from req
    humans_with_interactions = req.humans_with_interactions
    x = np.arange(req.min_x, req.max_x, req.resolution)
    y = np.arange(req.min_y, req.max_y, req.resolution)

    # calculate cost matrix for social space
    X, Y = np.meshgrid(x, y)
    Z = social_space(X, Y, humans_with_interactions,
                     req.personal_sigma_h_factor,
                     req.passing_right_hand_traffic, req.passing_sigma_h, req.passing_sigma_s, req.passing_sigma_r,
                     req.activity_object_sigma, req.activity_human_sigma)

    costmap_values = Z.flatten()
    return SocialspaceFunctionResponse(costmap_values)


if __name__ == "__main__":
    rospy.loginfo("starting socialspace_function_service")
    rospy.init_node("socialspace_function_service")
    s = rospy.Service("socialspace_function", SocialspaceFunction, handle_request)
    rospy.loginfo("socialspace_function_service is ready")
    rospy.spin()
