import numpy as np
from skimage.measure import EllipseModel

"""
Basic Functions
"""


def asymmetric_gaussian(x, y, x_center, y_center, theta, sigma_h, sigma_s, sigma_r):
    # implemented according to Algorithm A.1 in
    # Kirby, R. Social Robot Navigation. PhD Thesis. Carnegie Mellon University (May 2010)
    # https://www.ri.cmu.edu/publications/social-robot-navigation/
    alpha = np.arctan2(y - y_center, x - x_center) - theta + np.pi / 2
    # normalize alpha
    alpha[alpha > np.pi] -= 2 * np.pi
    alpha[alpha <= -np.pi] += 2 * np.pi

    sigma = np.where(alpha <= 0, sigma_r, sigma_h)
    a = np.cos(theta) ** 2 / (2 * sigma ** 2) + np.sin(theta) ** 2 / (2 * sigma_s ** 2)
    b = np.sin(2 * theta) / (4 * sigma ** 2) - np.sin(2 * theta) / (4 * sigma_s ** 2)
    c = np.sin(theta) ** 2 / (2 * sigma ** 2) + np.cos(theta) ** 2 / (2 * sigma_s ** 2)
    return np.exp(-(a * (x - x_center) ** 2 + 2 * b * (x - x_center) * (y - y_center) + c * (y - y_center) ** 2))


def triangle(x, y, x_human, y_human, x_start, y_start, x_end, y_end, sigma):
    # idea from
    # Bai, L. Merging human-object interaction behavior into a personal space model: for social robot navigation. Student Master thesis (Dec 2018)
    # https://research.tue.nl/en/studentTheses/merging-human-object-interaction-behavior-into-a-personal-space-m
    # however we choose distance not in terms of a radius but as points between two planes

    # check whether we are within the distance
    if x_start - x_end == 0:
        m = (y_start - y_end) / np.finfo(float).eps
    else:
        m = (y_start - y_end) / (x_start - x_end)
    c_object = y_start - m * x_start
    c_human = y_human - m * x_human
    dist_object_human = np.fabs((m * x_human - y_human + c_object)) / (np.sqrt(m ** 2 + 1))
    dist_to_object = np.fabs((m * x - y + c_object)) / (np.sqrt(m ** 2 + 1))
    dist_to_human = np.fabs((m * x - y + c_human)) / (np.sqrt(m ** 2 + 1))
    within_dist = (dist_to_object <= dist_object_human) & (dist_to_human <= dist_object_human)

    # check where we lie in terms of angle
    human_object_projection_x = (x_human + m * (y_human - c_object)) / (1 + m ** 2)
    human_object_projection_y = human_object_projection_x * m + c_object
    theta_human_object = np.arctan2(human_object_projection_y - y_human, human_object_projection_x - x_human)
    transform_angle_space = (np.abs(theta_human_object) * 180 / np.pi > 45)

    theta_start = np.arctan2(y_start - y_human, x_start - x_human)
    theta_end = np.arctan2(y_end - y_human, x_end - x_human)
    if transform_angle_space:
        # transform from [-pi, pi] to [0, 2pi] in order to avoid discontinuity in angle
        if theta_start < 0:
            theta_start += 2 * np.pi
        if theta_end < 0:
            theta_end += 2 * np.pi
    theta_l = np.maximum(theta_start, theta_end)
    theta_s = np.minimum(theta_start, theta_end)
    theta_xy = np.arctan2(y - y_human, x - x_human)
    if transform_angle_space:
        # transform from [-pi, pi] to [0, 2pi] in order to avoid discontinuity in angle
        theta_xy[theta_xy < 0] += 2 * np.pi

    within_triangle = (theta_xy <= theta_l) & (theta_xy >= theta_s)
    larger_than_triangle = (theta_xy > theta_l)
    smaller_than_triangle = (theta_xy < theta_s)

    # calculate result
    result = (within_triangle & within_dist).astype(float)
    result[larger_than_triangle & within_dist] = np.exp(
        -(theta_xy[larger_than_triangle & within_dist] - theta_l) ** 2 / (2 * sigma ** 2))
    result[smaller_than_triangle & within_dist] = np.exp(
        -(theta_xy[smaller_than_triangle & within_dist] - theta_s) ** 2 / (2 * sigma ** 2))
    return result


def ellipse(x, y, persons_x_y, center_x_y, sigma):
    # ensure center
    persons_x_y_reflected = center_x_y + (center_x_y - persons_x_y)
    points_x_y = np.concatenate((persons_x_y, persons_x_y_reflected))

    # fit the ellipse
    ellipse_model = EllipseModel()
    success = ellipse_model.estimate(points_x_y)
    if success:
        xc, yc, a, b, theta = ellipse_model.params
    else:
        # fit a circle (as a special case of an ellipse)
        diff_to_center = points_x_y - center_x_y
        radii = np.sqrt(diff_to_center[0] ** 2 + diff_to_center[1] ** 2)
        r = np.mean(radii)
        xc, yc, a, b, theta = center_x_y[0], center_x_y[1], r, r, 0.0

    # check if we lie within the ellipse (coordinate system transformation to ellipse coordinate frame)
    x_t = (x - xc) * np.cos(theta) + (y - yc) * np.sin(theta)
    y_t = -(x - xc) * np.sin(theta) + (y - yc) * np.cos(theta)
    within_ellipse = (x_t ** 2 / a ** 2 + y_t ** 2 / b ** 2 <= 1)

    # get distance to ellipse (coordinate system transformation from cartesian to polar coordinate frame)
    theta_xy_t = np.arctan2(y_t, x_t)
    theta_xy_t[theta_xy_t < 0] += 2 * np.pi  # transform from [-pi, pi] to [0, 2pi]
    ellipse_r = a * b / (np.sqrt(a ** 2 * np.sin(theta_xy_t) ** 2 + b ** 2 * np.cos(theta_xy_t) ** 2))
    xy_r = np.sqrt(x_t ** 2 + y_t ** 2)

    # calculate result
    result = within_ellipse.astype(float)
    result[~within_ellipse] = np.exp(
        -(ellipse_r[~within_ellipse] - xy_r[~within_ellipse]) ** 2 / (2 * sigma ** 2))
    return result


"""
Parameterized Functions for Modelling the Social Space
"""


def personal_space(x, y, human, sigma_h_factor=1.0):
    # default values for model from
    # Kirby, R. Social Robot Navigation. PhD Thesis. Carnegie Mellon University (May 2010)
    # https://www.ri.cmu.edu/publications/social-robot-navigation/
    sigma_h = sigma_h_factor * np.maximum(2.0 * human.velocity, 0.5)
    sigma_s = 2.0 / 3.0 * sigma_h
    sigma_r = 1.0 / 2.0 * sigma_h
    return human.confidence * asymmetric_gaussian(x, y, human.x, human.y, human.theta, sigma_h, sigma_s, sigma_r)


def passing_space(x, y, human, right_hand_traffic=True, sigma_h=2.0, sigma_s=0.25, sigma_r=0.01):
    # default values for model from
    # Kirby, R. Social Robot Navigation. PhD Thesis. Carnegie Mellon University (May 2010)
    # https://www.ri.cmu.edu/publications/social-robot-navigation/
    if human.velocity <= 0:
        return np.zeros_like(x, dtype=float)
    theta = human.theta
    if right_hand_traffic:
        theta -= np.pi / 2.0
    else:
        theta += np.pi / 2.0
    return human.confidence * asymmetric_gaussian(x, y, human.x, human.y, theta, sigma_h, sigma_s, sigma_r)


def activity_space_object(x, y, human, object_interaction, sigma=np.pi / 36.0 * 5.0 / 3.0):
    # default value for model is adapted from
    # Bai, L. Merging human-object interaction behavior into a personal space model: for social robot navigation. Student Master thesis (Dec 2018)
    # https://research.tue.nl/en/studentTheses/merging-human-object-interaction-behavior-into-a-humanal-space-m
    confidence = object_interaction.confidence
    obj = object_interaction.item
    return confidence * triangle(x, y, human.x, human.y, obj.x_start, obj.y_start, obj.x_end, obj.y_end, sigma)


def activity_space_human(x, y, human, human_interaction, sigma=0.5):
    # default value for model is related to personal space
    confidence = human_interaction.confidence
    humans_in_group = human_interaction.item
    humans_in_group.append(human)
    return confidence * ellipse(x, y, np.array([(p.x, p.y) for p in humans_in_group]),
                                np.array([human_interaction.center_x, human_interaction.center_y]), sigma)


"""
Social Space Function
"""

# aggregate functions are order-invariant for multiple calls and invariant to "batching" (a,b vs a,b,c)
aggregate_max = lambda a, b: np.maximum(a, b)
aggregate_product = lambda a, b: 1 - ((1 - a) * (1 - b))


def social_space(x, y, humans_with_interactions,
                 personal_sigma_h_factor=1.0,
                 passing_right_hand_traffic=True, passing_sigma_h=2.0, passing_sigma_s=0.25, passing_sigma_r=0.01,
                 activity_object_sigma=np.pi / 36.0 * 5.0 / 3.0, activity_human_sigma=0.5):
    social_space = np.zeros_like(x, dtype=float)
    for human_with_interactions in humans_with_interactions:
        # for each human we aggregate the individual spaces with product aggregation
        social_space_human = personal_space(x, y, human_with_interactions.human, personal_sigma_h_factor)
        social_space_human = aggregate_product(social_space_human,
                                               passing_space(x, y, human_with_interactions.human,
                                                             passing_right_hand_traffic, passing_sigma_h,
                                                             passing_sigma_s, passing_sigma_r))
        for object_interaction in human_with_interactions.object_interactions:
            social_space_human = aggregate_product(social_space_human,
                                                   activity_space_object(x, y, human_with_interactions.human,
                                                                         object_interaction, activity_object_sigma))
        for human_interaction in human_with_interactions.human_interactions:
            social_space_human = aggregate_product(social_space_human,
                                                   activity_space_human(x, y, human_with_interactions.human,
                                                                        human_interaction, activity_human_sigma))
        # the individual social spaces of different humans are combined with max aggregation
        social_space = aggregate_max(social_space, social_space_human)
    return social_space
