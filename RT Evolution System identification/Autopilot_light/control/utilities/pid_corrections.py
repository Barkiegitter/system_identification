import numpy


def heading_correction(error):
    """
    This function corrects the heading error in [deg] in such a way that its bounded between -180 and 180 so the
    control action will force the shortest way.
    :param error:
    :return: bounded error
    :rtype: float
    """
    if error < -180.:
        error += 360.
    return error
