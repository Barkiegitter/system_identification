# import sympy as sp
import math


# r, t, h, k, a1, a2, b1, b2, c1, c2, theta = sp.symbols('r t h k a1 a2 b1 b2 c1 c2 theta')
# # x1, y1, x2, y2 = sp.symbols('x1 y1 x2 y2')
#
# # x1, x2, y1, y2 = sp.symbols('x1, x2, y1, y2', cls=sp.Function)
# x1 = a1 * t ** 2 + b1 * t + c1
# y1 = a2 * t ** 2 + b2 * t + c2
# x2 = h + r * sp.cos(theta)
# y2 = k + r * sp.sin(theta)
#
# eq1 = sp.Eq(x1, x2)
# eq2 = sp.Eq(y1, y2)
#
# sp.simplify()


def camera_angles(h, k, a1, b1, c1, a2, b2, c2, theta, t_ego=None):
    """
    This function is a solution for the solving of the following parametric equations
    x = a1t^2+b1*t+c1
    y = a2t^2+b2*t+c2
    with the system
    x = h+r*sin(theta)
    y = k+r*cos(theta)
    It solves for t, as it is a polynomial of the second order the ABC rule is used.

    :param h: Geo x of ego vessel in Eucledian space
    :type h: float
    :param k: Geo x of ego vessel in Eucledian space
    :type k: float
    :param a1: first poly constant of target vessel for x
    :type a1: float
    :param b1: second poly constant of target vessel for x
    :type b1: float
    :param c1: third poly constant of target vessel for x
    :type c1: float
    :param a2: first poly constant of target vessel for y
    :type a2: float
    :param b2: second poly constant of target vessel for y
    :type b2: float
    :param c2: third poly constant of target vessel for y
    :type c3: float
    :param theta: detection angle in [rad]
    :type theta: float
    :param t_ego: (Optional) ego vehicle time, when filled returns the time with the smallest delta and the delta itself
    :type t_ego: floatls
    :return: t1, t2 calculated time points on the targets ship trajectory
    :rtype: tuple
    """
    t1 = (1 / 2) * (-b1 * math.sin(theta) + b2 * math.cos(theta) - math.sqrt(
        -4 * a1 * c1 * math.sin(theta) ** 2 + 2 * a1 * c2 * math.sin(2 * theta) + 4 * a1 * h * math.sin(
            theta) ** 2 - 2 * a1 * k * math.sin(2 * theta) + 2 * a2 * c1 * math.sin(2 * theta) - 4 * a2 * c2 * math.cos(
            theta) ** 2 - 2 * a2 * h * math.sin(2 * theta) + 4 * a2 * k * math.cos(theta) ** 2 + b1 ** 2 * math.sin(
            theta) ** 2 - b1 * b2 * math.sin(2 * theta) + b2 ** 2 * math.cos(theta) ** 2)) / (
                 a1 * math.sin(theta) - a2 * math.cos(theta))

    t2 = (1 / 2) * (-b1 * math.sin(theta) + b2 * math.cos(theta) + math.sqrt(
        -4 * a1 * c1 * math.sin(theta) ** 2 + 2 * a1 * c2 * math.sin(2 * theta) + 4 * a1 * h * math.sin(
            theta) ** 2 - 2 * a1 * k * math.sin(2 * theta) + 2 * a2 * c1 * math.sin(2 * theta) - 4 * a2 * c2 * math.cos(
            theta) ** 2 - 2 * a2 * h * math.sin(2 * theta) + 4 * a2 * k * math.cos(theta) ** 2 + b1 ** 2 * math.sin(
            theta) ** 2 - b1 * b2 * math.sin(2 * theta) + b2 ** 2 * math.cos(theta) ** 2)) / (
                     a1 * math.sin(theta) - a2 * math.cos(theta))
    if t_ego is None:
        return t1, t2
    else:
        d1 = math.fabs(t1 - t_ego)
        d2 = math.fabs(t2 - t_ego)
        if d1 < d2:
            return t1, d1
        else:
            return t2, d2


if __name__ == "__main__":
    print('Verify camera angles function')
    a1 = 0.
    b1 = 8.
    c1 = 2.
    a2 = 1.2
    b2 = 0.
    c2 = 5.
    h = 1.
    k = 1.
    theta = 60.0 / 180. * math.pi
    t1, t2 = camera_angles(h, k, a1, b1, c1, a2, b2, c2, theta)
    print(t1, t2)

    # The function has been verified using a Ti-84 plus calculator (Oldschool, I know)
