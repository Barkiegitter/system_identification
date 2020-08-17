import numpy as np
import scipy as sp
import time


class FutureSpline:
    def __init__(self, time, rate, type='sine'):
        """
        Spline Function (Work in Progress) maybe a future way to do a feed forward controller based on a shaping
        function.

        :param time: The expected timescale
        :param rate:
        :param type:
        """
        self.t_v = np.zeros(time*rate)
        self.heading = np.zeros(self.t_v.shape)
        self.commands = np.zeros(self.t_v.shape)
        self.derivative = np.zeros(self.t_v.shape)

    def update(self, command, heading, derivative):
        self.t_v = np.roll(self.t_v, -1)
        self.heading = np.roll(self.heading, -1)
        self.commands = np.roll(self.commands, -1)
        self.derivative = np.roll(self.derivative, -1)
        self.t_v[-1] = time.time()
        self.heading[-1] = heading
        self.commands[-1] = command
        self.derivative[-1] = derivative


