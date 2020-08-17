import numpy as np
import time, logging


class MovingAverage:
    def __init__(self, order=2):
        if order < 2:
            order = 2
            logging.warning("Selected filter order lower than 2, defaulting to -> 2")
        self.order = order
        # initiate as ones to make sure that we won't get divide by zero
        self.t = np.ones(order)
        self.x = np.ones(order)

    def filter(self, x_new, t_new):
        # move old variables to the front
        self.t = np.roll(self.t, 1)
        self.x = np.roll(self.x, 1)

        # update old variables
        self.t[0] = t_new
        self.x[0] = x_new/t_new
        rval = np.mean(self.x)
        return rval


if __name__ == "__main__":
    ma = MovingAverage(3)

    x = np.ones(10)

    t_old = time.time()
    for k in x:
        delta_t =time.time()-t_old
        print(ma.filter(k,delta_t))
        t_old = time.time()
        time.sleep(0.2)
