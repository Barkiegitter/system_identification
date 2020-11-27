import time
class PID:
    def __init__(self, P=0.0, I=0.0, D=1.0, F=0.0, Derivator=0, order=2, Integrator=0,
                 Predictor=0, p_hoz=3, Ibounds_speed=(-23.3, 23.3)):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Kf = F
        self.Predictor = Predictor
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_min, self.Integrator_max = Ibounds_speed

        # Internal Parameters
        self.D_value = None
        self.P_value = None
        self.I_value = None
        self.F_value = None
        self.set_point = 0.0
        self.error = 0.0
        self.p_hoz = p_hoz
        self.override = False
        self.t_old = time.time()

    def update(self, current_value, derivative=None):
        """
        Calculate PID output value for given reference input and feedback

        :param current_value: Current measured state.
        :type current_value: float
        :param derivative: if not None this value will be used as a direct derivative measurement.
        :type derivative: float
        :return: PID control output
        :rtype: float
        """
        # print(derivative)
        t_new = time.time()
        delta_t = t_new - self.t_old
        self.error = self.set_point - current_value

        # Anti windup

        self.P_value = self.Kp * self.error
        
        self.Integrator += (self.error * delta_t)
        self.I_value = self.Integrator * self.Ki
        # Check if external derivative value is supplied

        # print(self.Kd, self.error, self.Derivator)
        self.D_value = self.Kd * (self.error - self.Derivator)/delta_t
        self.Derivator = self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        return self.P_value + self.I_value + self.D_value 

    def setPoint(self, set_point):
        """
        Update the set-Point of the controller. Update is ignored if the same value is already the setpoint.
        On update the integrator is set to 0.

        :param set_point: The desired control setpoint
        :type set_point: float
        """
        if (self.set_point != set_point) and not self.override:
            self.set_point = set_point
            self.Integrator = 0
            if self.set_point < 0:
                self.set_point = (360 + self.set_point)
        else:
            pass
        
    def setPoint_speed(self, set_point):
        """
        Update the set-Point of the controller. Update is ignored if the same value is already the setpoint.
        On update the integrator is set to 0.

        :param set_point: The desired control setpoint
        :type set_point: float
        """
        if (self.set_point != set_point):
            self.set_point = set_point
            # self.Integrator = 0
            pass
    def setPoint_hdg(self, set_point):
        """
        Update the set-Point of the controller. Update is ignored if the same value is already the setpoint.
        On update the integrator is set to 0.

        :param set_point: The desired control setpoint
        :type set_point: float
        """
        if (self.set_point != set_point):
            self.set_point = set_point
            # self.Integrator = 0
            pass

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setPredictor(self, Predictor):
        self.Predictor = self.Predictor

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

    def setKf(self, F):
        self.Kf = F

    def switchOverride(self):
        self.override = not self.override

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

    def getPredictor(self):
        return self.Predictor