# The recipe gives simple implementation of a Discrete Proportional-Integral-Derivative (PID) controller. PID controller gives output value for error between desired reference input and measurement feedback to minimize error value.
# More information: http://en.wikipedia.org/wiki/PID_controller
#
# cnr437@gmail.com
#
#######	Example	#########
#
# p=PID(3.0,0.4,1.2)
# p.setPoint(5.0)
# while True:
#     pid = p.update(measurement_value)
import numpy as np
import time


class PID:

    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=180, Integrator_min=-180):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_1 = 0.0
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min

        self.set_point = 0.0
        self.set_point_1 = 0.0
        self.error = 0.0
        self.t_1 = time.time()


    def update(self, current_value):
        """
		Calculate PID output value for given reference input and feedback
		"""

        # self.error = self.set_point - current_value
        if self.set_point < 0:
            self.set_point = (360 + self.set_point)  # -1*

        self.error = self.set_point - current_value

        if self.error < 0:
            self.error = 360 + self.error

        if self.error > 180:
            self.error = - 360 + self.error

        if self.error  < -180:
        	#self.turning_right = True
        	self.error  = 360 - self.error

        elif self.error  > 180:
        	#self.turning_right = False
        	self.error  = self.error  - 360

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def update_speed(self, current_value):
        """
        Calculate PID output value for given reference input and feedback
        """
        self.error = self.set_point - current_value




        t = time.time()
        self.P_value = self.Kp * self.error #* (t - self.t_1)
        self.D_value =  self.Kd * ((self.error - self.Derivator) / (t - self.t_1))
        self.Derivator = self.error


        # if self.set_point_1>self.sets_point and self.error > 0:
            # self.Integrator= -1* self.Integrator
            # self.set_point_1 = self.set_point - 1
        # if self.Integrator > self.Integrator_max:
        #     self.Integrator = self.Integrator_max
        # elif self.Integrator < self.Integrator_min:
        #     self.Integrator = self.Integrator_min
        # print(((self.set_point - self.set_point_1)*0.5+self.set_point_1))
        PID = self.P_value + self.D_value
        if self.set_point_1<self.set_point:
            if ((self.set_point - self.set_point_1)*0.5+self.set_point_1)<current_value:
                self.Integrator = self.Integrator + self.error
                self.I_value = self.Integrator *  self.Ki * (t - self.t_1)
                # print(self.Integrator)
                PID = self.P_value  + self.D_value + self.I_value
            # elif ((self.set_point - self.set_point_1)*0.5+self.set_point_1)<current_value:
        elif self.set_point_1>self.set_point:
            if ((self.set_point_1 - self.set_point)*0.5+self.set_point)>current_value:
                self.Integrator = self.Integrator + self.error
                self.I_value = self.Integrator * self.Ki * (t - self.t_1)
                # print(self.Integrator)
                PID = self.P_value  + self.D_value + self.I_value



        self.t_1 = t
        return PID
    def update_rudder(self, current_value):
        """
        Calculate PID output value for given reference input and feedback
        """
        self.error = self.set_point - current_value

        if self.error>180:
            self.error = -abs(360-abs(self.error))
        if self.error<-180:
            self.error = abs(360-abs(self.error))




        if np.sign(self.Integrator*self.error)==0:
            self.Integrator=0

        # print(self.error)
        t = time.time()
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ((self.error - self.Derivator)/(t - self.t_1 ))
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        # if self.Integrator > self.Integrator_max:
        #     self.Integrator = self.Integrator_max
        # elif self.Integrator < self.Integrator_min:
        #     self.Integrator = self.Integrator_min

        self.I_value =  self.Integrator * self.Ki * (t - self.t_1 )

        PID = self.P_value  + self.D_value #+ self.I_value
        self.t_1 = t
        return PID

    def setPoint(self, set_point):
        """
		Initilize the setpoint of PID
		"""
        # self.set_point = set_point
        if self.set_point != set_point:
            self.set_point_1 = self.set_point
            self.set_point = set_point
            # if self.set_point_1>self.set_point:
            #     self.Integrator = -0.5* self.Integrator
            # if self.set_point_1 < self.set_point:
            #     self.Integrator = 0.5 * self.Integrator
            # self.Integrator = 0
            self.Derivator = 0
            self.Integrator_1 = self.Integrator

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator
