import numpy as np
from scipy.integrate import odeint


class component:
    def __init__(self, tau, zeta, theta, component):
        self.tau = tau
        self.zeta = zeta
        self.theta = theta
        self.time_remaining = self.theta
        
        self.old_target = 0.0
        

        if component=='rudder':
            self.Kp = 180
            self.current_pos = 180
            self.current_pos_1 = 180
            self.input_hold = 180
            
        if component=='throttle':
            self.Kp = 0
            self.current_pos = 0
            self.current_pos_1 = 0
            self.input_hold = 0
            
    def component_ODE_function(self, x,t):
        y = x[0]
        dydt = x[1]
        dy2dt2 = (-2.0*self.zeta*self.tau*dydt - y + self.Kp)/self.tau**2
        return [dydt,dy2dt2]
    
    
    def update_component_pos(self, new_input, dt):
        
        if self.old_target!=new_input:
            self.time_remaining = self.theta
            self.old_target = new_input
            self.input_hold = new_input
            
        if self.time_remaining<0.0:
            self.Kp = self.input_hold
        
        
        # print(self.Kp)
        dy = self.current_pos - self.current_pos_1
        dy_dt = dy/dt
        # print(dy)
        t_local = np.asarray([0, dt])
        sim_component = odeint(self.component_ODE_function, [self.current_pos, dy_dt], t_local)
        self.current_pos_1 = self.current_pos
        self.current_pos = sim_component[-1,0]
        # print(self.current_pos)
        self.time_remaining = self.time_remaining - dt
        return self.current_pos
            
            

        
        
     