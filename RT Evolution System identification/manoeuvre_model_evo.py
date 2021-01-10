import numpy as np
from ship_class import ship
import math
# import warnings
# warnings.filterwarnings('error')

# read coef from csv
from ship_class import ship
coef_ = np.genfromtxt('foo_evo_general.csv', delimiter=',')


from ship_components_model import component



class ship_model:
    def __init__(self, u_dot, v_dot, r_dot, ship, coef_):
        self.u_dot_old = 0
        self.v_dot_old = 0
        self.r_dot_old = r_dot
        self.u_dot = u_dot
        self.v_dot = v_dot
        self.r_dot = r_dot
        self.r_dot_array = np.zeros(10)
        self.ship =  ship
        self.coef_ = coef_
        # self.acc_lim = acc_lim
        # print(self.acc_lim)
        
        self.az_0 = component(0.95, 1.25, -0.1, 'rudder')
        self.az_1 = component(0.95, 1.25, -0.1, 'rudder')
        self.az_2 = component(0.95, 1.25, -0.1, 'rudder')
        
        self.throttle_0 = component(0.45, 1.5, -0.1, 'throttle')
        self.throttle_1 = component(0.45, 1.5, -0.1, 'throttle')
        self.throttle_2 = component(0.45, 1.5, -0.1, 'throttle')
        
        
    def thrust_cone(self, x_eng, y_eng, az_eng, cone_deg, flow_distance, x_down, y_down):
        #check axis!!!!!!! coherent to choosen axis system
        top_corner = [y_eng, x_eng]
        right_corner = [y_eng + np.sin(np.deg2rad(az_eng - cone_deg/2.))*flow_distance, x_eng + np.cos(np.deg2rad(az_eng - cone_deg/2.))*flow_distance]
        left_corner = [y_eng + np.sin(np.deg2rad(az_eng + cone_deg/2.))*flow_distance, x_eng + np.cos(np.deg2rad(az_eng + cone_deg/2.))*flow_distance]
        triangle_list = [top_corner, right_corner, left_corner]
        x1, y1, x2, y2, x3, y3, xp, yp = top_corner[0], top_corner[1], right_corner[0], right_corner[1], left_corner[0], left_corner[1], y_down, x_down
        c1 = (x2 - x1) * (yp - y1) - (y2 - y1) * (xp - x1)
        c2 = (x3 - x2) * (yp - y2) - (y3 - y2) * (xp - x2)
        c3 = (x1 - x3) * (yp - y3) - (y1 - y3) * (xp - x3)
        if (c1 < 0 and c2 < 0 and c3 < 0) or (c1 > 0 and c2 > 0 and c3 > 0):
        # if max([pos[0] for pos in triangle_list])>=y_down>=min([pos[0] for pos in triangle_list]) and max([pos[1] for pos in triangle_list])>=x_down>=min([pos[1] for pos in triangle_list]):
            return (1 - np.deg2rad(abs((self.azimuth([y_eng, x_eng],[y_down,x_down])-az_eng))))
        else:
            return 0
    
    def azimuth(self, point1, point2):
        angle = np.arctan2(point2[0] - point1[0], point2[1] - point1[1])
        return np.degrees(angle) if angle >= 0 else np.degrees(angle) + 360
        #add thrust fraction here
    
    def thruster_interaction_coefficient(self, x_eng, y_eng, az_eng, cone_deg, flow_distance, x_down, y_down, az_down):  # give engine number: self.shipclass engine number
        thrust_cone_boolean = self.thrust_cone(x_eng, y_eng, az_eng, cone_deg, flow_distance, x_down, y_down)
        x_d_ratio = np.sqrt((x_down-x_eng)**2+abs(y_down-y_eng)) / self.ship.D_p
        t_engine = (1 - 0.75**(x_d_ratio**(2/3)))/(thrust_cone_boolean)
        # print(abs(az_eng-az_down), 1)
        t = t_engine+(1-t_engine)*(((abs(az_eng-az_down))**3)/(((130.)/(t_engine**3)) + ((abs(az_eng-az_down))**3)))
        if math.isnan(t):
            return 0
        else:
            return 1-t


    def acc_avenger(self, acc):
        self.r_dot_array = np.roll(self.r_dot_array, 1)
        self.r_dot_array[0] = acc
        return np.average(self.r_dot_array)

    def acc_correction(self, u_dot, v_dot, r_dot ):
        delta_u_dot = u_dot - self.u_dot_old
        delta_v_dot = v_dot - self.v_dot_old
        delta_r_dot = r_dot - self.r_dot_old
        
        #u
        if u_dot>0.0:
            if self.acc_lim[0][1]>delta_u_dot:
                u_dot = u_dot + np.sign(delta_u_dot)*self.acc_lim[0][1]
            if self.acc_lim[0][0]<u_dot:
                u_dot = self.acc_lim[0][0]
            
                
        if u_dot<0.0:       
            if self.acc_lim[1][1]>delta_u_dot:
                u_dot = u_dot + np.sign(delta_u_dot)*self.acc_lim[0][1]
            if self.acc_lim[1][0]>u_dot:
                u_dot = self.acc_lim[1][0]
        #v
        if self.acc_lim[2][1]<abs(delta_v_dot):
            v_dot = v_dot + np.sign(delta_v_dot)*self.acc_lim[2][1]
        if self.acc_lim[2][0]<abs(v_dot):
            v_dot = self.acc_lim[2][0]
        
        #r
        if self.acc_lim[3][1]<abs(delta_r_dot):
            r_dot = r_dot + np.sign(delta_r_dot)*self.acc_lim[3][1]
        if self.acc_lim[3][0]<abs(r_dot):
            r_dot = self.acc_lim[3][0]
        return u_dot, v_dot, r_dot
        
        
    def az_map(self, angle):
        if angle>360.0:
            angle = angle%360.
        elif -360.0<angle<-0.0:
            angle = angle + 360
        elif angle<-360.:
            angle = -1*(abs(angle)%360)
        return angle
    def manoeuvre_model_rt_evolution(self, u, v, r, heading, rpm_0, rpm_1, rpm_2, rsa_0, rsa_1, rsa_2, dt):  #rpm in per second!
        
        rsa_0 = self.az_map(rsa_0)
        rsa_1 = self.az_map(rsa_1)
        rsa_2 = self.az_map(rsa_2)
        #, v, r, heading, rpm_0, rpm_1, rpm_2, rsa_0, rsa_1, rsa_2, dt)
        rsa_0 = self.az_0.update_component_pos(rsa_0, dt)
        rsa_1 = self.az_1.update_component_pos(rsa_1, dt)
        rsa_2 = self.az_2.update_component_pos(rsa_2, dt)
        
        rpm_0 = self.throttle_0.update_component_pos(rpm_0, dt)
        rpm_1 = self.throttle_1.update_component_pos(rpm_1, dt)
        rpm_2 = self.throttle_2.update_component_pos(rpm_2, dt)
        
        
        # print(rsa_1, rpm_1)
        
        # rsa_0, rsa_1, rsa_2 = np.rad2deg(rsa_0),np.rad2deg(rsa_1),np.rad2deg(rsa_2)
        u_a_2 = (1-self.ship.w)*((u+r*abs(self.ship.y_2))*-1*np.cos(np.deg2rad(rsa_2)) + (v+r*abs(self.ship.x_2))*-1*np.sin(np.deg2rad(rsa_2))) #(1-ship.w)*
        u_a_1 = (1-self.ship.w)*((u-r*abs(self.ship.y_1))*-1*np.cos(np.deg2rad(rsa_1)) + (-v-r*abs(self.ship.x_1))*-1*np.sin(np.deg2rad(rsa_1))) #(1-ship.w)*
        u_a_0 = (1-self.ship.w)*((u)*-1*np.cos(np.deg2rad(rsa_0)) + ((-v + r*abs(self.ship.x_0))*-1*np.sin(np.deg2rad(rsa_0))) ) #(1-ship.w)*

        beta_2 = np.rad2deg(np.arctan((u_a_2)/(0.7*np.pi*rpm_2*self.ship.D_p)))
        # print('a', beta_2)
        if beta_2<0:
            beta_2 = beta_2 + 360.
            
        elif math.isnan(beta_2):
            beta_2 = 0
    
        beta_1 = np.rad2deg(np.arctan((u_a_1)/(0.7*np.pi*rpm_1*self.ship.D_p)))
        if beta_1 < 0:
            beta_1 = beta_1 + 360.
        elif math.isnan(beta_1):
            beta_1 = 0
    
        beta_0 = np.rad2deg(np.arctan((u_a_0)/(0.7*np.pi*rpm_0*self.ship.D_p)))
        if beta_0 < 0:
            beta_0 = beta_0 + 360.
        elif math.isnan(beta_0):
            beta_0 = 0

        # first engine listed experiences thrust decrease, t_21 means thrust reduction ratio due to downstream flow caused by engine 1
        t_21_phi = self.thruster_interaction_coefficient(self.ship.x_1, self.ship.y_1, rsa_1, 25.0, 100.0, self.ship.x_2, self.ship.y_2, rsa_2)
        t_20_phi = self.thruster_interaction_coefficient(self.ship.x_0, self.ship.y_0, rsa_0, 25.0, 100.0, self.ship.x_2, self.ship.y_2, rsa_2)
        f_p_4Q_2 = 1.*((1 - self.ship.t) * self.ship.beta_coef(beta_2) * 0.5 * self.ship.rho * ((((1-self.ship.w)*u_a_2)**2) + (0.7 * np.pi * rpm_2 * self.ship.D_p) ** 2)) * np.pi / 4 * self.ship.D_p ** 2
    
        t_12_phi = self.thruster_interaction_coefficient(self.ship.x_2, self.ship.y_2, rsa_2, 25.0, 100.0, self.ship.x_1, self.ship.y_1, rsa_1)
        t_10_phi = self.thruster_interaction_coefficient(self.ship.x_0, self.ship.y_0, rsa_0, 25.0, 100.0, self.ship.x_1, self.ship.y_1, rsa_1)
        f_p_4Q_1 = 1.* ((1 - self.ship.t) * self.ship.beta_coef(beta_1) * 0.5 * self.ship.rho * ((((1-self.ship.w)*u_a_1)**2) + (0.7 * np.pi * rpm_1 * self.ship.D_p) ** 2)) * np.pi / 4 * self.ship.D_p ** 2
    # *(1 - t_12_phi) * (1 - t_10_phi) 
        t_02_phi = self.thruster_interaction_coefficient(self.ship.x_2, self.ship.y_2, rsa_2, 25.0, 100.0, self.ship.x_0, self.ship.y_0, rsa_0)
        t_01_phi = self.thruster_interaction_coefficient(self.ship.x_1, self.ship.y_1, rsa_1, 25.0, 100.0, self.ship.x_0, self.ship.y_0, rsa_0)
        f_p_4Q_0 = 1.* ((1 - self.ship.t) * self.ship.beta_coef(beta_0) * 0.5 * self.ship.rho * ((((1-self.ship.w)*u_a_0)**2) + (0.7 * np.pi * rpm_0 * self.ship.D_p) ** 2)) * np.pi / 4 * self.ship.D_p ** 2
       
        # df_main['f_p_40_0'] = 1.0*((1-ship.t)*ship.beta_coef(df_main.beta_0)*0.5*ship.rho*(((((1-ship.w)*df_main.u)**2)+(0.7*np.pi*df_main.rpm_0*ship.D_p)**2))*np.pi/4*(ship.D_p**2))#.rolling(20).mean()  #(1-df_main['t_02_phi'])*(1-df_main['t_01_phi'])*
        # print(beta_0, f_p_4Q_0)
       # df_main['f_p_40_0'] = 0.6*((1-self.ship.t)*self.ship.beta_coef(df_main.beta_0)*0.5*self.ship.rho*(((((1-self.ship.w)*df_main.u_a_0)**2)+(0.7*np.pi*df_main.rpm_0*self.ship.D_p)**2))*np.pi/4*self.ship.D_p**2)  #(1-df_main['t_02_phi'])*(1-df_main['t_01_phi'])*

        # *(1 - t_02_phi) * (1 - t_01_phi)
        # print(f_p_4Q_0)
                      # ((1 - self.ship.w) * u_a_0) ** 2 +          
        # print(beta_0, f_p_4Q_0, u_a_0, rsa_0)
        # if rpm_0<0.1: 
        # f_p_4Q_0, f_p_4Q_1, f_p_4Q_2 = 0, 0, 0

                            
        # f_p_4Q_2, f_p_4Q_1, f_p_4Q_0 = 0,0,0
        # f_p_4Q_0 = (1 - t_02_phi) * (1 - t_01_phi) * (1 - self.ship.t) * self.ship.beta_coef(beta_0) * 0.5 * self.ship.rho * ((((1 - self.ship.w) * u_a_0) ** 2) + ((0.7 * np.pi * rpm_0 * self.ship.D_p) ** 2)) * np.pi / 4 * (self.ship.D_p ** 2)

        # precalculate all values raised to powers or those with less than 5 chained multiplications (don't use numpy for this part)
        # https://stackoverflow.com/questions/18453771/why-is-x3-slower-than-xxx/18453999#18453999
    
        # i_t = 1
        # X = np.concatenate(
        #     [u_dot, u * v, u * r, u * u * r, u * u * v, v * v * v, r * r * r, r * r * v, v * v * r, abs(v) * v, abs(r) * v,
        #      r * abs(v), abs(r) * r], axis=1)
    
        # u2 = u * u 
        # u3 = u * u * u
        # v2 = v*v
        # v3 = v * v * v
        # r2 = r*r
        # r3 = r * r * r
        # rvu = r * v * u
        abs_v = abs(v)
        abs_r = abs(r)
        
        # self.u_dot_old, self.v_dot_old, self.r_dot_old = self.u_dot, self.v_dot, self.r_dot
        # surge
        # surge_partial_force = ( self.coef_[0][0] * self.u_dot +
        #                       self.coef_[0][1] * u * v +
        #                       self.coef_[0][2] * u * r +
        #                       self.coef_[0][3] * (u2) * r +
        #                       self.coef_[0][4] * (u2) * v +
        #                       self.coef_[0][5] * (v3) +
        #                       self.coef_[0][6] * r3 +
        #                       self.coef_[0][7] * (r2) * v +
        #                       self.coef_[0][8] * (v2) * r +
        #                       self.coef_[0][9] * abs_v * v +
        #                       self.coef_[0][10] * abs_r * v +
        #                       self.coef_[0][11] * abs_v * r +
        #                       self.coef_[0][12] * abs_r * r
        #                       )
        surge_partial_force = (#self.coef_[0][0] * self.u_dot +
                               self.coef_[0][1] * u +
                               self.coef_[0][2] * u * u +
                               self.coef_[0][3] * u*u*u +
                               self.coef_[0][4] * v*v +
                               self.coef_[0][5] * r*r +
                               self.coef_[0][6] * v*r +
                                self.coef_[0][7] * u*v*v +
                                self.coef_[0][8] * r*v*u +
                                self.coef_[0][9] * u*r*r  
                                # self.coef_[0][10] * np.deg2rad(rsa_0)*np.deg2rad(rsa_0) + 
                                # self.coef_[0][11] * np.deg2rad(rsa_1)*np.deg2rad(rsa_1) +
                                # self.coef_[0][12] * np.deg2rad(rsa_2)*np.deg2rad(rsa_2) +
                                # self.coef_[0][13] * np.deg2rad(rsa_0)*r*u + 
                                # self.coef_[0][14] * np.deg2rad(rsa_1)*r*u+
                                # self.coef_[0][15] * np.deg2rad(rsa_2)*r*u
                               )
                               
        self.u_dot = (surge_partial_force + r * v * self.ship.Mass + 1 *(-1*np.cos(np.deg2rad(rsa_0))*(f_p_4Q_0)-np.cos(np.deg2rad(rsa_1))*(f_p_4Q_1)-np.cos(np.deg2rad(rsa_2))*(f_p_4Q_2)))/ (self.ship.Mass-self.coef_[0][0])
        if abs(self.u_dot)<0.00001:
            self.u_dot = 0.0
        
        # print(u, surge_partial_force,-1*np.cos(np.deg2rad(rsa_0))*(f_p_4Q_0)-np.cos(np.deg2rad(rsa_1))*(f_p_4Q_1)-np.cos(np.deg2rad(rsa_2))*(f_p_4Q_2))
        sway_partial_force = (#self.coef_[1][0] * self.v_dot +
                              self.coef_[1][1] * v +
                              self.coef_[1][2] * v * v +
                              self.coef_[1][3] * u * v +
                              self.coef_[1][4] * u * r +
                              self.coef_[1][5] * u * u * r +
                              self.coef_[1][6] * u * u * v +
                              self.coef_[1][7] * v * v * v +
                              self.coef_[1][8] * r*r*r +
                              self.coef_[1][9] * r*r*v +
                               self.coef_[1][10] * v*v*r +
                               self.coef_[1][11] * abs_v * v +
                               self.coef_[1][12] * abs_r * v +
                               self.coef_[1][13] * abs_v * r +
                               self.coef_[1][14] * abs_r * r        
                              )

        self.v_dot = (sway_partial_force - self.ship.Mass * r * u + 1 * (-np.sin(np.deg2rad(rsa_0))*(f_p_4Q_0)-np.sin(np.deg2rad(rsa_1))*(f_p_4Q_1)-np.sin(np.deg2rad(rsa_2))*(f_p_4Q_2)))/(self.ship.Mass-self.coef_[1][0])
        if abs(self.v_dot)<0.00001:
            self.v_dot = 0.0
        # print(self.coef_[2][0])
        force_partial = (#self.coef_[2][0] * self.r_dot +
                         self.coef_[2][1] * r +
                         self.coef_[2][2] * r * r +
                         self.coef_[2][3] * v * r +
                         self.coef_[2][4] * u * r +
                         self.coef_[2][5] * u*u*r +
                         self.coef_[2][6] * u*u*v +
                         self.coef_[2][7] * v*v*v +
                         self.coef_[2][8] * r*r*r +
                         self.coef_[2][9] * r*r*v +
                         self.coef_[2][10] * v*v*r +
                          self.coef_[2][11] * abs_v * v +
                          self.coef_[2][12] * abs_r * v +
                          self.coef_[2][13] * abs_v * r +
                          self.coef_[2][14] * abs_r * r   
                            # self.coef_[2][15] * np.deg2rad(rsa_0)*np.deg2rad(rsa_0)*u + 
                            # self.coef_[2][16] * np.deg2rad(rsa_1)*np.deg2rad(rsa_1)*u +
                            # self.coef_[2][17] * np.deg2rad(rsa_2)*np.deg2rad(rsa_2)*u 
                            # self.coef_[2][15] * np.deg2rad(rsa_0)*r*u + 
                            # self.coef_[2][16] * np.deg2rad(rsa_1)*r*u+
                            # self.coef_[2][17] * np.deg2rad(rsa_2)*r*u
                          )
     
    
        self.r_dot = (force_partial + 1 * (self.ship.x_0*-1*np.sin(np.deg2rad(rsa_0))*(f_p_4Q_0) + self.ship.x_2*-1*np.sin(np.deg2rad(rsa_2))*(f_p_4Q_2) + self.ship.x_1*-1*np.sin(np.deg2rad(rsa_1))*(f_p_4Q_1) - self.ship.y_2*-1*np.cos(np.deg2rad(rsa_2))*(f_p_4Q_2) - self.ship.y_1*-1*np.cos(np.deg2rad(rsa_1))*(f_p_4Q_1)))/ (self.ship.I_e -self.coef_[2][0] )
        if abs(self.r_dot)<0.000001:
            self.r_dot = 0.0
        # self.r_dot = self.acc_avenger(self.r_dot_temp)
        # Calculating distance covered during this time iterval (using velocity from last interval)
        
        self.r_dot_old = self.r_dot
        
        # self.u_dot, self.v_dot, self.r_dot = self.acc_correction(self.u_dot, self.v_dot, self.r_dot)
        
        # next_u = u*19/20 + (self.u_dot * dt)/20
        # next_v = v*19/20 + (self.v_dot * dt)/20
        # next_r = r*19/20 + (self.r_dot * dt)/20
        
        next_u = u + (self.u_dot * dt)
        next_v = v + (self.v_dot * dt)
        next_r = r + (self.r_dot * dt)
        
        # print(self.u_dot, self.v_dot, self.r_dot)
        
        
        
        delta_x_0 = (u * np.sin(np.deg2rad(heading)) + v * np.cos(np.deg2rad(heading))) * dt
    
        delta_y_0 = (u * np.cos(np.deg2rad(heading)) - v * np.sin(np.deg2rad(heading))) * dt
        delta_r_0 = r * dt  # radians turned in the existing time step
        
        next_heading = heading + np.rad2deg(delta_r_0)
        if next_heading>360.:
            next_heading = next_heading-360.
        if next_heading<-0.0:
            next_heading = next_heading+360
        # self.u_dot_1 = self.u_dot
        # print(next_u, self.u_dot, delta_y_0, surge_partial_force,  f_p_4Q_0)
        # v_accel = (sway_partial_force - self.ship.Mass * r * u - 1 * (
        #             - np.sin(np.deg2rad(rsa_0)) * abs(f_p_4Q_0) - np.sin(np.deg2rad(rsa_1)) * abs(
        #         f_p_4Q_1) - np.sin(np.deg2rad(rsa_2)) * abs(f_p_4Q_2))) / (self.ship.Mass)

        # print(rsa_0)
        # print(self.r_dot)
        check=0
        return next_u, next_v, next_r, next_heading, delta_x_0, delta_y_0, delta_r_0, self.u_dot, self.v_dot, self.r_dot
        # return next_u, next_v, next_r
        # except RuntimeError:
        #     check=1
        #     return 0, 0, 0, 0, 0, 0, 0, self.u_dot, self.v_dot, self.r_dot, check
    
    