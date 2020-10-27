#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 14:51:20 2020

@author: erwinlodder
"""
import numpy as np
from ship_class import ship
coef_ = np.genfromtxt('foo_rpa3.csv', delimiter=',')

ship = ship()

class ship_model:
    def __init__(self, u_dot, v_dot, r_dot):
        self.u_dot = u_dot
        self.v_dot = v_dot
        self.r_dot = r_dot
    def manoeuvre_model_rpa_3(self, u, v, r, heading,rpm,rsa, dt):

        beta= np.rad2deg(np.arctan((u)/(0.7*np.pi*rpm*ship.D_p)))
        if beta>360.0:
            beta=beta-360
        elif beta<0.0:
            beta = beta + 360
        
        f_p_4Q = (1-ship.t)*ship.beta_coef(beta)*0.5*ship.rho*(((((1-ship.w)*u)**2)+ (0.7*np.pi*rpm*ship.D_p)**2))*np.pi/4*ship.D_p**2
        
        F_r = -21.1* ship.A_r*u*u*rsa

        
        u2 = u ** 2
        u3 = u * u * u
        v2 = v ** 2
        v3 = v * v * v
        r2 = r ** 2
        r3 = r * r * r
        rvu = r * v * u
        abs_v = abs(v)
        abs_r = abs(r)
        # surge
        # surge_partial_force = ( coef_[0][0] * self.u_dot +
        #                       coef_[0][1] * u * v +
        #                       coef_[0][2] * u * r +
        #                       coef_[0][3] * (u2) * r +
        #                       coef_[0][4] * (u2) * v +
        #                       coef_[0][5] * (v3) +
        #                       coef_[0][6] * r3 +
        #                       coef_[0][7] * (r2) * v +
        #                       coef_[0][8] * (v2) * r +
        #                       coef_[0][9] * abs_v * v +
        #                       coef_[0][10] * abs_r * v +
        #                       coef_[0][11] * abs_v * r +
        #                       coef_[0][12] * abs_r * r
        #                       )
        surge_partial_force = (#coef_[0][0] * self.u_dot +
                               coef_[0][1] * u * u +
                               coef_[0][2] * u*u*u +
                               coef_[0][3] * u*v +
                               coef_[0][4] * u*r +
                               coef_[0][5] * v*v +
                               coef_[0][6] * r2 +
                               coef_[0][7] * v*r +
                               coef_[0][8] * u*v*v +
                               coef_[0][9] * r*v*u +
                               coef_[0][10] * u*r*r 
                               # coef_[1][9] * np.cos(np.rad2deg(rsa_0)) + 
                               # coef_[1][10]* np.cos(np.rad2deg(rsa_0)) + 
                               # coef_[1][11] * np.cos(np.rad2deg(rsa_0))
                               )


    
        self.u_dot = (surge_partial_force + r * v * ship.Mass + 1.*f_p_4Q + F_r*np.sin(np.deg2rad(rsa)))/ (ship.Mass- coef_[0][0])
    
    
        # print(f'u {u}, v {v}, r {r} , surge_1_5 {surge_partial_force} )
    
        sway_partial_force = (#coef_[1][0] * self.v_dot +
                              coef_[1][1] * v +
                              coef_[1][2] * u * v +
                              coef_[1][3] * u * r +
                              coef_[1][4] * (u2) * r +
                              coef_[1][5] * (u2) * v +
                              coef_[1][6] * (v3) +
                              coef_[1][7] * r3 +
                              coef_[1][8] * (r2) * v +
                              coef_[1][9] * (v2) * r +
                              coef_[1][10] * abs_v * v +
                              coef_[1][11] * abs_r * v +
                              coef_[1][12] * abs_v * r +
                              coef_[1][13] * abs_r * r 
                              # coef_[1][13] * np.sin(np.rad2deg(rsa_0)) + 
                              # coef_[1][14] * np.sin(np.rad2deg(rsa_0)) + 
                              # coef_[1][15] * np.sin(np.rad2deg(rsa_0))
                                       
                              )
        # Y = np.concatenate([v_dot_spec,v,v*v, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1)


        self.v_dot = (sway_partial_force - ship.Mass * r * u + F_r*np.cos(np.radians(rsa)))/(ship.Mass-coef_[1][0])
    
        force_partial = (#coef_[2][0] * self.r_dot +
                         coef_[2][1] * r +
                         coef_[2][2] * u*v +
                         coef_[2][3] * u * r +
                         coef_[2][4] * (u2) * r +
                         coef_[2][5] * (u2) * v +
                         coef_[2][6] * (v3) +
                         coef_[2][7] * r3 +
                         coef_[2][8] * (r2) * v +
                         coef_[2][9] * (v2) * r +
                         coef_[2][10] * abs_v * v +
                         coef_[2][11] * abs_r * v +
                         coef_[2][12] * abs_v * r +
                         coef_[2][13] * abs_r * r )
# N = np.concatenate([r_dot,r, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1)


        self.r_dot = (force_partial - F_r*ship.x_r*np.cos(np.radians(rsa)))/ (ship.I_e- coef_[2][0] )
        # self.r_dot = self.acc_avenger(self.r_dot_temp)
        # Calculating distance covered during this time iterval (using velocity from last interval)
        
        
        # if self.u_dot>0.35:
        #     self.u_dot=0.35
        
        # if self.u_dot<-0.0653:
        #     self.u_dot=-0.0653
            
        # if abs(self.u_dot)>0.5:
        #     self.u_dot=0.5*np.sign(self.u_dot)
        # if abs(self.v_dot)>0.3:
        #     self.v_dot=0.3*np.sign(self.v_dot)
        # if abs(self.r_dot)>0.04:
        #     self.r_dot=0.04*np.sign(self.r_dot)
        # Calculating new velocities
        next_u = u + self.u_dot * dt
        next_v = v + self.v_dot * dt
        next_r = r + self.r_dot * dt
        
        print( rsa, F_r*np.sin(np.deg2rad(rsa)))
        
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
        # v_accel = (sway_partial_force - ship.Mass * r * u - 1 * (
        #             - np.sin(np.deg2rad(rsa_0)) * abs(f_p_4Q_0) - np.sin(np.deg2rad(rsa_1)) * abs(
        #         f_p_4Q_1) - np.sin(np.deg2rad(rsa_2)) * abs(f_p_4Q_2))) / (ship.Mass)

        # print(rsa_0)
        # print(self.r_dot)
        return next_u, next_v, next_r, next_heading, delta_x_0, delta_y_0, delta_r_0, self.u_dot, self.v_dot, self.r_dot
        # return next_u, next_v, next_r
    