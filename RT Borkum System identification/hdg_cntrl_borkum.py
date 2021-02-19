#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 29 16:32:59 2020

@author: erwinlodder
"""
import numpy as np
import time
from manoeuvre_model_borkum import ship_model
from ship_class import ship
from pid_small import PID
import matplotlib.pyplot as plt
ship = ship()
coef_ = np.genfromtxt('borkum_general.csv', delimiter=',')


# P_range = np.arange(0,10.2,0.2)
# I_range = np.arange(0,10.2,0.2)
# D_range = np.arange(0,10.2,0.2)

#initialise ship
u, v, r, hdg = 2,0,0,0
ship_model = ship_model(0,0,0, ship, coef_)
max_rpm_second = 1.2
rpm = 0 

# class MA_filter:
#     def __init__(self, periods):
#         self.filter_array = np.zeros(periods)    
#     def filter(self, input_):
#         self.filter_array = np.roll(self.filter_array, 1)
#         self.filter_array[0] = input_
#         return np.average(self.filter_array)

# (speed, time)
input_sequence = [(0, 0), (40, 20), (350, 140), (10, 250), (50, 450)]
end_input = 0
hdg_position = 0
current_hdg_setting = 0
t = 0
dt = 0.4 #future: add noise
t_end = 600
pid_hdg = PID(P=1.0, I=0.0, D=1., Ibounds_speed=(-90,90)) # -0.04646
max_deg_second = 15
#
u_ref = []
u_real = []
rpm_set = []
periods = 1

while t<t_end:
    # calculate speed input
    if end_input==0:
        if input_sequence[hdg_position][1] <= t:
            
            current_hdg_setting = input_sequence[hdg_position][0]
            pid_hdg.setPoint_hdg(current_hdg_setting)
            hdg_position += 1
            if hdg_position==len(input_sequence):
                end_input  = 1
        
        
    control_input = pid_hdg.update(hdg,dt)
    
    sign_hdg_diff = np.sign(current_hdg_setting - hdg)
    # print(control_input)
    if abs(current_hdg_setting - hdg)>180.:
        # print('a')
        control_input = control_input*-1
        # print(control_input)
    sign_control_input = np.sign(control_input)
    if abs(control_input)>35.:
        control_input = sign_control_input * 35
        
    
    
    
    
    
    
    rpm=10.
    rpm_0, rpm_1, rpm_2 = rpm, rpm-10, rpm-10
    
    # print(control_input)
    rsa_1, rsa_2 = 180, 180
    rsa_0 = 180 + control_input
    
    
    if abs(control_input-rsa_0)/dt>max_deg_second:
        
        rsa_0 = dt*sign_control_input*max_deg_second + rsa_0
        # if abs(rpm)>abs(pid_speed.Integrator_max):
        #     rpm = np.sign(rpm)*abs(pid_speed.Integrator_max)
            
            
    u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_model.manoeuvre_model_borkum(u, v, r, hdg,
                                                                                                   rpm_0, rpm_1, rpm_2,
                                                                                                   rsa_0,rsa_1,rsa_2,#rudder 
                                                                                                   dt,
                                                                                                   )
    
    u_ref.append(current_hdg_setting)
    u_real.append(hdg)
    rpm_set.append(rpm)
    t = t + dt

u_ref_array = np.asarray(u_ref)
u_real_array = np.asarray(u_real)

u_score = abs(u_ref_array-u_real_array)
u_score = u_score.sum()

print(u_score)


plt.plot(u_ref)
plt.plot(u_real)
# plt.plot(rpm_set)

plt.savefig('tuner_borkum.png')
plt.show()










