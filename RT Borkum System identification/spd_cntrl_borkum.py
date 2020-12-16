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

x = [-74457.11750483877, -10490.738952651574, -1187.8191439559441, -2457.801066996641, -2989.673950423439, -61068.77423530965, 133997.70949859568, -2838.947202522788, 12746.238573746034, -25594.850129087274, 258012.0744622793, 12810.641665534094, 2932.3533460067806, 40.61065106134689, 203554.21212032178, -2049.3920629119107, 2967.9544105649766, -34713.994179741734, 48190.86303828821, -276358.68064971006, 17105.962093648748, 26400.168281351453, 169431.45044537893, -16217.828243249993, -9565.466664165391, 14183974.355539804, 323267.90458734083, 63803.018535682524, 156209.8091029468, 98237.91744594082, -84066.03752057311, -3521.5959712115728, -231731.96146913688, 2623958.409839695, -159711.45483799872, -102801.89242951664, 235203.6012087705, -320188.94928284414, 75099.62906765344, -1080256.4149495093]
coef_ = np.array([np.asarray(x[:10]),np.asarray(x[10:25]),np.asarray(x[25:40])])
# P_range = np.arange(0,10.2,0.2)
# I_range = np.arange(0,10.2,0.2)
# D_range = np.arange(0,10.2,0.2)

#initialise ship
u, v, r, hdg = 0,0,0,0
ship_model = ship_model(0,0,0, ship, coef_)
max_rpm_second = 0.5
rpm = 0 




# (speed, time)
speed_u = [(0, 0), (2.5,10), (1.1, 140), (3.5, 250), (0, 450)]
end_input = 0
speed_u_position = 0
current_speed_setting = 0
t = 0
dt = 0.4 #future: add noise
t_end = 400
pid_speed = PID(P=2., I=.8, D=1.0) # -0.04646

#
u_ref = []
u_real = []
rpm_set = []
while t<t_end:
    # calculate speed input
    if end_input==0:
        if speed_u[speed_u_position][1] <= t:
            
            current_speed_setting = speed_u[speed_u_position][0]
            
            pid_speed.setPoint_speed(current_speed_setting)
            speed_u_position += 1
            
            if speed_u_position==len(speed_u):
                end_input  = 1
        
        
    
    control_input = pid_speed.update(u,dt)
    # print(u, control_input)
    sign_control_input = np.sign(control_input)
    # calculate speed control iput
    if abs(abs(control_input)-abs(rpm))/dt>max_rpm_second:
        
        rpm = dt*sign_control_input*max_rpm_second + rpm
        # if abs(rpm)>abs(pid_speed.Integrator_max):
        #     rpm = np.sign(rpm)*abs(pid_speed.Integrator_max)
    # print(u)
    # print(rpm)
    # model
    # if rpm<-0.5:
    #     rpm = -0.5
    
    rpm= control_input
    # print(control_input)
    if rpm>15.5:
        rpm = 15.5
    
    
    if rpm>0.0:
        rpm_1, rpm_2, rpm_3 = rpm+8.8, rpm+8.8, rpm+8.8
    elif rpm<=0.0:
        rpm_1, rpm_2, rpm_3 = 0, 0, 0
    
    # print(control_input)
    # print(u)
    u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_model.manoeuvre_model_borkum(u, v, r, hdg,
                                                                                                   rpm_1, rpm_2, rpm_3,
                                                                                                   180,180,180,#rudder 
                                                                                                   dt,
                                                                                                   )
    # print(u, rpm_1, rpm_2)
    
    u_ref.append(current_speed_setting)
    u_real.append(u)
    rpm_set.append(rpm)
    t = t + dt

u_ref_array = np.asarray(u_ref)
u_real_array = np.asarray(u_real)

u_score = abs(u_ref_array-u_real_array)
u_score = u_score.sum()

# print(u_score)


plt.plot(u_ref)
plt.plot(u_real)
# plt.plot(rpm_set)

plt.savefig('tuner_borkum.png')
plt.show()










