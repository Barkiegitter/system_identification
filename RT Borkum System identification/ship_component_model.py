#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 31 12:04:43 2020

@author: erwinlodder
"""
import pickle
import time
import struct
import matplotlib.pyplot as plt
import datetime
import numpy as np
class CANTimedFrame:
    """ Wraps a can frame and assigns attributes based on """
    
    def __init__(self, can_frame):
        self.can_frame = can_frame
        self.time = time.time()

    def __str__(self):
        return str(self.time) + str(self.can_frame)
        # return f"CAN {self.name}, content = {pretty_hex(bytes(self.can_frame.data))}; {values_str}"

target = './can2nmea/logged_frames_borkum_09_12_model_az/251598624.pkl'
target_1 = './can2nmea/logged_frames_borkum_09_12_model_az/251605040.pkl'

import os
# scores = {} # scores is an empty dict already
start = time.time()
if os.path.getsize(target) > 0:      
    with open(target, "rb") as f:
        unpickled = []
        while True:
            try:
                (az_current, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[2:4])
                (az_setting, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[:2])
                # (az, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[:2])

                # unpickled.append([datetime.datetime.fromtimestamp(pickle.load(f).time), pickle.load(f).can_frame.data])
                unpickled.append([datetime.datetime.fromtimestamp(pickle.load(f).time), az_current, az_setting])
                
            except EOFError:
                break
        end_time = time.time()
        
import os
# scores = {} # scores is an empty dict already
start = time.time()
if os.path.getsize(target_1) > 0:      
    with open(target_1, "rb") as f:
        unpickled_1 = []
        while True:
            try:
                (az_current, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[2:4])
                (az_setting, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[:2])
                # (az, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[:2])

                # unpickled.append([datetime.datetime.fromtimestamp(pickle.load(f).time), pickle.load(f).can_frame.data])
                unpickled_1.append([datetime.datetime.fromtimestamp(pickle.load(f).time), az_current, az_setting])
                
            except EOFError:
                break
        end_time = time.time()

# plt.plot([i[0] for i in unpickled] ,[i[6] for i in [k[1] for k in unpickled]])
figsize = (15, 9 * 9 / 16)
fig, ax1 = plt.subplots(figsize=figsize)
# ax1.plot([i[0] for i in unpickled_1], [i[1] for i in unpickled_1])
# ax1.plot([i[0] for i in unpickled_1], [i[2] for i in unpickled_1])
# plt.savefig('azblabla.png')
# #rpm
# target = './logged_frames_borkum_09_12/251605296.pkl'
# target_1 = './logged_frames_borkum_09_12/251605552.pkl'

# import os
# # scores = {} # scores is an empty dict already
# start = time.time()
# if os.path.getsize(target) > 0:      
#     with open(target, "rb") as f:
#         unpickled = []
#         while True:
#             try:
#                 (rpm_current, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[2:4])
#                 (rpm_setting, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[0:2])
#                 # (az, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[:2])

#                 # unpickled.append([datetime.datetime.fromtimestamp(pickle.load(f).time), pickle.load(f).can_frame.data])
#                 unpickled.append([datetime.datetime.fromtimestamp(pickle.load(f).time), rpm_current, rpm_setting])
                
#             except EOFError:
#                 break
#         end_time = time.time()

# if os.path.getsize(target_1) > 0:      
#     with open(target_1, "rb") as f:
#         unpickled_1 = []
#         while True:
#             try:
#                 (rpm_current_1, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[2:4])
#                 (rpm_setting_1, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[0:2])
#                 # (az, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[:2])

#                 # unpickled.append([datetime.datetime.fromtimestamp(pickle.load(f).time), pickle.load(f).can_frame.data])
#                 unpickled_1.append([datetime.datetime.fromtimestamp(pickle.load(f).time), rpm_current_1, rpm_setting_1])
                
#             except EOFError:
#                 break
#         end_time = time.time()


# unpickled = unpickled[:400]
# figsize = (15, 9 * 9 / 16)
# fig, ax1 = plt.subplots(figsize=figsize)
# ax1.plot([i[0] for i in unpickled], [i[1] for i in unpickled])
# ax1.plot([i[0] for i in unpickled_1], np.asarray([i[2] for i in unpickled_1]))
# # plt.plot([i[0] for i in unpickled] ,[i[5] for i in [k[1] for k in unpickled]])

# # plt.show()
# plt.savefig('rpmblabla.png')
time = np.asarray([i[0] for i in unpickled_1][60:])
real = np.asarray([i[1] for i in unpickled_1][60:]) #/600
target = np.asarray([i[2] for i in unpickled_1][60:])#/600 + 8.8

#%%

import numpy as np
from scipy import signal
from scipy.integrate import odeint

# tau * dy2/dt2 + 2*zeta*tau*dy/dt + y = Kp*u
Kp = 2.0    # gain
tau = 1.0   # time constant
zeta = 1.0 # damping factor
theta = 0.0 # no time delay
du = 1.0    # change in u

def model3(x,t):
    y = x[0]
    dydt = x[1]
    dy2dt2 = (-2.0*zeta*tau*dydt - y + Kp*du)/tau**2
    return [dydt,dy2dt2]
t3 = np.linspace(0,25,100)
x3 = odeint(model3,[0,0],t3)
y3 = x3[:,0]

sim_rpm = []

old_target_k = 0

old_position = 180
old_position_1 = 180

tau = 0.45  # time constant  for rpm
zeta = 1.5 # damping factor for rpm
theta = -0.4 # no time delay for rpm
du = 1.0    # change in u

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
            self.Kp = 4
            self.current_pos = 4
            self.current_pos_1 = 4
            self.input_hold = 4
            
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
        
        self.time_remaining = self.time_remaining - dt
        return self.current_pos




# tau = 0.35   # time constant
# zeta = 1.8 # damping factor
# theta = -0.1 # no time delay
# du = 1.0    # change in u

# tau = 0.45  # time constant  for hdg
# zeta = 1.0 # damping factor for hdg
# theta = -0.01 # no time delay for hdg
# du = 1.0    # change in u

tau = 0.95  # time constant  for hdg
zeta = 1.25 # damping factor for hdg
theta = -0.1 # no time delay for hdg
du = 1.0    # change in u

k = 180
input_hold = 0.0
time_remaining = theta
# model time delay
# put into manoeuver model:
#     autotune
component_function = component(tau, zeta, theta, 'rudder')
for i in range(1,len(time)):
    dt = (time[i] - time[i-1]).total_seconds()
    sim_rpm.append(component_function.update_component_pos(target[i], dt))
    
    # if old_target_k!=target[i]:
    #     time_remaining = theta
    #     input_hold = target[i]
    #     old_target_k = target[i]
    # if time_remaining<0.0:   
    #     k = input_hold    
    # Kp = k
    # def model3(x,t):
    #     y = x[0]
    #     dydt = x[1]
    #     dy2dt2 = (-2.0*zeta*tau*dydt - y + Kp*du)/tau**2
    #     return [dydt,dy2dt2]
    # # dy = real[i]- real[i-1]
    # dy = old_position - old_position_1
    # dt = time[i] - time[i-1]
    
    # dy_dt = dy/dt.total_seconds()
    # print(dy_dt)
    # t3 = np.asarray([0,dt.total_seconds()])
    # rpm_sim = odeint(model3,[old_position,dy_dt],t3)
    # sim_rpm.append(rpm_sim[-1,0])
    # old_position_1 = old_position
    # old_position = rpm_sim[-1,0]
    # time_remaining = time_remaining - dt.total_seconds()
    

       # voor de delta t die is verstreken met terugwerkende kracht berekenen wat het had moeten zijn.      

    
ax1.plot(real[1:])
ax1.plot(target[1:])
ax1.plot(sim_rpm)    
    
plt.savefig('az_model_1.png')
    
    
    
    




